# Examples of basic methods for simulation competition
import numpy as np
import matplotlib.pyplot as plt
import time
import cv2
from queue import PriorityQueue


# The available ground truth state measurements can be accessed by calling sensor_data[item]. All values of "item" are provided as defined in main.py lines 296-323. 
# The "item" values that you can later use in the hardware project are:
# "x_global": Global X position
# "y_global": Global Y position
# "range_down": Downward range finder distance (Used instead of Global Z distance)
# "range_front": Front range finder distance
# "range_left": Leftward range finder distance 
# "range_right": Rightward range finder distance
# "range_back": Backward range finder distance
# "roll": Roll angle (rad)
# "pitch": Pitch angle (rad)
# "yaw": Yaw angle (rad)

# Global variables
on_ground = True
height_desired = 1.0
timer = None
startpos = None
timer_done = None

init = False
t = 0

STATE_ON_GROUND = 0
STATE_FORWARD = 1
STATE_SEARCH_PAD = 2
STATE_RETURN = 3
STATE_STOP = 4
STATE_LAND = 5
STATE_DEPART = 6
STATE_ASTAR_FOLLOW = 7
STATE_RETURN_FOLLOW = 8

CRUISE_HEIGHT = 0.5
LANDING_AREA_LIMIT = 3.6 # meter, to do check
FORWARD_OBJECTIVE = 5.0 # meter
RETURN_OBJECTIVE = 0.
MAX_X     = 5.0 # meter
MAX_Y     = 3.0 # meter
RANGE_MAX = 2.0 # meter, maximum range of distance sensor
RES_POS   = 0.05 # meter
CONF      = 0.2 # certainty given by each measurement
GRID_SEARCH_SPACING = 0.2  # meter
LANDING_PAD_HEIGHT = 0.1
SETPOINT_DIST_THRESHOLD = 0.08
ATTRACTIVE_GAIN  = 0.7
REPULSIVE_GAIN   = 0.47
REPULSIVE_RADIUS = 0.7

KP = .4
MAX_VEL = 1

YAW_RATE = 1.5


# This is the main function where you will implement your control algorithm
def get_command(sensor_data, camera_data, dt):
    global init, control

    if not init:
        control = Control()
        init = True

    # Open a window to display the camera image
    # NOTE: Displaying the camera image will slow down the simulation, this is just for testing
    #cv2.imshow('Camera Feed', camera_data)
    #cv2.waitKey(1)

    return control.get_command(sensor_data, camera_data, dt)

# Function to detect pink squares
def detect_pink_square(frame):
    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define lower and upper bounds for pink color
    lower_pink = np.array([140, 50, 50])
    upper_pink = np.array([180, 255, 255])
    
    # Threshold the HSV image to get only pink colors
    mask = cv2.inRange(hsv, lower_pink, upper_pink)
    

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Iterate through detected contours
    for contour in contours:
        # Approximate the contour to a polygon
        approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)
        
        # If the polygon has 4 vertices, it's likely a square
        if len(approx) == 4:
            return contour
    
    return None


class Control:
    def __init__(self):
        self.timer = None
        self.startpos = None
        self.timer_done = None
        self.state = STATE_ON_GROUND
        self.backward_y_error = 0
        self.backward_x_error = 0
        self.stop_counter = .8
        self.setpoints = self.generate_grid(3.7, 0.1, 1.5, 3.0)
        self.setpoint_index = 0
        self.land_counter = .8
        self.depart_counter = 0.0
        self.astar_counter = 0
        self.astar_path = None
        self.astar_path__ = None
        self.security_counter = 0

        self.map = np.zeros((int(MAX_X/RES_POS), int(MAX_Y/RES_POS))) # 0 = unknown, 1 = free, -1 = occupied
        self.map_astar = np.copy(self.map)
        self.t = 0

    def get_command(self, sensor_data, camera_data, dt):
        # Take off
        #print(f"state: {self.state}")
        if self.startpos is None:
            self.startpos = [sensor_data['x_global'], sensor_data['y_global']]#, sensor_data['z_global']]

        self.occupancy_map(sensor_data)

        current_pos = np.array([sensor_data['x_global'], sensor_data['y_global']]) #, sensor_data['yaw']])

        if self.state == STATE_ON_GROUND:
            if sensor_data['range_down'] > (0.9*CRUISE_HEIGHT - 0.1):
                self.state = STATE_FORWARD
                print("change state ", self.state)
            
            yaw_rate = YAW_RATE if sensor_data['range_down'] > 0.4*CRUISE_HEIGHT else 0

            control_command = [0.0, 0.0, CRUISE_HEIGHT, yaw_rate]
            return control_command

        # print("ciclo")
        if self.state == STATE_FORWARD:
            if current_pos[0] > LANDING_AREA_LIMIT: 
                control_command = [0.0, 0.0, CRUISE_HEIGHT, 0.0]
                self.state = STATE_SEARCH_PAD
                print("change state search pad")
                return control_command
            
            cmd = self.calculate_navigation_direction_forward(self.map, FORWARD_OBJECTIVE, current_pos)
            cmd = self.rotate(cmd[0], cmd[1],-sensor_data['yaw'])

            #print(current_pos, cmd)
            cmd = (cmd/np.linalg.norm(cmd)) * min(MAX_VEL, KP*np.linalg.norm(cmd))


            if min(MAX_VEL, KP*np.linalg.norm(cmd)) == MAX_VEL:
                print("limiting")
            control_command = [cmd[0], cmd[1], CRUISE_HEIGHT, YAW_RATE]
            return control_command
        
        if self.state == STATE_SEARCH_PAD:
            start = self.point_to_map_cell(sensor_data['x_global'], sensor_data['y_global'])
            astar_goal = self.point_to_map_cell(self.setpoints[self.setpoint_index][0], self.setpoints[self.setpoint_index][1])
            #astar_goal = (astar_goal[0]-1, astar_goal[1])


            if self.setpoint_remove(self.map_astar, np.array(astar_goal)):
                while self.setpoint_remove(self.map_astar, np.array(astar_goal)):
                    print("skipping setpoint")
                    self.setpoint_index += 1
                    start = self.point_to_map_cell(sensor_data['x_global'], sensor_data['y_global'])
                    astar_goal = self.point_to_map_cell(self.setpoints[self.setpoint_index][0], self.setpoints[self.setpoint_index][1])

            if self.setpoint_index == len(self.setpoints) - 1:
                self.setpoint_index = 0
                return [0,0,CRUISE_HEIGHT,0]

            
            #start = (start[0]+3, start[1]+1)
            #print("start: ", start, " - map of start: ", self.map_astar[start[0], start[1]])
            #print("goal: ", astar_goal, " - map of goal: ", self.map_astar[astar_goal[0], astar_goal[1]])
            self.astar_path = self.a_star(self.map_astar, start, astar_goal)
            if self.astar_path is None:
                print("NO PATH FOUND")
                if self.setpoint_index < len(self.setpoints) - 1 and self.map_astar[start[0], start[1]] == 1:
                    self.setpoint_index += 1
                return [0.1,0.1,CRUISE_HEIGHT,0]
            #for _ in range(3):
            #    if len(self.astar_path) > 1:
            #        self.astar_path.pop()
            
            self.astar_path__ = self.astar_path.copy()

            #print("PATH: ", self.astar_path)

            for i in range(len(self.astar_path)):
                if self.map_astar[self.astar_path[i][0], self.astar_path[i][1]] != 1:
                    print("PATH GENERATED PASSES THROUGH OCCUPIED CELL")
                    #return [0.1,0.1,CRUISE_HEIGHT,0]
                
                self.astar_path[i] = self.map_cell_to_point(self.astar_path[i][0], self.astar_path[i][1])
    


            self.astar_counter = 0
            self.state = STATE_ASTAR_FOLLOW
            #return [0,0,CRUISE_HEIGHT,0]

        if self.state == STATE_ASTAR_FOLLOW:

            if abs(sensor_data['z_global'] - sensor_data['range_down']) > 0.08:
                print("landing pad FOUND!!!!")
                print("change state ", self.state)
                self.state = STATE_LAND
                return [0,0,CRUISE_HEIGHT,0]
            
            #print(" - - - - astar follow - - - - ")
            
            current_pos = (sensor_data['x_global'], sensor_data['y_global'])
            goal = np.array([self.astar_path[self.astar_counter][0], self.astar_path[self.astar_counter][1]])

            if np.linalg.norm(current_pos - goal) < 0.04 or self.security_counter > 150:
                #print("astar follow reached setpoint, next one - security counter: ", self.security_counter)
                self.astar_counter += 1
                self.security_counter = 0

            if self.astar_counter == len(self.astar_path):
                self.setpoint_index += 1
                self.state = STATE_SEARCH_PAD
                print("astar follow go back to search pad ", self.state)
                self.security_counter = 0
                return [0,0,CRUISE_HEIGHT,0]
        
            self.security_counter += 1
            v = np.array([goal[0] - current_pos[0], goal[1] - current_pos[1]])
            cmd = 0.2 * v / np.linalg.norm(v)


            goal_cell = self.point_to_map_cell(goal[0], goal[1])
            #print("current pos: ", current_pos)
            #print("goal: ", goal, " - astar_map[goal]: ", self.map_astar[goal_cell[0],goal_cell[1]])
            #print("cmd: ", cmd)
            
            cmd = self.rotate(cmd[0], cmd[1],-sensor_data['yaw'])

            control_command = [cmd[0], cmd[1], CRUISE_HEIGHT, YAW_RATE*0.65]
            
            return control_command
        
        if self.state == STATE_LAND:
            # print("LANDING")

            if abs(sensor_data['z_global'] - sensor_data['range_down']) < 0.08:
                print("landing pad LOST!!!!")
                print("change state ", self.state)
                self.state = STATE_ASTAR_FOLLOW
          
            if self.land_counter > -0.01:
               self.land_counter -= 0.0006
            else:
                self.state = STATE_DEPART


            if self.land_counter > CRUISE_HEIGHT:
                cmd = [0.0, 0.0, CRUISE_HEIGHT, 0.0]
            else:
                cmd = [0.0, 0.0, self.land_counter, 0.0]
            #print(current_pos, cmd)

            return cmd
        

        
        if self.state == STATE_DEPART:
            #print("DEPARTING")

            if self.depart_counter < CRUISE_HEIGHT:
                self.depart_counter += 0.0006
            else:
                self.state = STATE_RETURN
            cmd = [0.0, 0.0, self.depart_counter, 0.0]
            
            return cmd
        





        if self.state == STATE_RETURN:

            start = self.point_to_map_cell(sensor_data['x_global'], sensor_data['y_global'])
            astar_goal = self.point_to_map_cell(self.startpos[0], self.startpos[1])

            print("start: ", start, " - map of start: ", self.map_astar[start[0], start[1]])
            print("goal: ", astar_goal, " - map of goal: ", self.map_astar[astar_goal[0], astar_goal[1]])
            self.astar_path = self.a_star(self.map_astar, start, astar_goal)
            if self.astar_path is None:
                print("NO PATH FOUND")
                return [0,0,CRUISE_HEIGHT,0]
            self.astar_path__ = self.astar_path.copy()

            for i in range(len(self.astar_path)):
                if self.map_astar[self.astar_path[i][0], self.astar_path[i][1]] != 1:
                    print("CAZZO\n\n\nPATH GENERATED PASSES THROUGH OCCUPIED CELL")
                    #return [0.1,0.1,CRUISE_HEIGHT,0]
                
                self.astar_path[i] = self.map_cell_to_point(self.astar_path[i][0], self.astar_path[i][1])
    
            #print("PATH: ", self.astar_path)


            self.astar_counter = 0
            self.state = STATE_RETURN_FOLLOW

            return [0,0,CRUISE_HEIGHT,0] # [vx, vy, alt, yaw_rate]

        if self.state == STATE_RETURN_FOLLOW:            
            current_pos = (sensor_data['x_global'], sensor_data['y_global'])
            goal = np.array([self.astar_path[self.astar_counter][0], self.astar_path[self.astar_counter][1]])

            if np.linalg.norm(current_pos - goal) < 0.07 or self.security_counter > 150:
                #print("return follow reached setpoint, next one, security counter: ", self.security_counter)
                self.astar_counter += 1
                self.security_counter = 0

            if self.astar_counter == len(self.astar_path):
                self.setpoint_index += 1
                self.state = STATE_STOP
                self.security_counter = 0
                print("return follow go back to search pad ", self.state)
                return [0,0,CRUISE_HEIGHT,0]
        
            self.security_counter += 1
            v = np.array([goal[0] - current_pos[0], goal[1] - current_pos[1]])


            #goal_cell = self.point_to_map_cell(goal[0], goal[1])
            cmd = 0.2 * v / np.linalg.norm(v)
            #print("current pos: ", current_pos)
            #print("goal: ", goal, " - astar_map[goal]: ", self.map_astar[goal_cell[0],goal_cell[1]])
            #print("cmd: ", cmd)
            
            cmd = self.rotate(cmd[0], cmd[1],-sensor_data['yaw'])

            control_command = [cmd[0], cmd[1], CRUISE_HEIGHT, YAW_RATE*0.65]
            
            return control_command






        if self.state == STATE_STOP:

            if self.stop_counter > -0.01:
                self.stop_counter -= 0.0006

            if self.stop_counter > CRUISE_HEIGHT:
                cmd = [0.0, 0.0, CRUISE_HEIGHT, 0.0]
            else:
                cmd = [0.0, 0.0, self.stop_counter, 0.0]
            #print(current_pos, cmd)

            return cmd












        print("ERROR WRONG MODE")
        return [0,0,CRUISE_HEIGHT,0] # [vx, vy, alt, yaw_rate]
    
    def setpoint_remove(self, map, setpoint):
        if map[setpoint[0], setpoint[1]] != 1:
            return True
        return False

    def calculate_navigation_direction_forward(self, map, goal, current_pos):
        cmd_x = 1.25 * (goal - current_pos[0])/np.linalg.norm(goal-current_pos[0])

        cmd_y = 0

        for i in range(map.shape[0]):
            for j in range(map.shape[1]):
                dist = np.linalg.norm(np.array([i, j])* RES_POS - (current_pos + np.array([0.14, 0.0])))

                if dist < REPULSIVE_RADIUS and map[i, j] < 0:
                    b = .7
                    a = REPULSIVE_RADIUS
                    func = ((1.0 / (dist+b)) + (1.0 / a)) / ((dist+b)**2) - 1.08
                    func = func/2.75

                    if func < 0:
                        func = 0
                        print("FUNC < 0\n\n\n\n\n")
                        exit
                    rep = - map[i, j] * REPULSIVE_GAIN * func * (current_pos - RES_POS*np.array([i, j]))/np.linalg.norm(current_pos - RES_POS*np.array([i, j]))
                    cmd_y += rep[1]
                    cmd_x += rep[0]/3
        return np.array([cmd_x, cmd_y])

    def rotate(self, x, y, angle):
        x_prime = x * np.cos(angle) - y * np.sin(angle)
        y_prime = x * np.sin(angle) + y * np.cos(angle)
        return np.array([x_prime, y_prime])

    def occupancy_map(self, sensor_data):
        global t
        pos_x = sensor_data['x_global']
        pos_y = sensor_data['y_global']
        yaw = sensor_data['yaw']
        
        for j in range(4): # 4 sensors
            yaw_sensor = yaw + j*np.pi/2 #yaw positive is counter clockwise
            if j == 0:
                measurement = sensor_data['range_front']
            elif j == 1:
                measurement = sensor_data['range_left']
            elif j == 2:
                measurement = sensor_data['range_back']
            elif j == 3:
                measurement = sensor_data['range_right']
            
            for i in range(int(RANGE_MAX / RES_POS)): # range is 2 meters
                dist = i*RES_POS
                idx_x = int(np.round((pos_x + dist*np.cos(yaw_sensor))/RES_POS,0))
                idx_y = int(np.round((pos_y + dist*np.sin(yaw_sensor))/RES_POS,0))

                # make sure the current_setpoint is within the map
                if idx_x < 0 or idx_x >= self.map.shape[0] or idx_y < 0 or idx_y >= self.map.shape[1] or dist > RANGE_MAX:
                    break

                # update the map
                if dist < measurement:
                    self.map[idx_x, idx_y] += CONF

                    self.map_astar[idx_x, idx_y] += CONF
                else:
                    self.map[idx_x, idx_y] -= CONF

                    for dx in range(-2, 3):
                        for dy in range(-2, 3):
                            if 0 <= idx_x + dx < self.map.shape[0] and 0 <= idx_y + dy < self.map.shape[1]:
                                self.map_astar[idx_x + dx, idx_y + dy] -= CONF
                    break
        
        self.map = np.clip(self.map, -1, 1) # certainty can never be more than 100%
        self.map_astar = np.clip(self.map_astar, -1, 1) # certainty can never be more than 100%
        self.map[0, :] = -1
        self.map[-1, :] = -1
        self.map[:, 0] = -1
        self.map[:, -1] = -1
        self.map_astar[0, :] = -1
        self.map_astar[-1, :] = -1
        self.map_astar[:, 0] = -1
        self.map_astar[:, -1] = -1
        if self.t % 50 == 0:
            # np.savetxt("map.txt",np.flip(self.map,1))
            plt.imshow(np.flip(self.map,1), vmin=-1, vmax=1, cmap='gray', origin='lower') # flip the map to match the coordinate system
            plt.savefig("map.png")
            plt.close()
            # np.savetxt("map.txt",np.flip(self.map,1))
            plt.imshow(np.flip(self.map_astar,1), vmin=-1, vmax=1, cmap='gray', origin='lower') # flip the map to match the coordinate system
            
            
            
            if self.astar_path__ is not None:
                trajectory_x, trajectory_y = zip(*self.astar_path__)
                trajectory_y = tuple(map(lambda x: self.map_astar.shape[1] - x, trajectory_y))
                plt.plot(trajectory_y, trajectory_x, marker='o', linestyle='-', color='blue', label='Trajectory')


            plt.savefig("map_astar.png")
            plt.close()
            self.t = 0 
        self.t = 24
        # self.t += 1

    # # only plot every Nth time step (comment out if not needed)
        #print("occupancy",t)
        # if t % 50 == 0:
        #     plt.imshow(np.flip(self.map,1), vmin=0, vmax=1, cmap='gray', origin='lower') # flip the map to match the coordinate system
        #     plt.savefig("map.png")
        #     plt.close()
        # t +=1

    # return map

# def calculate_total_potential(goal_pos, current_pos):
#     attractive_pot = attractive_potential(goal_pos, current_pos)
#     repulsive_pot = repulsive_potential(current_pos)
#     return attractive_pot + repulsive_pot
    def generate_grid(self, start_x, start_y, width, height):
        setpoints = []
        #trajectory = []
        
        # Calculate the number of points in each direction
        num_points_x = int(width / GRID_SEARCH_SPACING)
        num_points_y = int(height / GRID_SEARCH_SPACING)
        #print("num points: ", num_points_x, num_points_y)
        # Generate setpoints
        for i in range(int(num_points_x/2)):
            for j in range(int(num_points_y)):
                x = start_x + 2 * i * GRID_SEARCH_SPACING
                y = start_y + j * GRID_SEARCH_SPACING
                #print(i,2*j, "-", x, y)
                setpoints.append((x, y))
                #trajectory.append((x, y))  # Add points to trajectory

            for j in range(num_points_y-1, -1, -1):
                x = start_x + (2 * i + 1) * GRID_SEARCH_SPACING
                y = start_y + j * GRID_SEARCH_SPACING
                setpoints.append((x, y))
                #print(i,2*j+1, "-", x, y)
                #trajectory.append((x, y))  # Add points to trajectory

        for j in range(num_points_y):
            x = start_x + 2 * (i + 1) * GRID_SEARCH_SPACING
            y = start_y + j * GRID_SEARCH_SPACING
            #print(i,2*j, "-", x, y)
            setpoints.append((x, y))
            #trajectory.append((x, y))  # Add points to trajectory

        return setpoints #, trajectory

    def point_to_map_cell(self, x, y):
        idx_x = int(x / RES_POS) 
        idx_y = int(y / RES_POS)

        return idx_x, idx_y
    
    def map_cell_to_point(self, cell_x, cell_y):
        return ((cell_x + 0.5)*RES_POS, (cell_y + 0.5)*RES_POS)




    def heuristic(self, a, b):
        """Calculate the Manhattan distance between two points a and b"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def a_star(self, grid, start, goal):
        #start = (start[0]-1, start[1])
        
        if grid[goal[0], goal[1]] != 1:
            print("start or goal not valid")
            self.setpoint_index += 1
            return None

        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        # Only include non-diagonal directions
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

        while not open_set.empty():
            current = open_set.get()[1]

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]  # Return reversed path

            for direction in directions:
                neighbor = (current[0] + direction[0], current[1] + direction[1])

                # Ensure neighbor is within grid bounds
                if 0 < neighbor[0] < grid.shape[0] and 0 < neighbor[1] < grid.shape[1]:
                    # Skip if neighbor is an obstacle or not traversable
                    if grid[neighbor[0], neighbor[1]] != 1:
                        continue

                    # Uniform cost for horizontal and vertical moves
                    move_cost = 1

                    tentative_g_score = g_score[current] + move_cost

                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                        open_set.put((f_score[neighbor], neighbor))

        return None  # Path not found
