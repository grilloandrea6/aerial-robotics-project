# Examples of basic methods for simulation competition
import numpy as np
import matplotlib.pyplot as plt
import time
import cv2

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
SETPOINT_REMOVE_THRESHOLD = 1.0 # TODO!!!!!!
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
        self.stop_counter = 1.0
        self.setpoints = self.generate_grid(3.65, 0.1, 1.5, 3.0)
        self.setpoint_index = 0
        self.land_counter = 1.0
        self.depart_counter = 0.0

        self.map = np.zeros((int(MAX_X/RES_POS)+2, int(MAX_Y/RES_POS)+2)) # 0 = unknown, 1 = free, -1 = occupied
        rows = len(self.map)
        cols = len(self.map[0])
        for j in range(cols):
            self.map[0][j] = -1
            self.map[rows - 1][j] = -1
        for i in range(rows):
            self.map[i][0] = -1
            self.map[i][cols - 1] = -1

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

        if self.state == STATE_FORWARD:
            if current_pos[0] > LANDING_AREA_LIMIT: 
                control_command = [0.0, 0.0, CRUISE_HEIGHT, 0.0]
                self.state = STATE_SEARCH_PAD
                print("change state ", self.state)
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
            if self.setpoint_remove(self.map, np.array(self.setpoints[self.setpoint_index])):
                print("skipping setpoint")
                self.setpoint_index += 1

            if self.setpoint_index == len(self.setpoints):
                self.state = STATE_RETURN
                print("change state ", self.state)
                return [0,0,CRUISE_HEIGHT,0]

            if np.linalg.norm(current_pos - np.array(self.setpoints[self.setpoint_index])) < SETPOINT_DIST_THRESHOLD:
                print("reached setpoint, going to next one")
                self.setpoint_index += 1

            if abs(sensor_data['z_global'] - sensor_data['range_down']) > 0.08:
                print("landing pad FOUND!!!!")
                print("change state ", self.state)
                self.state = STATE_LAND
                return [0,0,CRUISE_HEIGHT,0]
            
            # navigate towards the setpoint

            cmd = self.calculate_navigation_direction_search(self.map, self.setpoints[self.setpoint_index][0], current_pos, y_pos=self.setpoints[self.setpoint_index][1])
            cmd = self.rotate(cmd[0], cmd[1],-sensor_data['yaw'])

            cmd = (cmd/np.linalg.norm(cmd)) * min(MAX_VEL, KP*np.linalg.norm(cmd))


            if min(MAX_VEL, KP*np.linalg.norm(cmd)) == MAX_VEL:
                print("limiting")

            #print("current pos", current_pos, "current setpoint", self.setpoints[self.setpoint_index], "cmd", cmd)

            control_command = [cmd[0], cmd[1], CRUISE_HEIGHT, YAW_RATE]

            #self.state = STATE_RETURN
            #print("change state ", self.state)
            return control_command # [0,0,CRUISE_HEIGHT,0]
        
        if self.state == STATE_LAND:
            print("LANDING")

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
            print("DEPARTING")

            if self.depart_counter < CRUISE_HEIGHT:
               self.depart_counter += 0.0006
            else:
                self.state = STATE_RETURN

            cmd = [0.0, 0.0, self.depart_counter, 0.0]
            
            return cmd


        if self.state == STATE_RETURN:
            if np.linalg.norm(current_pos - self.startpos) < 0.06: 
                control_command = [0.0, 0.0, CRUISE_HEIGHT, 0.0]
                self.state = STATE_STOP
                print("change state ", self.state)
                return control_command
            
            cmd = self.calculate_navigation_direction_backward(self.map, self.startpos[0], current_pos, y_pos=self.startpos[1])
            cmd = self.rotate(cmd[0], cmd[1],-sensor_data['yaw'])

            #print(current_pos, cmd)
            cmd = (cmd/np.linalg.norm(cmd)) * min(MAX_VEL, KP*np.linalg.norm(cmd))


            if min(MAX_VEL, KP*np.linalg.norm(cmd)) == MAX_VEL:
                print("limiting")
            control_command = [cmd[0], cmd[1], CRUISE_HEIGHT, YAW_RATE]
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

        
        # vx,vy = self.rotate(navigation_direction[0],navigation_direction[1],-yaw)
        # #desired_vx, desired_vy, desired_alt, desired_yaw_rate = command[0], command[1], command[2], command[3]

        # control_command = [vx/1.5, vy/1.5, CRUISE_HEIGHT, 0.0]
        print("ERROR WRONG MODE")
        return [0,0,CRUISE_HEIGHT,0] # [vx, vy, alt, yaw_rate]
    
    def calculate_navigation_direction_forward(self, occupancy_map, goal, current_pos):
        cmd_x = 1.25 * (goal - current_pos[0])/np.linalg.norm(goal-current_pos[0])

        cmd_y = 0

        for i in range(occupancy_map.shape[0]):
            for j in range(occupancy_map.shape[1]):
                dist = np.linalg.norm(np.array([i+1, j+1])* RES_POS - (current_pos + np.array([0.14, 0.0])))

                if dist < REPULSIVE_RADIUS and occupancy_map[i, j] < 0:
                    b = .7
                    a = REPULSIVE_RADIUS
                    func = ((1.0 / (dist+b)) + (1.0 / a)) / ((dist+b)**2) - 1.08
                    func = func/2.75

                    if func < 0:
                        func = 0
                        print("FUNC < 0\n\n\n\n\n")
                        exit
                    rep = - occupancy_map[i, j] * REPULSIVE_GAIN * func * (current_pos - RES_POS*np.array([i+1, j+1]))/np.linalg.norm(current_pos - RES_POS*np.array([i+1, j+1]))
                    cmd_y += rep[1]
                    cmd_x += rep[0]/3
        return np.array([cmd_x, cmd_y])


    def calculate_navigation_direction_backward(self, occupancy_map, goal, current_pos, y_pos=None):
        cmd_x = 0.85 * (goal - current_pos[0])
        print("X - cmd_x: ", cmd_x)
        if np.linalg.norm(cmd_x) > 1.3:
            print("clipping return vel")
            cmd_x = 1.3 * np.sign(cmd_x)
        if np.linalg.norm(goal - current_pos[0]) < 0.45:
            print("add integral to x")
            self.backward_x_error += (goal - current_pos[0])
            #cmd_x = 0.6 * np.sign(cmd_x)
            cmd_x += 0.005 * self.backward_x_error
            print("X - integral term: ", 0.0015 * self.backward_x_error)
        

        cmd_y = 0
        if y_pos != None:
            
            cmd_y = (y_pos - current_pos[1]) * 0.85
            print("Y - cmd_y: ", cmd_y)
            if np.linalg.norm(y_pos - current_pos[1]) < 0.8 and np.linalg.norm(goal - current_pos[0]) < 0.45:
                self.backward_y_error += (y_pos - current_pos[1])
                cmd_y += 0.002 * self.backward_y_error
                print("Y - integral term: ", 0.0015 * self.backward_y_error)
        # if np.linalg.norm(cmd_y) < 0.6:
        #     print("aug return vel y")
        #     cmd_y = 0.6 * np.sign(cmd_y)

        for i in range(occupancy_map.shape[0]):
            for j in range(occupancy_map.shape[1]):
                dist = np.linalg.norm(np.array([i+1, j+1])* RES_POS - (current_pos - np.array([0.14, 0.0])))

                if dist < REPULSIVE_RADIUS and occupancy_map[i, j] < 0:
                    b = .7
                    a = REPULSIVE_RADIUS
                    func = ((1.0 / (dist+b)) + (1.0 / a)) / ((dist+b)**2) - 1.08
                    func = func/2.75

                    if func < 0:
                        func = 0
                        print("FUNC < 0\n\n\n\n\n")
                        exit
                    rep = - occupancy_map[i, j] * REPULSIVE_GAIN * func * (current_pos - RES_POS*np.array([i+1, j+1]))/np.linalg.norm(current_pos - RES_POS*np.array([i+1, j+1]))
                    cmd_y += rep[1] * .75
                    cmd_x += rep[0]/3
        print("X - cmd_x after repulsion: ", cmd_x)
        print("Y - cmd_y after repulsion: ", cmd_y)

        return np.array([cmd_x, cmd_y])


    def calculate_navigation_direction_search(self, occupancy_map, goal, current_pos, y_pos=None):
        cmd_x = 0.85 * (goal - current_pos[0])
        #print("X - cmd_x: ", cmd_x)
        if np.linalg.norm(cmd_x) > 1.3:
            #print("clipping return vel")
            cmd_x = 1.3 * np.sign(cmd_x)
        if np.linalg.norm(goal - current_pos[0]) < 0.45:
            #print("add integral to x")
            self.backward_x_error += (goal - current_pos[0])
            #cmd_x = 0.6 * np.sign(cmd_x)
            cmd_x += 0.005 * self.backward_x_error
            #print("X - integral term: ", 0.0015 * self.backward_x_error)
        

        cmd_y = 0
        if y_pos != None:
            
            cmd_y = (y_pos - current_pos[1]) * 0.85
            #print("Y - cmd_y: ", cmd_y)
            if np.linalg.norm(y_pos - current_pos[1]) < 0.8 and np.linalg.norm(goal - current_pos[0]) < 0.45:
                self.backward_y_error += (y_pos - current_pos[1])
                cmd_y += 0.002 * self.backward_y_error
                #print("Y - integral term: ", 0.0015 * self.backward_y_error)
        # if np.linalg.norm(cmd_y) < 0.6:
        #     print("aug return vel y")
        #     cmd_y = 0.6 * np.sign(cmd_y)

        for i in range(occupancy_map.shape[0]):
            for j in range(occupancy_map.shape[1]):
                dist = np.linalg.norm(np.array([i+1, j+1])* RES_POS - current_pos)
                                                                       
                if dist < REPULSIVE_RADIUS and occupancy_map[i, j] < 0:
                    b = .7
                    a = REPULSIVE_RADIUS
                    func = ((1.0 / (dist+b)) + (1.0 / a)) / ((dist+b)**2) - 1.08
                    func = func/2.85

                    if func < 0:
                        func = 0
                        print("FUNC < 0\n\n\n\n\n")
                        exit
                    rep = - occupancy_map[i, j] * REPULSIVE_GAIN * func * (current_pos - RES_POS*np.array([i+1, j+1]))/np.linalg.norm(current_pos - RES_POS*np.array([i+1, j+1]))
                    cmd_y += rep[1] * .75 / 2
                    cmd_x += rep[0]/3 / 2
        #print("X - cmd_x after repulsion: ", cmd_x)
        #print("Y - cmd_y after repulsion: ", cmd_y)

        return np.array([cmd_x, cmd_y])




    def setpoint_remove(self, occupancy_map, setpoint_pos):
        cmd = [0, 0]
        for i in range(occupancy_map.shape[0]):
            for j in range(occupancy_map.shape[1]):
                dist = np.linalg.norm(np.array([i+1, j+1])* RES_POS - setpoint_pos)

                if dist < REPULSIVE_RADIUS and occupancy_map[i, j] < 0:
                    b = .7
                    a = REPULSIVE_RADIUS
                    func = ((1.0 / (dist+b)) + (1.0 / a)) / ((dist+b)**2) - 1.08
                    func = func/2.75

                    if func < 0:
                        func = 0
                        print("FUNC < 0\n\n\n\n\n")
                        exit
                    rep = - occupancy_map[i, j] * REPULSIVE_GAIN * func * (setpoint_pos - RES_POS*np.array([i+1, j+1]))/np.linalg.norm(setpoint_pos - RES_POS*np.array([i+1, j+1]))
                    cmd += rep
        if np.linalg.norm(cmd) > SETPOINT_REMOVE_THRESHOLD:
            return True
        return False

    def dist(self, pos1, pos2):
        return np.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2)

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
                idx_x = int(np.round((pos_x + dist*np.cos(yaw_sensor))/RES_POS,0)) + 1
                idx_y = int(np.round((pos_y + dist*np.sin(yaw_sensor))/RES_POS,0)) + 1

                # make sure the current_setpoint is within the map
                if idx_x < 1 or idx_x >= (self.map.shape[0] - 1) or idx_y < 1 or idx_y >= (self.map.shape[1]-1) or dist > RANGE_MAX:
                    break

                # update the map
                if dist < measurement:
                    self.map[idx_x, idx_y] += CONF
                else:
                    self.map[idx_x, idx_y] -= CONF
                    break
        
        self.map = np.clip(self.map, -1, 1) # certainty can never be more than 100%
        # if self.t % 50 == 0:
        #     np.savetxt("map.txt",np.flip(self.map,1))
        #     plt.imshow(np.flip(self.map,1), vmin=-1, vmax=1, cmap='gray', origin='lower') # flip the map to match the coordinate system
        #     plt.savefig("map.png")
        #     plt.close()
        #     self.t = 0 
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



def clip_angle(angle):
    angle = angle%(2*np.pi)
    if angle > np.pi:
        angle -= 2*np.pi
    if angle < -np.pi:
        angle += 2*np.pi
    return angle
