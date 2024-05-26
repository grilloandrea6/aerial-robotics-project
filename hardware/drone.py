import matplotlib.pyplot as plt
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander
import time
import numpy as np

DPOS = 0.01                         # meters
LANDING_REGION_X = 3.6              # meters
LATERAL_SENSOR_THRESHOLD = 0.17     # meters
FRONT_SENSOR_THRESHOLD = 0.325      # meters


MAX_X     = 5.0                     # meters
MAX_Y     = 3.0                     # meters
RES_POS   = 0.01                    # meters

FIELD_SIZE_Y = MAX_Y                # meters
GRID_FORWARD_STEP = 0.3             # meters

ALPHA = 0.05                        # EMA filter parameter

FORWARD = 0
LEFT = 1
BACK = 2
RIGHT = 3

class Drone:
    def __init__(self, scf, home_position):
        print("Init drone - start")
        self.home_position = home_position
        self._x_off = self.home_position[0]
        self._y_off = self.home_position[1]
        self._scf = scf

        # Connection callback
        self._scf.cf.connected.add_callback(self._connected)
        self._scf.cf.disconnected.add_callback(self._disconnected)
        self._scf.cf.connection_failed.add_callback(self._connection_failed)
        self._scf.cf.connection_lost.add_callback(self._connection_lost)

        self.cruise_height = 0.30

        self._z = 0
        self.cross_active = False
        self.init_detection()
        self._init_logging()

        self.image_index = 0
        self.print_map = np.zeros((int(MAX_X/RES_POS), int(MAX_Y/RES_POS)))

        print("Init drone - finished")

    def run(self):
        print("Drone run - start")
        self.position_commander = MotionCommander(self._scf, default_height=self.cruise_height)
        self.position_commander.take_off(velocity=10.0)

        time.sleep(1.0)
        print("Position before forward:", self._x, self._y)
        self.forward()
    
        print("Forward ended")
        time.sleep(.5)
        self.init_detection()
        time.sleep(2.0)

        direction = self.grid_search()
        print("End grid: Direction: ", direction)
        self.cross(direction)
        print("Position before landing: ", self._x, self._y)
        self._x_off = self._x
        self._y_off = self._y

        # Do land in two steps to minimize drift
        self.position_commander.down(self.cruise_height/2)
        self.position_commander.land()
        time.sleep(1.0)

        # Take off
        self.position_commander.take_off(velocity=10.0)
        time.sleep(2.0)
        print("Position after taking off: ", self._x, self._y)
        direction = self.backward()
        print("End back: Direction: ", direction)
        time.sleep(1.0)

        if direction == -1:
            print("Position before spiral: ", self._x, self._y)
            direction = self.spiral()
            print("End spiral: Direction: ", direction)
        print("Position before cross: ", self._x, self._y)
        self.cross(direction)

        # Do land in two steps to minimize drift
        self.position_commander.down(self.cruise_height/2)
        self.position_commander.land()

        print("Drone run - finished")

    def forward(self):
        print("FORWARD - start")
        while self._x < LANDING_REGION_X:
            if self._front < FRONT_SENSOR_THRESHOLD:
                print("Obstacle detected in front")

                dir = np.sign(self._y - (FIELD_SIZE_Y / 2))  # 1 towards the right, -1 towards the left
                print("Going right" if dir > 0 else "Going left")

                while self._front < FRONT_SENSOR_THRESHOLD:
                    sens = self._right if dir > 0 else self._left
                    if sens < LATERAL_SENSOR_THRESHOLD:
                        print("Detected lateral obstacle, inverting direction")
                        dir = -dir
                        print("Going right" if dir > 0 else "Going left")
                        
                    self.position_commander.right(dir * DPOS)
                
                print("Obstacle in front not detected anymore, moving a little bit more")
                for _ in range(10):
                    self.position_commander.right(dir * DPOS)

            else:
                if self._left < 0.1:
                    print("Correction in forward - going right")
                    self.position_commander.right(DPOS)
                elif self._right < 0.1:
                    print("Correction in forward - going left")
                    self.position_commander.left(DPOS)

                self.position_commander.forward(DPOS)
        print("FORWARD - finished")

    def grid_search(self):
        print("GRID_SEARCH - start")

        # start to the right
        dir = 1
    
        s = LANDING_REGION_X+0.1
        step = GRID_FORWARD_STEP
        setpoints = [s]
        for i in range(10):
            setpoints.append(s + i*step)
        print("GS - setpoints: ", setpoints)

        print("GS: inizio con setpoint: ", setpoints[0])
        i=0
        while True: # not self.detected:  
            print("starting lateral with dir: ", dir)
            direction = self.lateral(setpoints[i], dir)
            if direction != -1:
                print("detected, finished grid search")
                return direction

            print("GS: arrived to the setpoint")
            
            i += 1
            # Invert dir
            dir = -dir
            while self._x < setpoints[i]: 
                if self._front < FRONT_SENSOR_THRESHOLD:
                    self.position_commander.right(dir * DPOS)
                    if(self.detected):
                        return RIGHT if dir > 0 else LEFT

                else:
                    self.position_commander.forward(DPOS)
                    if(self.detected):
                        return FORWARD
            
            while not (0.2 < self._y < FIELD_SIZE_Y-0.2):
                self.position_commander.right(dir * DPOS)
                if(self.detected):
                    return RIGHT if dir > 0 else LEFT
            print("GS: go with next setpoint: ", setpoints[i])
     
    def lateral(self, setpoint, dir_grid):
        print("LATERAL - start")

        myflag = 0
        lat_obstacle = False
        while 0.2 < self._y < FIELD_SIZE_Y-0.2:
            
            # No-obstacle mode resetting
            sens = self._back if self._back < self._front else self._front
            if sens > LATERAL_SENSOR_THRESHOLD:
                lat_obstacle = False

            # Go towards center
            if abs(self._x - setpoint) > 0.1 and not lat_obstacle and myflag<=0:
                print("Moving towards the setpoint")
                dir = np.sign(self._x - setpoint) 
                while np.abs(self._x - setpoint) > 0.1:
                    sens = self._back if dir > 0 else self._front
                    if sens < LATERAL_SENSOR_THRESHOLD:
                        print("Detected front/back obstacle, just go (lateral)")
                        lat_obstacle = True
                        break
                    self.position_commander.back(dir * DPOS)
                    if self.detected:
                        return BACK if dir == 1 else FORWARD
                

            sens = self._right if dir_grid == 1 else self._left
            # Obstacle avoidance
            if sens < FRONT_SENSOR_THRESHOLD:
                print("Obstacle detected in front (lateral)")
                dir = np.sign(self._x - setpoint)
                if dir == 0:
                    dir = 1
                sens2 = self._back if dir == 1 else self._front
                while sens < FRONT_SENSOR_THRESHOLD:
                    sens = self._right if dir_grid == 1 else self._left
                    sens2 = self._back if dir == 1 else self._front
                    if sens2 < LATERAL_SENSOR_THRESHOLD:
                        print("Detected lateral obstacle, inverting direction")
                        dir = -dir
                        print("Going back" if dir > 0 else "Going front")

                    self.position_commander.back(dir * DPOS)
                    if(self.detected):
                        return BACK if dir == 1 else FORWARD
                myflag=70
                
                print("Obstacle in front not detected anymore, moving a little bit more")
                for _ in range(2):
                    self.position_commander.right(dir * DPOS)
                    if(self.detected):
                        return RIGHT if dir == 1 else LEFT
            else:
                if self._front < 0.1:
                    print("correzione in lateral - going back")
                    self.position_commander.back(DPOS)
                    if(self.detected):
                        return BACK
                elif self._back < 0.1:
                    print("correzione in lateral - going forward")
                    self.position_commander.forward(DPOS)
                    if(self.detected):
                        return FORWARD

                myflag-=1
                self.position_commander.right(dir_grid * DPOS)
                if(self.detected):
                        return RIGHT if dir_grid == 1 else LEFT
        print("LATERAL - finished")
        return -1

    def cross(self, direction):
        self.map = 0.5 * np.ones((int(MAX_X/RES_POS), int(MAX_Y/RES_POS)))
        self.cross_active = True
        print("CROSS - start")

        # forward
        self.movement_cross(direction, 45)

        # left
        self.movement_cross((direction + 1) % 4, 45)

        # back
        self.movement_cross((direction + 2) % 4, 41.5)
        
        # right
        self.movement_cross((direction + 3) % 4, 95)

        self.cross_active = False

        coordinates = np.argwhere(self.map == 1)

        # Calculate the mean of the coordinates along each axis (rows and columns)
        average_coordinates = np.mean(coordinates, axis=0)

        mean_x, mean_y = self.map_cell_to_point(average_coordinates[0], average_coordinates[1])
        
        print("center of the cross: ", mean_x, mean_y)
        print("actual position: ", self._x, self._y)

        while(np.linalg.norm([mean_x - self._x, mean_y - self._y]) > 0.01):
            print("aligning")
            print("center of the cross: ", mean_x, mean_y)
            print("actual position: ", self._x, self._y)

            self.position_commander.move_distance(mean_x - self._x, mean_y - self._y, 0.0)
            time.sleep(0.2)
        print("FINAL - actual position: ", self._x, self._y)
        
        return

    def backward(self):
        print("BACKWARD - start")
        lat_obstacle = False
        myflag = 0
        detecting = False
        while self._x > self.home_position[0] or abs(self._y - self.home_position[1]) > 0.05:
            if not detecting and self._x - self.home_position[0] < 1.0:
                self.init_detection()
                time.sleep(2.0)
                detecting = True

            sens = self._right if self._right < self._left else self._left
            if sens > LATERAL_SENSOR_THRESHOLD:
                for _ in range(3):
                    self.position_commander.back(DPOS)
                    if self.detected and detecting:
                        return BACK
                lat_obstacle = False

            # Going towards center
            if abs(self._y - self.home_position[1]) > 0.1 and not lat_obstacle and myflag<=0:
                print("Moving towards the center")
                dir = np.sign(self._y - self.home_position[1])
                while np.abs(self._y - self.home_position[1]) > 0.1:

                    sens = self._right if dir > 0 else self._left
                    if sens < LATERAL_SENSOR_THRESHOLD:
                        print("Detected lateral obstacle, just go forward")
                        lat_obstacle = True
                        break
                    self.position_commander.right(dir * DPOS)
                    if self.detected and detecting:
                        return RIGHT if dir == 1 else LEFT
                

            # Oobstacle avoidance
            if self._back< FRONT_SENSOR_THRESHOLD:
                print("Obstacle detected in front")

                dir = np.sign(self._y - self.home_position[1])
                print("Going right" if dir > 0 else "Going left")

                while self._back < FRONT_SENSOR_THRESHOLD:
                    sens = self._right if dir > 0 else self._left
                    if sens < LATERAL_SENSOR_THRESHOLD:
                        print("Detected lateral obstacle, inverting direction")
                        dir = -dir
                        print("Going right" if dir > 0 else "Going left")
                        
                    self.position_commander.right(dir * DPOS)
                    if self.detected and detecting:
                        return RIGHT if dir == 1 else LEFT
                myflag=70
                
                print("Obstacle in front not detected anymore, moving a little bit more")
                for _ in range(2):
                    self.position_commander.right(dir * DPOS)
                    if self.detected and detecting:
                        return RIGHT if dir == 1 else LEFT
            else:
                myflag-=1

                if self._left < 0.1:
                    print("Correction in forward - going right")
                    self.position_commander.right(DPOS)
                    if self.detected and detecting:
                        return RIGHT
                elif self._right < 0.1:
                    print("Correction in forward - going left")
                    self.position_commander.left(DPOS)
                    if self.detected and detecting:
                        return LEFT

                self.position_commander.back(DPOS)
                if self.detected and detecting:
                    return BACK
        print("BACKWARD - finished")
        return -1
    
    def spiral(self):
        dist = 5
        while True:
            for dir in range(4):
                if self.movement_spiral(dir, dist):
                    return dir

                dist += 8

    def landing_pad_detected(self):
        self.filtered_z = ALPHA * self.filtered_z + (1 - ALPHA) * self._z
        undershoot = self.filtered_z < 0.27
        overshoot = self.filtered_z > 0.347

        if undershoot and (not self.old_undershoot) and not self.firstUndershootDetected:
            self.detected = True
            print("START DETECTING")
            self.firstUndershootDetected = True
        elif undershoot and (not self.old_undershoot) and self.firstUndershootDetected:
            self.firstUndershootDetected = False

        if overshoot and (not self.old_overshoot) and not self.firstOvershootDetected:
            self.firstOvershootDetected = True
        elif overshoot and (not self.old_overshoot) and self.firstOvershootDetected:
            self.detected = False
            print("STOP DETECTING")
            self.firstOvershootDetected = False
        
        self.old_overshoot = overshoot
        self.old_undershoot = undershoot  

        if self.cross_active and self.detected:
            x, y = self.point_to_map_cell(self._x, self._y)
            self.map[x, y] = 1
        elif self.cross_active:
            x, y = self.point_to_map_cell(self._x, self._y)
            self.map[x, y] = 0


        if self.cross_active and self.image_index % 80 == 0:
            plt.imshow(np.flip(self.map, 1), cmap='viridis', origin='lower')
            plt.savefig("./cross_map/file%05d.png" % (self.image_index/10))
            plt.close()


        x, y = self.point_to_map_cell(self._x, self._y)
        self.print_map[x, y] = 1
        if self.image_index % 100 == 0:
            plt.imshow(np.flip(self.print_map, 1), cmap='viridis', origin='lower')
            plt.savefig("./images/file%05d.png" % (self.image_index/10))
            plt.close()

        self.image_index += 1
        
    def init_detection(self):
        self.detected = False
        self.old_overshoot = False
        self.old_undershoot = False
        self.firstOvershootDetected = False
        self.firstUndershootDetected = False
        self.filtered_z = self._z

    def movement_cross(self, dir, dist):
        if dir == FORWARD:
            func = self.position_commander.forward
        elif dir == LEFT:
            func = self.position_commander.left
        elif dir == BACK:
            func = self.position_commander.back
        elif dir == RIGHT:
            func = self.position_commander.right
        
        func(dist * DPOS)
    
    def movement_spiral(self, dir, dist):
        if dir == FORWARD:
            func = self.position_commander.forward
        elif dir == LEFT:
            func = self.position_commander.left
        elif dir == BACK:
            func = self.position_commander.back
        elif dir == RIGHT:
            func = self.position_commander.right
        for _ in range(dist):
            func(DPOS)
            if self.detected:
                return True
        return False

    def _init_logging(self):
        # The definition of the logconfig can be made before connecting
        lpos = LogConfig(name='Position', period_in_ms=10)
        lpos.add_variable('stateEstimate.x')
        lpos.add_variable('stateEstimate.y')
        lpos.add_variable('stateEstimate.z')

        try:
            self._scf.cf.log.add_config(lpos)
            lpos.data_received_cb.add_callback(self.pos_data)
            lpos.error_cb.add_callback(self._log_error)
            lpos.start()

        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Position log config, bad configuration.')

        lmeas = LogConfig(name='Meas', period_in_ms=10)
        lmeas.add_variable('range.front')
        lmeas.add_variable('range.back')
        lmeas.add_variable('range.left')
        lmeas.add_variable('range.right')
        lmeas.add_variable('range.zrange')

        try:
            self._scf.cf.log.add_config(lmeas)
            lmeas.data_received_cb.add_callback(self.meas_data)
            lmeas.error_cb.add_callback(self._log_error)
            lmeas.start()

        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Measurement log config, bad configuration.')
        except Exception as e:
            print("Exception in start logging: ", e)
        return 
    
    def meas_data(self, timestamp, data, logconf):
        self._front  = self._convert_log_to_distance(data['range.front'])
        self._back   = self._convert_log_to_distance(data['range.back'])
        self._left   = self._convert_log_to_distance(data['range.left'])
        self._right  = self._convert_log_to_distance(data['range.right'])
        self._zrange = self._convert_log_to_distance(data['range.zrange'])
        #print("up: ", self._up, " front: ", self._front, " back: ", self._back, " left: ", self._left, " right: ", self._right, " zrange: ", self._zrange)
        
    def pos_data(self, timestamp, data, logconf):
        self._x = data['stateEstimate.x'] + self._x_off
        self._y = data['stateEstimate.y'] + self._y_off
        self._z = data['stateEstimate.z']
        self.landing_pad_detected()
        #print("x: ", self._x, " y: ", self._y, " z: ", self._z)

    def point_to_map_cell(self, x, y):
        idx_x = int(x / RES_POS) 
        idx_y = int(y / RES_POS)

        return idx_x, idx_y
    
    def map_cell_to_point(self, cell_x, cell_y):
        return ((cell_x + 0.5)*RES_POS, (cell_y + 0.5)*RES_POS)

    def _connected(self, URI):
        print('We are now connected to {}'.format(URI))

    def _log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Callback - Error when logging %s: %s' % (logconf.name, msg))
        return

    def _convert_log_to_distance(self, data):
        return data / 1000.0

    def _connection_failed(self, link_uri, msg):
        """
        Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)
        """
        print('Connection to %s failed: %s' % (link_uri, msg))
        return 

    def _connection_lost(self, link_uri, msg):
        """
        Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)
        """
        print('Connection to %s lost: %s' % (link_uri, msg))
        return 

    def _disconnected(self, URI):
        print('Disconnected')

    def _log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))
        return