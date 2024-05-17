from cflib.crazyflie.log import LogConfig
# from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.positioning.motion_commander import MotionCommander
import time
import numpy as np
import sys

DPOS = 0.01
DTIME = 0
LANDING_REGION_X = 4.0 # TODO
LATERAL_SENSOR_THRESHOLD = 0.25
FRONT_SENSOR_THRESHOLD = 0.325
FIELD_THRESHOLD = 0.2

FIELD_SIZE_Y = 3.0
GRID_FORWARD_STEP = 0.25

Z_ALPHA = 0.92 # Z EMA PARAMETER

LANDING_PAD_DETECTION_THRESHOLD = 0.018

class Drone:
    def run(self):
        print("Drone run - start")
        with MotionCommander(self._scf, default_height=0.3) as self.position_commander:
            # Take off and wait a second
            # time.sleep(6.0)
            # self.position_commander.forward(3)

            time.sleep(1.0)
            self.forward()
            return
            self.forward_bis()
            print("after forward bis!!")
            for _ in range(200):
                print("y", self._y)
                time.sleep(0.01)
            


            # Forward
            #self.forward()

            # Landing pad search
            #self.grid_search()
            # Land
            # for _ in range(20):
            #     self.position_commander.forward(DPOS)
            # while True:

            #     print(abs(self._zrange -  self._zrange_filt))
            #     if self.landing_pad_detected():
            #         break
            #     self.position_commander.forward(DPOS)


            # Take off

            # Return to home position

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
                    time.sleep(DTIME)
                
                print("Obstacle in front not detected anymore, moving a little bit more")
                for _ in range(2):
                    self.position_commander.right(dir * DPOS)
                    time.sleep(DTIME)

            else:
                self.position_commander.forward(DPOS)
                time.sleep(DTIME)
        print("FORWARD - finished")


    def forward_bis(self):
        print("GRID_SEARCH - start")
        lat_obstacle = False
        myflag = 0

        while self._x < LANDING_REGION_X:
            # parte che resetta la modalita' di NON ostacolo
            sens = self._right if self._right < self._left else self._left
            if sens > LATERAL_SENSOR_THRESHOLD:
                for _ in range(3):
                    self.position_commander.forward(DPOS)
                    time.sleep(DTIME)
                lat_obstacle = False

            # Parte che va al centro se non sono al centro
            if abs(self._y - (FIELD_SIZE_Y / 2)) > 0.1 and not lat_obstacle and myflag<=0:
                print("Moving towards the center")
                dir = np.sign(self._y - (FIELD_SIZE_Y / 2))
                while np.abs(self._y - (FIELD_SIZE_Y / 2)) > 0.1:
                    print("differenza dal centro", abs(self._y - (FIELD_SIZE_Y / 2)))
                    print("y", self._y)
                    sens = self._right if dir > 0 else self._left
                    if sens < LATERAL_SENSOR_THRESHOLD:
                        print("Detected lateral obstacle, just go forward")
                        lat_obstacle = True
                        break
                    self.position_commander.right(dir * DPOS)
                    time.sleep(DTIME)
                

            # Parte per obstacle avoidance
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
                    time.sleep(DTIME)
                    # print("differenza dal centro", abs(self._y - (FIELD_SIZE_Y / 2)))
                myflag=70
                
                print("Obstacle in front not detected anymore, moving a little bit more")
                for _ in range(2):
                    self.position_commander.right(dir * DPOS)
                    time.sleep(DTIME)
            # Parte che va a dritto se non e' ostacolo e se sono al centro
            else:
                # print("differenza dal centro", abs(self._y - (FIELD_SIZE_Y / 2)))
                myflag-=1
                self.position_commander.forward(DPOS)
                time.sleep(DTIME)
        print("FORWARD - finished")

    def lateral(self, setpoint, dir_grid):
        '''
        Inputs: setpoint is la direzione del vettore da seguire
                dir_grid is il verso del vettore da seguire: 1 se right, -1 se left
        '''
        while 0.2 < self._y < FIELD_SIZE_Y-0.2:
                
                # parte che resetta la modalita' di NON ostacolo
                sens = self._back if self._back < self._front else self._front
                if sens > LATERAL_SENSOR_THRESHOLD:
                    lat_obstacle = False

                # Parte che va al centro se non sono al centro
                if abs(self._x - setpoint) > 0.1 and not lat_obstacle and myflag<=0:
                    print("Moving towards the setpoint")
                    dir = np.sign(self._x - setpoint) # 1 se sono avanti al setpoint --> voglio andare indietro
                    while np.abs(self._x - setpoint) > 0.1:
                        print("differenza dal setpoint", abs(self._x - setpoint))
                        print("x", self._x)
                        sens = self._back if dir > 0 else self._front
                        if sens < LATERAL_SENSOR_THRESHOLD:
                            print("Detected front/back obstacle, just go (lateral)")
                            lat_obstacle = True
                            break
                        self.position_commander.back(dir * DPOS)
                        time.sleep(DTIME)
                    

                sens = self._right if dir_grid == 1 else self._left
                # Parte per obstacle avoidance
                if sens < FRONT_SENSOR_THRESHOLD:
                    print("Obstacle detected in front (lateral)")
                    print("Going back" if dir > 0 else "Going front")
                    dir = np.sign(self._x - setpoint)
                    sens2 = self._back if dir == 1 else self._front
                    while sens < FRONT_SENSOR_THRESHOLD:
                        if sens2 < LATERAL_SENSOR_THRESHOLD:
                            print("Detected lateral obstacle, inverting direction")
                            dir = -dir
                            print("Going back" if dir > 0 else "Going front")
                            
                        self.position_commander.back(dir * DPOS)
                        time.sleep(DTIME)
                        # print("differenza dal centro", abs(self._y - (FIELD_SIZE_Y / 2)))
                    myflag=70
                    
                    print("Obstacle in front not detected anymore, moving a little bit more")
                    for _ in range(2):
                        self.position_commander.right(dir * DPOS)
                        time.sleep(DTIME)
                # Parte che va a dritto se non e' ostacolo e se sono al centro
                else:
                    # print("differenza dal centro", abs(self._y - (FIELD_SIZE_Y / 2)))
                    myflag-=1
                    self.position_commander.right(dir_grid * DPOS)
                    time.sleep(DTIME)
        print("FORWARD - finished")


    # GRID SEARCH----------------------------------------------------------------------
    def grid_search(self): #to be rotated of 90 degrees
        print("GRID_SEARCH - start")
        lat_obstacle = False
        myflag = 0
        detected = False

        dir = 1
        print("GS: inizio con setpoint: ", self._x)
        #Finchè non detecto il landing pad [ TODO o finche non sono alla fine della landing region --> use backward]
        s = LANDING_REGION_X+0.1
        step = 0.25
        setpoints = []
        for i in range(6):
            setpoints.append(s + i*step)
        print(setpoints)

        i=0
        while not detected:  

            self.lateral(setpoints[i], dir)

            print("GS: sono arrivato al bordo")
            
            # Save position before starting forward movements
            i += 1
            while self._x < setpoints[i]: 
                self.position_commander.forward(DPOS)
                time.sleep(DTIME)
            # Invert dir
            dir = -dir
            print("GS: ricomincio su nuovo setpoint: ", setpoints[i])
    #----------------------------------------------------------------------------------

    def cross(self):
        center = [0,0]
        point_0 = [self._x, self._y]
        self.dir_long = 1 # 1 = forward, -1 = backward 
        while self.landing_pad_detected() == False or abs(self._x - point_0[0]) < 0.30 :
            self.position_commander.forward(self.dir_long * DPOS)
        pointA = self._x
        self.dir_long = - self.dir_long
        while self.landing_pad_detected() == False or abs(self._x - pointA) < 0.30 :
            self.position_commander.forward(self.dir_long * DPOS)
        pointB = self._x
        center[0] = (pointA - pointB ) / 2

        # Go to center point on x coordinate
        self.dir_long = - self.dir_long
        while abs(self._x - center[0]):
            self.command.forward(self.dir_long*DPOS)

        self.dir_lat = 1 # 1 = right, -1 = left
        while self.landing_pad_detected() == False or abs(self._y - point_0[1]) < 0.30 :
            self.position_commander.right(self.dir_lat * DPOS)
        pointC = self._y
        self.dir_lat = -1 # 1 = right, -1 = left
        while self.landing_pad_detected() == False or abs(self._y - pointC) < 0.30 :
            self.position_commander.right(self.dir_lat * DPOS)
        pointD = self._y

        center[1] = pointD

        #Go to center
        self.dir_lat = -self.dir_lat # 1 = right, -1 = left
        while abs(self._x - center[1]):
            self.position_commander.right(self.dir_lat * DPOS)



        #continue lateral/forward until you detect the end of the landing pad (change in zrange)
        while self.landing_pad_detected() == False:
            self.position_commander.right(DPOS)
            time.sleep(DTIME)
        return

    

    def filter_zrange(self):
        self._zrange_filt = self._zrange_filt * Z_ALPHA + self._zrange * (1.0 - Z_ALPHA)
        return self._zrange_filt

    def landing_pad_detected(self):
        detected = (abs(self._zrange - self._zrange_filt) > LANDING_PAD_DETECTION_THRESHOLD)
        if detected:
            print("Landing pad detected!")
        return detected
    
    

    def __init__(self, scf, home_position):
        print("Init drone - start")
        self.home_position = home_position
        self._scf = scf

        # Connection callback
        self._scf.cf.connected.add_callback(self._connected)
        self._scf.cf.disconnected.add_callback(self._disconnected)
        self._scf.cf.connection_failed.add_callback(self._connection_failed)
        self._scf.cf.connection_lost.add_callback(self._connection_lost)

        self._zrange_filt = 0.0
        
        print("Init drone - resetting kalman estimation")
        # reset kalman filter values
        self._scf.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.5)
        self._scf.cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)
        self._init_logging()
        print("Init drone - finished")


    def _connected(self, URI):
        print('We are now connected to {}'.format(URI))
    
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
        lmeas.add_variable('range.up')
        lmeas.add_variable('range.left')
        lmeas.add_variable('range.right')
        lmeas.add_variable('range.zrange')
        lmeas.add_variable('stabilizer.roll')
        lmeas.add_variable('stabilizer.pitch')
        lmeas.add_variable('stabilizer.yaw')

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
        self._up     = self._convert_log_to_distance(data['range.up'])
        self._front  = self._convert_log_to_distance(data['range.front'])
        self._back   = self._convert_log_to_distance(data['range.back'])
        self._left   = self._convert_log_to_distance(data['range.left'])
        self._right  = self._convert_log_to_distance(data['range.right'])
        self._zrange = self._convert_log_to_distance(data['range.zrange'])
        self.filter_zrange()

        #print("up: ", self._up, " front: ", self._front, " back: ", self._back, " left: ", self._left, " right: ", self._right, " zrange: ", self._zrange)
        

    def pos_data(self, timestamp, data, logconf):
        self._x = data['stateEstimate.x'] + self.home_position[0]
        self._y = data['stateEstimate.y'] + self.home_position[1]
        self._z = data['stateEstimate.z']
        #print("x: ", self._x, " y: ", self._y, " z: ", self._z)

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