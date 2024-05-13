from cflib.crazyflie.log import LogConfig
# from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.positioning.motion_commander import MotionCommander
import time
import numpy as np

DPOS = 0.005
DTIME = 0
LANDING_REGION_X = 2.0 # TODO
LATERAL_SENSOR_THRESHOLD = 0.25
FRONT_SENSOR_THRESHOLD = 0.4

FIELD_SIZE_Y = 3.0

Z_ALPHA = 0.3 # Z EMA PARAMETER

LANDING_PAD_DETECTION_THRESHOLD = 0.03

class Drone:
    def run(self):
        print("Drone run - start")
        with MotionCommander(self._scf, default_height=0.3) as self.position_commander:
            # Take off and wait a second
            time.sleep(2.0)

            # Forward
            self.forward()

            # Landing pad search

            # Land

            # Take off

            # Return to home position

        print("Drone run - finished")

    def forward(self):
        print("FORWARD - start")
        while self._x < LANDING_REGION_X:
            if self._front < FRONT_SENSOR_THRESHOLD:
                print("Obstacle detected in front")

                dir = np.sign(self._y - (FIELD_SIZE_Y / 2))
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
                #self.position_commander.right(dir * DPOS)
                time.sleep(DTIME)

            else:
                self.position_commander.forward(DPOS)
                time.sleep(DTIME)
        print("FORWARD - finished")

    def filter_zrange(self):
        self._zrange_filt = self._zrange_filt * Z_ALPHA + self._zrange * (1.0 - Z_ALPHA)
        return self._zrange_filt

    def landing_pad_detected(self):
        return abs(self._zrange - self._zrange_filt) > LANDING_PAD_DETECTION_THRESHOLD
    
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
        self._x = data['stateEstimate.x'] - self.home_position[0]
        self._y = data['stateEstimate.y'] - self.home_position[1]
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