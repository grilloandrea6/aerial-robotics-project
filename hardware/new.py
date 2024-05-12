from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import cflib.crtp
import time
import numpy as np

class Drone:
    def __init__(self, cf, scf, mc, home_position, cruise_height=0.4):
        print("Init drone")
        self.home_position = home_position
        self.cruise_height = cruise_height

        self._cf = cf
        self._scf = scf

        self._up = None
        self._front = None
        self._back = None
        self._left = None
        self._right = None
        self._down = None

        # Connection callback
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self.init_logging()


        print("resetting kalman")
        # reset kalman filter values
        self._cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self._cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)
        
        self.motion_commander = mc
        print("after position commander")

        time.sleep(1)


    def run(self):
        # finish first sequence
        #land_detected = False
        #obstacle_avoid = False
        time.sleep(0.3)
        print("Taking off")
        self.motion_commander.take_off(height=0.5,velocity=2)
        time.sleep(1.0)
        print("taken off!! - start moving")
        self.motion_commander.start_linear_motion(-0.5,0.5,0.0)
        time.sleep(1.0)
        self.motion_commander.start_linear_motion(0.0,0.0,0.0)
        print("stopped")
        time.sleep(1.0)
        self.motion_commander.down(0.5)
        time.sleep(2.5)
        self.motion_commander.up(0.5)
        time.sleep(2)
        self.motion_commander.land()
        print("Finished")
    

    def _convert_log_to_distance(self, data):
        if data >= 8000:
            return None
        else:
            return data / 1000.0

    def meas_data(self, timestamp, data, logconf):
        self._up     = self._convert_log_to_distance(data['range.up'])
        self._front  = self._convert_log_to_distance(data['range.front'])
        self._back   = self._convert_log_to_distance(data['range.back'])
        self._left   = self._convert_log_to_distance(data['range.left'])
        self._right  = self._convert_log_to_distance(data['range.right'])
        self._zrange = self._convert_log_to_distance(data['range.zrange'])
        print(f"up: {self._up}, front: {self._front}, back: {self._back}, left: {self._left}, right: {self._right}, zrange: {self._zrange}")

    def pos_data(self, timestamp, data, logconf):
        self._x = self._convert_log_to_distance(data['stateEstimate.x'])
        self._y = self._convert_log_to_distance(data['stateEstimate.y'])
        self._z = self._convert_log_to_distance(data['stateEstimate.z'])
        

    def init_logging(self):

        # The definition of the logconfig can be made before connecting
        lpos = LogConfig(name='Position', period_in_ms=10)
        lpos.add_variable('stateEstimate.x')
        lpos.add_variable('stateEstimate.y')
        lpos.add_variable('stateEstimate.z')

        try:
            self._cf.log.add_config(lpos)
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
        # lmeas.add_variable('stabilizer.roll')
        # lmeas.add_variable('stabilizer.pitch')
        # lmeas.add_variable('stabilizer.yaw')

        try:
            self._cf.log.add_config(lmeas)
            lmeas.data_received_cb.add_callback(self.meas_data)
            lmeas.error_cb.add_callback(self._log_error)
            lmeas.start()

        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Measurement log config, bad configuration.')
        return 


    def _connected(self, URI):
        print('We are now connected to {}'.format(URI))

    def _connection_failed(self, link_uri, msg):
        """
        Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)
        """
        print('Callback - Connection to %s failed: %s' % (link_uri, msg))
        return 

    def _connection_lost(self, link_uri, msg):
        """
        Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)
        """
        print('Callback - Connection to %s lost: %s' % (link_uri, msg))
        return 

    def _disconnected(self, URI):
        print('Callback - Disconnected')

    def _log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Callback - Error when logging %s: %s' % (logconf.name, msg))
        return
    


      
    

        


if __name__ == '__main__':    
    uri = uri_helper.uri_from_env(default='radio://0/37/2M/E7E7E70101')
    cflib.crtp.init_drivers()

    # Define the home position (X,Y) of take off pad
    home = np.array([1.0,2.0])

    # Define the cruise height
    CRUISE_HEIGHT = 0.4

    # Run control

    cf = Crazyflie(rw_cache=".cache")
    with SyncCrazyflie(uri,cf) as scf:
        # with MotionCommander(scf, default_height=0.2) as mc:
        #     print("before constructor")
        #     drone = Drone(cf, scf, mc, home_position=home, cruise_height=CRUISE_HEIGHT)
        #     print("after constructor")
        #     drone.run()

        mc = MotionCommander(scf, default_height=0.2)

        print("before constructor")
        drone = Drone(cf, scf, mc, home_position=home, cruise_height=CRUISE_HEIGHT)
        print("after constructor")
        drone.run()
