from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import cflib.crtp
import time
import numpy as np

class Drone:
    def __init__(self, cf, scf, home_position, cruise_height=0.4):
        print("Init drone")
        self.home_position = home_position
        self.cruise_height = cruise_height

        self._cf = cf
        self._scf = scf

        # Connection callback
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)


        self.position_commander = PositionHlCommander(self._scf, default_height=self.cruise_height)
        print("after position commander")

        time.sleep(1)


    def run(self):
        # finish first sequence
        #land_detected = False
        #obstacle_avoid = False
        print("resetting kalman")
        # reset kalman filter values
        self._cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self._cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)
        print("resetting kalman")

        print("Taking off")
        self.position_commander.take_off(self.cruise_height)   
        time.sleep(1.0)
        print("Taking off done")

        time.sleep(1.0)

        self.position_commander.go_to(0.5,0.5, self.cruise_height)
        time.sleep(1.0)
        self.position_commander.land()
        print("Finished")


    def _connected(self, URI):
        print('We are now connected to {}'.format(URI))

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
        lmeas.add_variable('stabilizer.roll')
        lmeas.add_variable('stabilizer.pitch')
        lmeas.add_variable('stabilizer.yaw')

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
        print("before constructor")
        drone = Drone(cf, scf,home_position=home, cruise_height=CRUISE_HEIGHT)
        print("after constructor")
        drone.run()