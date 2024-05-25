from cflib.utils import uri_helper
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie
import cflib.crtp

from drone import Drone
import numpy as np

if __name__ == '__main__':    

    uri_ = 'radio://0/37/2M/E7E7E70101'

    uri = uri_helper.uri_from_env(default=uri_)
    cflib.crtp.init_drivers()

    # Define the home position (X,Y) of take off pad
    home = np.array([0.4,1.70])


    # Run control
    cf = Crazyflie(rw_cache=".cache")
    with SyncCrazyflie(uri,cf) as scf:
        drone = Drone(scf,home_position=home)
        
        drone.run()