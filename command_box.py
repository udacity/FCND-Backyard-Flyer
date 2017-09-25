# -*- coding: utf-8 -*-


import numpy as np

import time
import frame_utils
from drone import Drone








# This is the 
if __name__ == "__main__":
    drone = Drone()
    drone.connect("tcp:127.0.0.1:5760")
    
    time.sleep(2)
    drone.run_mission()
        
    # terminate the connection
    drone.disconnect()