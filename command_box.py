# -*- coding: utf-8 -*-


import numpy as np

import time
import frame_utils
from drone import Drone



#Take control of the drone, arm motors, takeoff to a height of 3m, fly a 10m box, land, and disarm
def takeoff_and_fly_box(drone):
#TODO: filled out this function
    return True




# This is the 
if __name__ == "__main__":
    drone = Drone()
    drone.connect("tcp:127.0.0.1:5760")
    
    time.sleep(2)
    takeoff_and_fly_box(drone)
        
    # terminate the connection
    drone.disconnect()