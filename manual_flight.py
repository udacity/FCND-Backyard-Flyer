# -*- coding: utf-8 -*-

import time
from drone import Drone


# This is the 
if __name__ == "__main__":
    drone = Drone()
    drone.connect("tcp:127.0.0.1:5760")
    
    print('Drone Connected')
    #Continue until the connected
    while(drone.connected):
        drone.connected=False
        time.sleep(1)
        if drone.connected:
            print('Heartbeat')
    
    # terminate the connection
    drone.disconnect()