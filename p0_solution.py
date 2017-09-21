# -*- coding: utf-8 -*-


import numpy as np

import utm
import time
import logger
import connection
import os
from pymavlink import mavutil



class Drone:
    
    #This method will be provided to the students
    def __init__(self):
        self.global_position = np.array([None, None, None]) #Longitude, Latitude, Altitude
        self.global_home = np.array([None, None, None])
        self.motors_armed = False
        self.global_velocity = np.array([None, None, None])
        self.heading = None
        self.mode = None
        self.connected = False
        
        
        self.log = logger.Logger(os.path.join("Logs","navLog.txt"))
		
		
    #Provided
    def connect(self, device):
        self.connection = connection.Connection(device, self.decode_mav_msg)
        while self.connected != True:

            time.sleep(1)
    
    #Provided
    def disconnect(self):
        self.connection.disconnect()
        self.log.close()

    
    #Sets the mode to guided, arms the vehicle (with checks) and save the home position as the position it is armed
    def arm_vehicle(self):
        #TODO: fill out this method
        return True

    #Command the vehicle to a specific height and return True when it gets to the specified altitude
    def takeoff(self):
        # TODO: fill out this method
        return True

    # Command the vehicle to the target position and return True when it has arrived
    def goto(self, target):
        #TODO: fill out this method
        return True

    # Lands the vehicle in the current location and returns true when the vehicle is on the ground
    def land(self):
        #TODO: fill out this method
        return True
        
    #Disarms the vehicle, and returns true when the motors report armed
    def disarm_vehicle(self):
        #TODO: fill out this method   
        return True

    #This method is provided
    def decode_mav_msg(self, name, msg):
        # NOTE: this effectively becomes a callback of the main connection thread
        #This will be implemented for the students and sort the mavlink message to different callbacks for different types
        #It may actually just populate the vehicle class data directly
        
        if name is 'STATUSTEXT':
            name #Do nothing

        elif name is 'HEARTBEAT':
            # print('Heartbeat Message')
            self.connected = True
            self.motors_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            # Correctly parse the state information
        elif name is 'GLOBAL_POSITION_INT':
            
            self.global_position[0] = float(msg.lon) / (10 ** 7)
            data = [self.global_position[0]]
            self.global_position[1] = float(msg.lat) / (10 ** 7)
            data.append(self.global_position[1])
            self.global_position[2] = float(msg.relative_alt) / 1000
            data.append(self.global_position[2])
            self.global_velocity[0] = float(msg.vx) / 100
            data.append(self.global_velocity[0])
            self.global_velocity[1] = float(msg.vy) / 100
            data.append(self.global_velocity[1])
            self.global_velocity[2] = -float(msg.vz) / 100
            data.append(self.global_velocity[2])
            self.heading = float(msg.hdg) / 100
            data.append(self.heading)
            
            self.log.log_data(data)
            
        else:
            print(name)


#Take control of the drone, arm motors, takeoff to a height of 3m, fly a 10m box, land, and disarm
def takeoff_and_fly_box(drone):
#TODO: filled out this function



#Helper functions provided to the students
#Convert a global position (lon,lat,up) to a local position (north,east,down) relative to the home position
def global_to_local(global_position, global_home):
    (east_home, north_home, _, _) = utm.from_latlon(global_home[1],
                                                    global_home[0])
    (east, north, _, _) = utm.from_latlon(global_position[1],
                                          global_position[0])
                                          
    local_position = [
        north - north_home, east - east_home,
        -global_position[2]
    ]
    return local_position


#Convert a local position (north,east,down) relative to the home position to a global position (lon,lat,up)
def local_to_global(local_position, global_home):
    (east_home, north_home, zone_number, zone_letter) = utm.from_latlon(
        global_home[1], global_home[0])
    (lat, lon) = utm.to_latlon(east_home + local_position[1],
                               north_home + local_position[0], zone_number,
                               zone_letter)
                               
    lla = [lon, lat, -local_position[2]]
    return lla


#Solve for the distance between two local positions
def distance_between(position1, position2):
    sum_square = 0.0
    for i in range(len(position1)):
        sum_square = sum_square + np.power(position1[i] - position2[i],2)
    return np.sqrt(sum_square)



# This is the 
if __name__ == "__main__":
    drone = Drone()
    drone.connect("tcp:127.0.0.1:5760")
    
    time.sleep(2)
    takeoff_and_fly_box(drone)
        
    # terminate the connection
    drone.disconnect()