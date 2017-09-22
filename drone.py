# -*- coding: utf-8 -*-

import numpy as np
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
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.heading = None
        self.mode = None
        self.connected = False
        
        #TODO: Change the log name
        logname = "navLog.txt"
        self.log = logger.Logger(os.path.join("Logs",logname))
		
		
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

    # Command the vehicle to the target position, assign the target to self.target_position and return True when it has arrived
    def goto(self, target):
        #TODO: fill out this method
        return True

    # Lands the vehicle in the current location and returns true when the vehicle is on the ground
    def land(self):
        #TODO: fill out this method
        return True
        
    #Disarms the vehicle, returns control to manual, and returns true when the motors report armed
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
            #print('Heartbeat Message')
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
            
            for i in range(3):
                data.append(self.target_position[i])
            
            self.log.log_data(data)
            
        else:
            print(name)



