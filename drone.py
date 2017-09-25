# -*- coding: utf-8 -*-

import numpy as np
import time
import logger
import connection
import os
from pymavlink import mavutil
from enum import Enum
import threading
import frame_utils

class States(Enum):
    MANUAL=0
    ARMING=1
    TAKEOFF=2
    WAYPOINT=3
    LANDING=4
    DISARMING=5
    
def calculate_box(global_home):
    
    global_waypoints = []#np.zeros((4, 3))
    local_waypoints = np.array([[10.0, 0.0, -3.0],[10.0, 10.0, -3.0],[0.0, 10.0, -3.0],[0.0, 0.0, -3.0]])
    for i in range(0,4):
        global_waypoints.extend([frame_utils.local_to_global(local_waypoints[i, :], global_home)])

    return global_waypoints

class Drone:
    
    #This method will be provided to the students
    def __init__(self):
        self.global_position = np.array([None, None, None]) #Longitude, Latitude, Altitude
        self.global_home = np.array([None, None, None])
        self.motors_armed = False
        self.global_velocity = np.array([None, None, None])
        self.heading = None
        self.guided = False
        self.connected = False
        

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = False
        self.state = States.MANUAL
        self.check_transition = {}
        self.transition = []
        
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


    def manual_callback(self):
        return True
            
    def manual(self,set_guided):
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE, set_guided, 0,
                             0, 0, 0, 0, 0)
        if(set_guided==0):
            self.state = States.MANUAL
    
    #Set mode to guided, arm the motors
    def arm(self):
        self.state = States.ARMING
        self.manual(1)
		
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1,
                                     0, 0, 0, 0, 0, 0)
        
    #Save position and transition to takeoff when armed
    def arming_callback(self):
        #TODO: fill out this method
        
        if(self.motors_armed):
            self.global_home = np.copy(self.global_position)
            print("Home Set: " + np.str(self.global_home))
            self.takeoff()
        
    def takeoff(self,altitude=3.0):
        self.state = States.TAKEOFF
        self.target_position = [x for x in self.global_home]
        self.target_position[2] = self.target_position[2] + altitude

        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0,
                                     0, 0,altitude)

    #Command the vehicle to a specific height and return True when it gets to the specified altitude
    def takeoff_callback(self):
        # TODO: fill out this method
        
        if(self.global_position[2] > 0.95*self.target_position[2]):
            self.all_waypoints = calculate_box(self.global_home)
            self.target_position = self.all_waypoints.pop(0)
            self.goto(self.target_position)



    #Go to the commanded target, specified as Lat, Long, Alt
    def goto(self,target):
        self.state = States.WAYPOINT
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 0, 0, target[1]*10**7, target[0]*10**7, target[2])
        
    # Check the position to the target position
    def waypoint_callback(self):
        #TODO: fill out this method
        
        local_position = frame_utils.global_to_local(self.global_position,self.global_home)
        target_local = frame_utils.global_to_local(self.target_position,self.global_home)
        if(np.linalg.norm(target_local-local_position)<1.0):
            if(len(self.all_waypoints)>0):
                self.target_position = self.all_waypoints.pop(0)
                self.goto(self.target_position)
            else:
                self.land()
                
    def land(self, altitude=0.0):
        self.state = States.LANDING
        self.target_position = np.copy(self.global_home)
        self.target_position[2] = self.target_position[2]-1.0
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0,
                                     0, 0, self.target_position[2])
        

    # Lands the vehicle in the current location and returns true when the vehicle is on the ground
    def landing_callback(self):
        #TODO: fill out this method
        
        if(self.global_position[2]-self.global_home[2]<0.05):
            if(np.abs(self.global_velocity[2])<0.1):
                self.disarm()

        
    def disarm(self):
        self.state = States.DISARMING
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                     0, 0, 0, 0, 0, 0)
        
    #Disarms the vehicle, returns control to manual, and returns true when the motors report armed
    def disarming_callback(self):
        #TODO: fill out this method   
        
        if(~self.motors_armed):
            self.manual(0)
            self.in_mission = False

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
            self.guided = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED) != 0
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
            
    def set_transitions(self):
        self.check_transition[States.MANUAL] = self.manual_callback
        self.check_transition[States.ARMING] = self.arming_callback
        self.check_transition[States.TAKEOFF] = self.takeoff_callback
        self.check_transition[States.WAYPOINT] = self.waypoint_callback
        self.check_transition[States.LANDING] = self.landing_callback
        self.check_transition[States.DISARMING] = self.disarming_callback
    
    def run_mission(self):
        self.in_mission = True
        self.set_transitions()
        
        self.arm()        
        while self.in_mission:
            self.check_transition[self.state]()




