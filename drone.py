# -*- coding: utf-8 -*-

import numpy as np
import logger
import connection
import os
from pymavlink import mavutil
from enum import Enum
import frame_utils
import sys

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
        self.check_state = {}
        self.transitioning = False
        
        #TODO: Change the log name
        logname = "navLog.txt"
        self.log = logger.Logger(os.path.join("Logs",logname))
		
		
    #Provided
    def connect(self, device):
        self.connection = connection.Connection(device, self.decode_mav_msg)
        self.connection.wait_for_message()
        
    
    #Provided
    def disconnect(self):
        self.connection.disconnect()
        
        
    #Provided
    def manual(self):
        self.in_mission
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE, 0, 0,
                             0, 0, 0, 0, 0)
        self.state = States.MANUAL
        return
    
    #Initiate the ARMING state, put the vehicle in guided mode and arm
    def arm(self):
        #TODO: fill out this method
        return
    
    #Initiate the TAKEOFF state, command the vehicle the target altitude (m)
    def takeoff(self,altitude=3.0):
        #TODO: fill out this method
        return
    
    #Initiate the WAYPOINT state, command the vehicle to the position specified as Lat, Long, Alt
    def waypoint(self,target):
        #TODO: fill out this method
        return
    
    #Initiate the LANDING state, command the vehicle to specified altitude (m)
    def land(self, altitude=0.0):
        #TODO: fill out this method
        return
    
    #Initiate the DIARMING state, command the vehicle to disarm
    def disarm(self):
        #TODO; fill out this method
        return
     
    #Provided
    def manual_callback(self):
        
        return True
    
    
    #Save the current position as the home position and transition to the next state when the vehicle is armed and in guided mode
    def arming_callback(self):
        #TODO: fill out this callback
        return
        
    
    #Transition to the next state when the target altitude is reach
    def takeoff_callback(self):
        # TODO: fill out this callback
        return
    
        
    # Transition to the next state when the target waypoint is reached (within 1m)
    def waypoint_callback(self):
        #TODO: fill out this callback
        return                
       

    # Transition to the next state when the drone is on the ground
    def landing_callback(self):
        #TODO: fill out this callback
        return
    
    # Transition to the next state when the drone is disarmed
    def disarming_callback(self):
        #TODO: fill in this callback
        return

    #This method is provided
    def decode_mav_msg(self, name, msg):
        print('Data Received')
        # NOTE: this effectively becomes a callback of the main connection thread
        #This will be implemented for the students and sort the mavlink message to different callbacks for different types
        #It may actually just populate the vehicle class data directly
        self.connected = True        
        if name is 'STATUSTEXT':
            name #Do nothing

        elif name is 'HEARTBEAT':
            #print('Heartbeat Message')
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
    
    #Provided, sets the list of callbacks      
    def set_callbacks(self):
        self.check_state[States.MANUAL] = self.manual_callback
        self.check_state[States.ARMING] = self.arming_callback
        self.check_state[States.TAKEOFF] = self.takeoff_callback
        self.check_state[States.WAYPOINT] = self.waypoint_callback
        self.check_state[States.LANDING] = self.landing_callback
        self.check_state[States.DISARMING] = self.disarming_callback

    #Provided
    def init_mission(self):
        self.connect("tcp:127.0.0.1:5760")
        self.in_mission = True
        self.set_callbacks()

    #Provided, runs the mission
    def run_mission(self):
              
        self.init_mission() # run the mission intialization
        self.arm() # Set the first state to arm

        while self.in_mission & self.connected:
            timeout = self.connection.wait_for_message() #Block thread until a new Mavlink message arrives
            if timeout:
                print('Connection Timeout')
                self.connected = False
            self.check_state[self.state]() #Check for a state transition
        
        if self.connected:
            self.connection.disconnect()
        self.log.close()
        
    #Provided
    def init_manual(self):
        self.connect("tcp:127.0.0.1:5760")
    
    #Provided, runs the logger while flying manually
    def fly_manual(self):
        self.init_manual()
        while self.connected:
            timeout = self.connection.wait_for_message()
            if timeout:
                print('Connection Timeout')
                self.connected = False
        
        self.log.close()    

if __name__ == "__main__":
    arguments = sys.argv[1:]
    count = len(arguments)
    
    drone = Drone()
    if count == 0:
        print('Manual Flight')
        drone.fly_manual()
    elif count == 1:
        if arguments[0] == '-r':
            print('Running Mission')
            drone.run_mission()
        else:
            print('Invalid argment')
    else:
        print('Invalid number of arguments')
