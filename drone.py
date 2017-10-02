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
        self.connection = connection.Connection(device)
        timeout = self.connection.wait_for_message()
        
        if ~timeout:
            self.connected = True
        
    
    #Provided
    def disconnect(self):
        self.connection.disconnect()
        
        
    #Provided
    def manual(self):
        self.in_mission = False
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE, 0, 0,
                             0, 0, 0, 0, 0)
        self.state = States.MANUAL
        print('Commanded Manual')
        return
    
    #Initiate the ARMING state, put the vehicle in guided mode and arm
    def arm(self):
        self.state = States.ARMING
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE, 1, 0,
                                     0, 0, 0, 0, 0)
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1,
                                     0, 0, 0, 0, 0, 0)          
        return
    
    #Initiate the TAKEOFF state, command the vehicle the target altitude (m)
    def takeoff(self,altitude=3.0):
        self.state = States.TAKEOFF
        self.target_position = np.copy(self.global_position)
        self.target_position[2] = altitude
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0,
                                     0, 0, altitude)
        return
    
    #Initiate the WAYPOINT state, command the vehicle to the position specified as Lat, Long, Alt
    def waypoint(self,target):
        self.state = States.WAYPOINT
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 0, 0, target[1]*10**7, target[0]*10**7, target[2])
        return
    
    #Initiate the LANDING state, command the vehicle to specified altitude (m)
    def land(self, altitude=0.0):
        self.state = States.LANDING
        self.target_position = np.copy(self.global_home)
        self.target_position[2] = altitude
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0,
                                     0, 0, self.target_position[2])

        return
    
    #Initiate the DIARMING state, command the vehicle to disarm
    def disarm(self):
        self.state = States.DISARMING
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                     0, 0, 0, 0, 0, 0)
        return
     
    #Provided
    def manual_transition(self):
        #Do nothing
        return
    
    
    #Save the current position as the home position and transition to the next state when the vehicle is armed and in guided mode
    def arming_transition(self):
        if self.motors_armed & self.guided:
            self.global_home = np.copy(self.global_position)
            self.takeoff()
        return
        
    
    #Transition to the next state when the target altitude is reach
    def takeoff_transition(self):
        if self.global_position[2] >= 0.95*self.target_position[2]:
            self.all_waypoints = calculate_box(self.global_home)
            self.target_position = self.all_waypoints.pop(0)
            self.waypoint(self.target_position)
        return
    
        
    # Transition to the next state when the target waypoint is reached (within 1m)
    def waypoint_transition(self):
        local_target = frame_utils.global_to_local(self.target_position,self.global_home)
        local_position = frame_utils.global_to_local(self.global_position,self.global_home)
        if np.linalg.norm(local_target[0:2]-local_position[0:2])<1.0:
            if len(self.all_waypoints)>0:
                self.target_position = self.all_waypoints.pop(0)
                self.waypoint(self.target_position)
            else:
                self.land(self.global_home[2]-1.0)
                
        return                
       

    # Transition to the next state when the drone is on the ground
    def landing_transition(self):
        if self.global_position[2] - self.global_home[2] < 0.05:
            if self.global_velocity[2] < 0.05:
                self.disarm()
        return
    
    # Transition to the next state when the drone is disarmed
    def disarming_transition(self):
        if ~self.motors_armed:
            self.manual()
        return


    def heartbeat_callback(self,msg):
        self.connected = True
        self.motors_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        self.guided = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED) != 0
        
        if self.state in self.check_state.keys():
            self.check_state[self.state]()
        
    def global_position_callback(self, msg):
        self.connected = True
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
        
        if self.state in self.check_state.keys():
            self.check_state[self.state]()

    
    #Provided, sets the list of callbacks      
    def set_transitions(self):
        self.check_state[States.MANUAL] = self.manual_transition
        self.check_state[States.ARMING] = self.arming_transition
        self.check_state[States.TAKEOFF] = self.takeoff_transition
        self.check_state[States.WAYPOINT] = self.waypoint_transition
        self.check_state[States.LANDING] = self.landing_transition
        self.check_state[States.DISARMING] = self.disarming_transition

    #Provided
    def init_mission(self):
        self.connect("tcp:127.0.0.1:5760")
        self.in_mission = True
        self.set_transitions()
        self.connection.set_callback('HEARTBEAT',self.heartbeat_callback)
        self.connection.set_callback('GLOBAL_POSITION_INT',self.global_position_callback)

    #Provided, runs the mission
    def run_mission(self):
              
        self.init_mission() # run the mission intialization
        self.arm() # Set the first state to arm

        while self.in_mission & self.connected:
            timeout = self.connection.wait_for_message() #Block thread until a new Mavlink message arrives
            if timeout:
                print('Connection Timeout')
                self.connected = False
        
        if self.connected:
            self.connection.disconnect()
        self.log.close()
        
    #Provided
    def init_manual(self):
        self.connect("tcp:127.0.0.1:5760")
        self.connection.set_callback('GLOBAL_POSITION_INT',self.global_position_callback)
    
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
