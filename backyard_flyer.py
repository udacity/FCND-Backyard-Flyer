# -*- coding: utf-8 -*-
"""
Created on Tue Oct 24 16:17:28 2017

@author: steve
"""

from drone import Drone
from enum import Enum
from connection import message_types as mt
import numpy as np
import frame_utils

class States(Enum):
    MANUAL=0
    ARMING=1
    TAKEOFF=2
    WAYPOINT=3
    LANDING=4
    DISARMING=5

class BackyardFlyer(Drone):
    
    def __init__(self):
        super().__init__()
        self.target_position = np.array([0.0,0.0,0.0])
        #self.global_home = np.array([0.0,0.0,0.0])  # can't set this here, no setter for this property
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL
        self.whatever = States.MANUAL
    
    def callbacks(self):
        """ Define your callbacks within here"""
        super().callbacks()
        
        @self.msg_callback(mt.MSG_GLOBAL_POSITION)
        def global_position_callback(msg_name,msg):
            if self.flight_state == States.MANUAL:
                pass
            elif self.flight_state == States.ARMING:
                pass
            elif self.flight_state == States.TAKEOFF:
                if msg.altitude > 0.95*self.target_position[2]:
                    self.all_waypoints = self.calculate_box()
                    self.waypoint_transition()
            elif self.flight_state == States.WAYPOINT:
                if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                    if len(self.all_waypoints) > 0:
                        self.waypoint_transition()
                    else:
                        self.landing_transition()
            elif self.flight_state == States.LANDING:
                if msg.altitude - self.global_home[2] < 0.05:
                    self.disarming_transition()
            elif self.flight_state == States.DISARMING:
                pass
        
        @self.msg_callback(mt.MSG_STATE)
        def state_callback(msg_name,msg):
            if self.flight_state == States.MANUAL:
                self.arming_transition()
                pass
            elif self.flight_state == States.ARMING:
                if msg.armed:
                    self.takeoff_transition()
                    
            elif self.flight_state == States.TAKEOFF:
                pass
            elif self.flight_state == States.WAYPOINT:
                pass
            elif self.flight_state == States.LANDING:
                pass
            elif self.flight_state == States.DISARMING:
                if ~msg.armed:
                    self.manual_transition()
    
    def calculate_box(self):
        
        global_waypoints = []#np.zeros((4, 3))
        local_waypoints = [[0, 0, 5]]
        #local_waypoints = [[10.0, 0.0, -3.0],[10.0, 10.0, -3.0],[0.0, 10.0, -3.0],[0.0, 0.0, -3.0]]
        #for i in range(0,4):
        #    global_waypoints.extend([frame_utils.local_to_global(local_waypoints[i, :], global_home)])
        return local_waypoints

    def arming_transition(self):
        print("arming transition")
        self.arm()
        self.flight_state = States.ARMING
        #self.whatever = States.ARMING
        
    def takeoff_transition(self):  
        print("takeoff transition")     
        #self.global_home = np.copy(self.global_position)  # can't write to this variable!
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF        
        
    def waypoint_transition(self):
        print("waypoint transition")
        self.target_position = self.all_waypoints.pop(0)
        self.cmd_position(self.target_position[0],self.target_position[1],self.target_position[2],0.0)
        self.flight_state = States.WAYPOINT
        
    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING
        
    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING
        
    def manual_transition(self):
        print("manual transition")
        self.release_control()
        self.in_mission = False
        self.flight_state = States.MANUAL   
        
    def start(self):
        
        self.start_log("Logs","NavLog.txt")
        #self.connect()
        
        print("starting connection")
        self.connection.start()
        while self.in_mission:
            pass

        self.stop_log()

if __name__ == "__main__":
    drone = BackyardFlyer()

    drone.start()