# -*- coding: utf-8 -*-
"""
Created on Tue Oct 24 16:17:28 2017

@author: steve
"""

from drone import Drone
from enum import Enum
from connection import message_types as mt
import numpy as np

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
        self.in_mission = False
        self.check_state = {}

        # initial start
        self.state = States.MANUAL
    
    def callbacks(self):
        """ Define your callbacks within here"""
        super().callbacks()
        
        @self.msg_callback(mt.MSG_GLOBAL_POSITION)
        def global_position_callback(msg_name,msg):
            if self.state == States.MANUAL:
                pass
            elif self.state == States.ARMING:
                pass
            elif self.state == States.TAKEOFF:
                if msg.altitude > 0.95*self.target_position[2]:
                    self.all_waypoints = self.calculate_box(self.global_home)
                    self.waypoint_transition()
            elif self.state == States.WAYPOINT:
                if np.linalg.norm(local_target[0:2]-local_position[0:2])<1.0:
                    if len(self.all_waypoints)>0:
                        self.waypoint_transition()
                    else:
                        self.landing_transition()
            elif self.state == States.LANDING:
                if msg.global_position[2] - self.global_home[2] < 0.05:
                    self.disarming_transition()
            elif self.state == States.DISARMING:
                pass
        
        @self.msg_callback(mt.MSG_STATE)
        def state_callback(msg_name,msg):
            if self.state == States.MANUAL:
                self.arming_transition()
                pass
            elif self.state == States.ARMING:
                if msg.armed:
                    self.takeoff_transition()
                    
            elif self.state == States.TAKEOFF:
                pass
            elif self.state == States.WAYPOINT:
                pass
            elif self.state == States.LANDING:
                pass
            elif self.state == States.DISARMING:
                if ~msg.armed:
                    self.manual_transition()
        
    def arming_transition(self):
        self.arm()
        self.state = States.ARMING
        
    def takeoff_transition(self):       
        self.global_home = np.copy(self.global_position)
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.state = States.TAKEOFF        
        
    def transition_waypoint(self):
        self.target_position = self.all_waypoints.pop(0)
        self.cmd_position(self.target_position[0],self.target_position[1],self.target_position[2],0.0)
        self.state = States.WAYPOINT
        
    def landing_transition(self):
        self.land()
        self.state = States.LANDING
        
    def disarming_transition(self):
        self.disarm()
        self.state = States.DISARMING
        
    def manual_transition(self):
        self.release_control()
        self.in_mission = False
        self.state = States.MANUAL   
        
    def start(self):
        
        self.start_log("Logs","NavLog.txt")
        self.connect()
        
        while self.connected & self.in_mission:
            pass

if __name__ == "__main__":
    drone = BackyardFlyer()

    drone.start()