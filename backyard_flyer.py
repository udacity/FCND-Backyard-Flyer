# -*- coding: utf-8 -*-
"""
Created on Tue Oct 24 16:17:28 2017

@author: steve
"""

from drone import Drone
from enum import Enum
from connection import message_types as mt
import numpy as np
import time


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, protocol='tcp', ip_addr='127.0.0.1', port=5760, threaded=True):
        super().__init__(protocol=protocol, ip_addr=ip_addr, port=port, threaded=threaded)
        # The position the drone is currently flying to.
        # This should be used to liftoff, land and transition to waypoints.
        self.target_position = np.array([0.0, 0.0, 0.0])
        # self.global_home = np.array([0.0,0.0,0.0])  # can't set this here, no setter for this property
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL
        self.whatever = States.MANUAL

    def callbacks(self):
        """ Define your callbacks within here"""
        super().callbacks()

        @self.msg_callback(mt.MSG_STATE)
        def state_callback(msg_name, msg):
            """TODO: FIll out this callback (if required)
            
            For example:
                if self.state==States.MANUAL:
                    self.arming_transition()
            """
            pass

        @self.msg_callback(mt.MSG_GLOBAL_POSITION)
        def global_position_callback(msg_name, msg):
            """TODO: Fill out this callback (if required)"""
            pass

        @self.msg_callback(mt.MSG_LOCAL_POSITION)
        def local_position_callback(msg_name, msg):
            """TODO: Fill out this callback (if required)"""
            pass

        @self.msg_callback(mt.MSG_VELOCITY)
        def velocity_callback(msg_name, msg):
            """TODO: FIll out this callback (if required)"""
            pass

    def calculate_box(self):
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box
        """
        pass

    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the vehicle
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")

    def takeoff_transition(self):
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the vehicle to land
        2. Transition to the LANDING state
        """
        print("landing transition")

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the vehicle to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the vehicle
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Start the drone
        """
        super().start()

        #Only required if they do threaded
        #while self.in_mission:
        #    pass


if __name__ == "__main__":
    drone = BackyardFlyer(threaded=False)
    time.sleep(2)
    drone.start()