# -*- coding: utf-8 -*-
"""
Created on Tue Oct 24 16:17:28 2017

@author: steve
"""

from drone import Drone
from enum import Enum
from connection import message_types as mt
from connection import mavlink_connection as mc
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
<<<<<<< HEAD
        
        @self.msg_callback(mt.MSG_LOCAL_POSITION)
        def local_position_callback(msg_name,msg):
            if self.flight_state == States.MANUAL:
                pass
            elif self.flight_state == States.ARMING:
                pass
            elif self.flight_state == States.TAKEOFF:
                if -1.0*msg.down > 0.95*self.target_position[2]:
                    self.all_waypoints = self.calculate_box()
                    self.waypoint_transition()
            elif self.flight_state == States.WAYPOINT:
                if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                    if len(self.all_waypoints) > 0:
                        self.waypoint_transition()
                    else:
                        self.landing_transition()
            elif self.flight_state == States.LANDING:
                pass
            elif self.flight_state == States.DISARMING:
                pass
        
        @self.msg_callback(mt.MSG_VELOCITY)
        def velocity_callback(msg_name,msg):
            if self.flight_state == States.MANUAL:
                pass
            elif self.flight_state == States.ARMING:
                pass
            elif self.flight_state == States.TAKEOFF:
                pass
            elif self.flight_state == States.WAYPOINT:
                pass
            elif self.flight_state == States.LANDING:
                pass
                #if self.global_position[2] - self.global_home[2] < 0.1:
                #    if abs(msg.down)<0.01:
                #        self.disarming_transition()
            elif self.flight_state == States.DISARMING:
                pass
            
            
        @self.msg_callback(mt.MSG_STATE)
        def state_callback(msg_name,msg):
            if self.in_mission:
                if self.flight_state == States.MANUAL:
                    if msg.guided:
                        self.flight_state = States.ARMING
                    pass
                elif self.flight_state == States.ARMING:
                    if msg.armed:
                        print("have control and armed")
                        self.takeoff_transition()
                        
                elif self.flight_state == States.TAKEOFF:
                    pass
                elif self.flight_state == States.WAYPOINT:
                    pass
                elif self.flight_state == States.LANDING:
                    if not msg.armed and not msg.guided:
                        self.stop()
                        self.in_mission = False
                elif self.flight_state == States.DISARMING:
                    #if ~msg.armed:
                    #    self.manual_transition()
                    pass
    
    def calculate_box(self):
        print("Setting Home")
        #local_waypoints = [[10.0, 0.0, 3.0],[10.0,10.0,3.0],[0.0,10.0,3.0],[0.0,0.0,3.0]]
        local_waypoints = [[10.0, 0.0, -5.0], [10.0, 10.0, -5.0], [0.0, 10.0, -5.0], [0.0, 0.0, -5.0]]
        #for i in range(0,4):
        #    global_waypoints.extend([frame_utils.local_to_global(local_waypoints[i, :], global_home)])
        return local_waypoints
=======

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
>>>>>>> origin/master

    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
<<<<<<< HEAD
        self.take_control()
        self.arm()
        #self.set_home_position(self.global_position[0],self.global_position[1],self.global_position[2]) #set the current location to be the home position

        self.flight_state = States.ARMING
        #self.whatever = States.ARMING
        
    def takeoff_transition(self):  
        print("takeoff transition")     
        #self.global_home = np.copy(self.global_position)  # can't write to this variable!
        target_altitude = -5.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF        
=======

    def takeoff_transition(self):
        """TODO: Fill out this method
>>>>>>> origin/master
        
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
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
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
        while self.in_mission:
            pass


if __name__ == "__main__":
    # create the necessary connection first
    
    # using a HITL simulator
    connection = mc.MavlinkConnection("udp:127.0.0.1:14540", threaded=True, PX4=True)
    
    # using a companion computer plugged into Pixhawk UART port
    #connection = mc.MavlinkConnection("/dev/ttyUSB0,921600", threaded=True, PX4=True)
    
    # using a wireless connection using a 3DR radio (or equivalent)
    #connection = mc.MavlinkConnection("/dev/ttyUSB0,57600", threaded=True, PX4=True)
    
    # same code as before
    drone = BackyardFlyer(connection=connection)
    time.sleep(2)
    drone.start()