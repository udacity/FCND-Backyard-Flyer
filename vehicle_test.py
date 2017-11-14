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
    MANUAL=0
    ARMING=1
    TAKEOFF=2
    WAYPOINT=3
    LANDING=4
    DISARMING=5

class BackyardFlyer(Drone):
    
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
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
        
        @self.msg_callback(mt.MSG_LOCAL_POSITION)
        def local_position_callback(msg_name,msg):
            print(msg.local_vector)
    
        @self.msg_callback(mt.MSG_STATE)
        def state_callback(msg_name,msg):
            print(msg.guided)

        @self.msg_callback(mt.MSG_GLOBAL_POSITION)
        def global_pos_callback(msg_name, msg):
            print(msg.global_vector)

        @self.msg_callback(mt.MSG_VELOCITY)
        def velocity_callback(msg_name, msg):
            print(msg.local_vector)

    def start(self):
        
        self.start_log("Logs","NavLog.txt")
        #self.connect()
        
        print("starting connection")
        #self.connection.start()
        
        super().start()        
        
        #Only required if they do threaded
        #while self.in_mission:
        #    pass

        self.stop_log()

if __name__ == "__main__":
    drone = BackyardFlyer(threaded=False)
    time.sleep(2)
    drone.start()