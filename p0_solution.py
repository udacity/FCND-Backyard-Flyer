# -*- coding: utf-8 -*-

import threading
import time
import numpy as np
from pymavlink import mavutil
import utm
import os

# force use of mavlink v2.0
os.environ['MAVLINK20'] = '1'

class Connection:
    def __init__(self, device, fn):
        self.callback = fn
        # TODO: mavutil connection here...
        self.master = mavutil.mavlink_connection(device, source_system=190)
        self._running = True
        self.read_handle = threading.Thread(
            target=self.read_thread)  # TODO: start read thread here
        self.read_handle.start()
        self.write_handle = 0  # TODO: decide if want a write thread, and if so, start it here

        self.target_system = 0
        self.target_component = 0


    def read_thread(self):

        while self._running:

            # get the next message
            # NOTE: this is a blocking call, which is why we have a thread for it
            msg = self.master.recv_match(blocking=True)

            # if it's a good message, send it back to the callback
            if msg.get_type() != 'BAD_DATA':
                self.callback(msg.get_type(), msg)

            # want to send a heartbeat periodically, so can just do that when we receive one
            if msg.get_type() == 'HEARTBEAT':
                # print("sending heartbeat")
                # send -> type, autopilot, base mode, custom mode, system status
                self.master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                               mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                               0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)

        print("read ended")

    
    def send_mav_command(self, command_type, param1, param2, param3, param4, x,
                         y, z):
        self.master.mav.command_int_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, command_type, 1, 0, param1,
            param2, param3, param4, x, y, z)

    def arm_drone(self):
        self.master.arducopter_arm()

    def disarm_drone(self):
        self.master.arducopter_disarm()

    def disconnect(self):
        # stop running the read loop, and wait for the thread to finish
        self._running = False
        self.read_handle.join()

        # close the mavutil connection (tcp/udp/serial, etc)
        self.master.close()


class Drone:
    def __init__(self):
        self.global_position = np.array([None, None, None]) #Longitude, Latitude, Altitude
        self.global_home = np.array([None, None, None])
        self.motors_armed = False
        self.global_velocity = np.array([None, None, None])
        self.heading = None
        self.mode = None
		
		
    #Provided
    def connect(self, device):
        self.connection = Connection(device, self.decode_mav_msg)
    
    #Provided
    def disconnect(self):
        self.connection.disconnect()

    
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
            self.motors_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            # Correctly parse the state information
        elif name is 'GLOBAL_POSITION_INT':
            self.global_position[0] = float(msg.lon) / (10 ** 7)
            self.global_position[1] = float(msg.lat) / (10 ** 7)
            self.global_position[2] = float(msg.relative_alt) / 1000
            self.global_velocity[0] = float(msg.vx) / 100
            self.global_velocity[1] = float(msg.vy) / 100
            self.global_velocity[2] = -float(msg.vz) / 100
            self.heading = float(msg.hdg) / 100
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
        -global_position[2] - global_home[2]
    ]
    return local_position


#Convert a local position (north,east,down) relative to the home position to a global position (lon,lat,up)
def local_to_global(local_position, global_home):
    (east_home, north_home, zone_number, zone_letter) = utm.from_latlon(
        global_home[1], global_home[0])
    (lat, lon) = utm.to_latlon(east_home + local_position[1],
                               north_home + local_position[0], zone_number,
                               zone_letter)
                               
    lla = [lon, lat, -local_position[2] - global_home[2]]
    return lla


#Solve for the distance between two local positions
def distance_between(position1, position2):
    sum_square = 0.0
    for i in range(len(position1)):
        sum_square = sum_square + np.power(position1[i] - position2[i],2)
    return np.sqrt(sum_square)


# run the script here to just do a couple simple things to see if it is working
if __name__ == "__main__":
    drone = Drone()
    drone.connect("tcp:127.0.0.1:5760")
    
    time.sleep(2)
    takeoff_and_fly_box(drone)
        
    # terminate the connection
    drone.disconnect()