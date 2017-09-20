# -*- coding: utf-8 -*-

import threading
import time
import sys
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
                target_system = msg.get_srcSystem()
                target_component = msg.get_srcComponent()
                self.callback(msg.get_type(), msg)

            # want to send a heartbeat periodically, so can just do that when we receive one
            if msg.get_type() == 'HEARTBEAT':
                # print("sending heartbeat")
                # send -> type, autopilot, base mode, custom mode, system status
                self.master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                               mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                               0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)

        print("read ended")

    def send_message(self, msg):
        a = 0
        # TODO: send the message

    def send_waypoint_command(self, hold_time, acceptance_radius, fly_through,
                              yaw, x, y, z):
        self.master.mav.command_int_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1, 0,
            hold_time, acceptance_radius, fly_through, yaw, x, y, z)

    def send_takeoff_command(self, z):
        self.master.mav.command_int_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF_LOCAL, 1,
            0, 0, 0, 0, 0, x, y, z)

    def send_local_position_command(self, x, y, z):
        self.master.mav.command_int_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF_LOCAL, 1,
            0, 0, 0, 0, 0, x, y, z)

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
    
    #This method will be provided to the students
    def __init__(self):
        self.global_position = np.array([None, None, None]) #Longitude, Latitude, Altitude
        self.global_home = np.array([None, None, None])
        self.motors_armed = False
        self.global_velocity = np.array([None, None, None])
        self.heading = None
        self.mode = None
        self.connected = False
		
		
    #This method will be provided to the students
    def connect(self, device):
        self.connection = Connection(device, self.decode_mav_msg)
        while self.connected != True:
            print("Waiting to Connect")
            time.sleep(1)
    
    #This method will be provided to the students
    def disconnect(self):
        self.connection.disconnect()

    #TODO: Fill in this method
    #This method should set the vehicle mode to GUIDED, arm the motors and return True when the motors are successfully armed
    def arm_vehicle(self):
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE, 1, 0,
                                     0, 0, 0, 0, 0)
		
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1,
                                     0, 0, 0, 0, 0, 0)

        while self.motors_armed != True:
            time.sleep(1)

        # Record home (initial) position
        self.global_home = np.copy(self.global_position)
        return True

    #TODO: Fill in this method
    #This method should command the vehicle to an altitude of 3.0m above the takeoff altitude and terminate when the vehicle has successfully reached this altitude
    def takeoff(self,altitude):
        # Set the takeoff position
        take_off_pos = [x for x in self.global_home]
        take_off_pos[2] = take_off_pos[2] + altitude

        print(self.global_home, take_off_pos)
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0,
                                     0, 0,
                                     altitude)

        # Monitor the vehicle altitude until it is within 95% of the specified takeoff altitude
        print(self.global_position[2], take_off_pos[2])
        while self.global_position[2] < 0.9 * take_off_pos[2]:
            print(self.global_position[2], take_off_pos[2])
            time.sleep(0.1)
        print("Exiting takeoff")
        return True

    #TODO: Fill in this method
    #This method command the vehicle to fly to the target location and terminate true when the vehicle has arrived
    def goto(self, target):
        print("Starting GoTo")
        print(target)
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 0, 0, target[1]*10**7, target[0]*10**7, target[2])
        print("Commanded target")
        local_position = global_to_local(self.global_position,self.global_home)
        target_position = global_to_local(target,self.global_home)
        distance_to_target = distance_between(target_position,local_position)
        current_speed = np.linalg.norm(self.global_velocity)
        ## Terminate when within 1.0m of the target AND the vehicle velocity is lower than 0.1 m/s
        while (distance_to_target > 0.5 or current_speed > 0.1):
            time.sleep(0.1)            
            local_position = global_to_local(self.global_position,self.global_home)
            distance_to_target=distance_between(target_position,local_position)
            current_speed = np.linalg.norm(self.global_velocity)            
            print(distance_to_target)


        return True

    #TODO: Fill in this method
    #This method should command the vehicle to land at its current location and terminate true when on the ground
    def land(self):
        #Set the position below ground level
        land_pos = np.copy(self.global_position)
        land_pos[2] = self.global_home[2] - 1.0

        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0,
                                     land_pos[0], land_pos[1], 5.0)

        print(self.global_position[2]-land_pos[2])
        #Monitor both the position and the velocity, terminate when the vehicle is near the ground with a near-zero vertical velocity
        while self.global_position[2] - land_pos[2] > 0.1:
            if self.global_position[2] - self.global_home[2] < 0.1:
			    if abs(self.global_velocity[2]) < 0.1:
					return True
            print(self.global_position[2]-land_pos[2])
            time.sleep(0.1)
        print("Finished Landing")
        return True

    #TODO: Fill in this method
    #This method should disarm the vehicle and return true when they are successfully disarmed
    def disarm_vehicle(self):
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                     0, 0, 0, 0, 0, 0)

        while self.motors_armed:
            time.sleep(1)

        return True

    #This method will be provided to the students
    def decode_mav_msg(self, name, msg):
        # NOTE: this effectively becomes a callback of the main connection thread
        #This will be implemented for the students and sort the mavlink message to different callbacks for different types
        #It may actually just populate the vehicle class data directly
        
        if name is 'STATUSTEXT':
            name # Do nothing right now
			#print(msg.text)
        elif name is 'HEARTBEAT':
            # print('Heartbeat Message')
            self.connected = True
            self.motors_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            # print(self.motors_armed)
            # TODO: correctly parse the state information
        elif name is 'GLOBAL_POSITION_INT':
            self.global_position[0] = float(msg.lon) / (10 ** 7)
            self.global_position[1] = float(msg.lat) / (10 ** 7)
            self.global_position[2] = float(msg.relative_alt) / 1000
            self.global_velocity[0] = float(msg.vx) / 100
            self.global_velocity[1] = float(msg.vy) / 100
            self.global_velocity[2] = -float(msg.vz) / 100
            self.heading = float(msg.hdg) / 100
        # TODO: add other messages of interest....
        else:
            print(name)


#TODO: Fill in this function
#This function should command the vehicle to arm, takeoff to a 3m altitude, fly a 10m box at 3m altitude, land and disarm 
def takeoff_and_fly_box(drone):
    print('Arming Vehicle')
    drone.arm_vehicle()
    time.sleep(1)
    print('Launching Vehicle')
    drone.takeoff(3.0)
    time.sleep(1)    

    #Box waypoints specified in global frame
    global box_waypoints
    box_waypoints = calculate_box(drone.global_home)
    print("Done calculating box")

    for i in range(len(box_waypoints)):
        next_waypoint = box_waypoints[i, :]
        drone.goto(next_waypoint)
        time.sleep(1)

    time.sleep(2)
    print('Landing')
    drone.land()
    time.sleep(1)

    print('Disarming Vehicle')
    drone.disarm_vehicle()
    time.sleep(1)


#This function will not be provided, it is part of my solution
def calculate_box(global_home):
    
    global_waypoints = np.zeros((4, 3))
    local_waypoints = np.array([[10.0, 0.0, -3.0],[10.0, 10.0, -3.0],[0.0, 10.0, -3.0],[0.0, 0.0, -3.0]])
    for i in xrange(0,4):
        global_waypoints[i, :] = local_to_global(local_waypoints[i, :], global_home)

    return global_waypoints


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


M2Latitude = 1.0 / 111111.0;

latitude0 = 37.412939;
longitude0 = 121.995635;
M2Longitude = 1.0 / (0.8 * 111111.0);

#Solve for the distance between two local positions
def distance_between(position1, position2):
    sum_square = 0.0
    for i in range(len(position1)):
        sum_square = sum_square + np.power(position1[i] - position2[i],2)
    return np.sqrt(sum_square)



# This is the 
if __name__ == "__main__":
    drone = Drone()
    drone.connect("tcp:127.0.0.1:5760")
    
    time.sleep(2)
    takeoff_and_fly_box(drone)
        
    # terminate the connection
    drone.disconnect()