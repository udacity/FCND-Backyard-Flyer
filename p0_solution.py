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
    def __init__(self):
        self.global_position = np.array([None, None, None]) #Longitude, Latitude, Altitude
        self.global_home = np.array([None, None, None])
        self.motors_armed = False
        self.global_velocity = np.array([None, None, None])
        self.heading = None
        self.mode = None
		
		
    #TODO: this function will be completed for the students
    def connect(self, device):
        self.connection = Connection(device, self.decode_mav_msg)

    def disconnect(self):
        self.connection.disconnect()

    #The student should set the mode to guided, arm the vehicle (with checks) and save the home position as the position it is armed
    def arm_vehicle(self):
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE, 1, 0,
                                     0, 0, 0, 0, 0)
		
		# Ignore for right now
        # while self.mode != GUIDED:
        #    time.sleep(0.1)

        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1,
                                     0, 0, 0, 0, 0, 0)

        while self.motors_armed != True:
            time.sleep(0.1)

        # Record home (initial) position
        self.global_home = np.copy(self.global_position)
        return True

    #TODO: the students will write this function
    def takeoff(self):
        # Set the takeoff position
        take_off_pos = [x for x in self.global_home]
        take_off_pos[2] += 1.5
        take_off_pos[0] = (take_off_pos[0])
        take_off_pos[1] = (take_off_pos[1])
        take_off_pos[2] = (take_off_pos[2])
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0,
                                     int(take_off_pos[0]), int(take_off_pos[1]),
                                     take_off_pos[2])

        # Monitor the vehicle altitude until it is within 95% of the specified takeoff altitude
        # print(self.global_position[2], take_off_pos[2])
        while self.global_position[2] < 0.9 * take_off_pos[2]:
            # print(self.global_position[2], take_off_pos[2])
            time.sleep(0.1)
        print("Exiting takeoff")
        return True

    # TODO: the students will write this function. The target can either be a GPS target or a local target
    def goto(self, target):
        print("Starting GoTo")
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 0, 0, int(target[1]*10**7), int(target[0]*10**7), target[2])
        print("Commanded target")
        local_position = global_to_local(self.global_position, self.global_home)
        target_position = global_to_local(target,self.global_home)
        distance_to_target = distance_between(target_position,local_position)
		## Terminate when within 1.0m of the target
        while (distance_to_target > 1.0):
            time.sleep(0.1)            
            local_position = global_to_local(self.global_position,self.global_home)
            distance_to_target=distance_between(target_position,local_position)
            # print(distance_to_target)


        return True

    #TODO: the students will write this function
    def land(self):
        #Set the position below ground level
        land_pos = np.copy(self.global_position)
        land_pos[2] = self.global_home[2]

        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0,
                                     int(land_pos[0]), int(land_pos[1]), land_pos[2])


        #Monitor both the position and the velocity
        while self.global_position[2] > 1.5 * land_pos[2]:
            time.sleep(0.1)
        print("Finished Landing")
        return True

    #TODO: the students will fill out this function
    def disarm_vehicle(self):
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                     0, 0, 0, 0, 0, 0)

        while self.motors_armed:
            time.sleep(1)

        return True

    def decode_mav_msg(self, name, msg):
        # NOTE: this effectively becomes a callback of the main connection thread
        #This will be implemented for the students and sort the mavlink message to different callbacks for different types
        #It may actually just populate the vehicle class data directly
        if name is 'STATUSTEXT':
            a = 1
			#print(msg.text)
        elif name is 'HEARTBEAT':
            # print('Heartbeat Message')
            self.motors_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
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


#TODO: The students will write this function (or something similar)
def takeoff_and_fly_box(drone):
    print('Arming Vehicle')
    drone.arm_vehicle()
    print('Vehicle Armed')

    print('Launching Vehicle')
    drone.takeoff()
    #print('Successfully launched to %f m height'.format(drone.local_position[0,2]))

    #Box waypoints specified in global frame
    box_waypoints = calculate_box(drone.global_home)
    print("Done calculating box")

    for i in range(len(box_waypoints)):
        next_waypoint = box_waypoints[i, :]
        #print('Going to next waypoint: (%f,%f,%f)'.format(
        #    next_waypoint[0,0], next_waypoint[0,1], next_waypoint[0,2]))
        drone.goto(next_waypoint)
        #print('Arrived at waypoint, vehicle position = (%f,%f,%f)'.format(
        #    drone.global_position[0], drone.global_position[1],
        #    drone.global_position[2]))

    time.sleep(2)
    print('Landing')
    drone.land()

    print('Disarming Vehicle')
    drone.disarm_vehicle()


def calculate_box(global_home):
    global_waypoints = np.zeros((4, 3))
    local_waypoints = np.array([[10.0, 0.0, -3.0],[10.0, 10.0, -3.0],[0.0, 10.0, -3.0],[0.0, 0.0, -3.0]])
    for i in range(len(local_waypoints)):
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
