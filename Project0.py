# -*- coding: utf-8 -*-


import threading
import time
import sys
import numpy as np
from pymavlink import mavutil
import mavlink


class Connection:

    def __init__(self, device, fn):
        self.callback = fn
        # TODO: mavutil connection here...
        self.master = mavutil.mavlink_connection(device, source_system=190)
        self._running = True
        self.read_handle = threading.Thread(target=self.read_thread)  # TODO: start read thread here
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
                target_system = msg.get_srcSystem();
                target_component = msg.get_srcComponent();
                self.callback(msg.get_type(), msg)

            # want to send a heartbeat periodically, so can just do that when we receive one
            if msg.get_type() == 'HEARTBEAT':
                #print("sending heartbeat")
                # send -> type, autopilot, base mode, custom mode, system status
                self.master.mav.heartbeat_send(mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavlink.MAV_STATE_ACTIVE)


        print("read ended")

    def send_message(self, msg):
        a = 0
        # TODO: send the message

    def send_waypoint_command(self, hold_time, acceptance_radius, fly_through, yaw, x, y, z):
        self.master.mav.command_int_send(self.target_system, self.target_component, mavlink.MAV_FRAME_LOCAL_NED, mavlink.MAV_CMD_NAV_WAYPOINT, 1, 0, hold_time, acceptance_radius, fly_through, yaw, x, y, z);

    def send_takeoff_command(self, z):
        self.master.mav.command_int_send(self.target_system, self.target_component, mavlink.MAV_FRAME_LOCAL_NED, mavlink.MAV_CMD_NAV_TAKEOFF_LOCAL, 1, 0, 0, 0, 0, 0, x, y, z)

    def send_local_position_command(self, x, y, z):
        self.master.mav.command_int_send(self.target_system, self.target_component, mavlink.MAV_FRAME_LOCAL_NED, mavlink.MAV_CMD_NAV_TAKEOFF_LOCAL, 1, 0, 0, 0, 0, 0, x, y, z)

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


class drone_vehicle:
	#gps_location
    #gps_home
    #local_position
    #motors_armed
    #is_armable #Optional if I can that data
    #velocity
    
    def __init__(self):
        self.gps_location = []
        self.gps_home = []
        self.motors_armed = []
        self.velocity = []
        self.local_position = []
        self.armed = False
        
    #TODO: this function will be completed for the students
    def connect(self, device):
        self.connection = Connection(device, self.decode_mav_msg)

    def disconnect(self):
        self.connection.disconnect()
        
    #The student should set the mode to guided, arm the vehicle (with checks) and return the global location as the home
    def arm_vehicle(self):
        #TODO: Insert mavlink command for setting the mode to Guided
        #mav.mission_item_send(mav.target_system, mav.target_component, mav.seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z, mission_type, force_mavlink1=False):
        
        #TODO: Insert the mavlink command for arming 
        #vehicle.armed = True
        self.connection.arm_drone()

        while ~self.armed:
            time.sleep(1)
            
        self.gps_home = self.gps_location
        return True

    #TODO: the students will write this function
    def takeoff(self):
        takeoff_pos = self.gps_home
        takeoff_pos[2] = takeoff_pos[2]+1.5
        
        #TODO: Insert mavlink command
        #vehicle.simple_goto(takeoff_pos)
        self.connection.send_local_position_command(0, 0, -5)
        
        while(self.gps_position < 0.95*takeoff_pos[2]):
            time.sleep(0.1)
        
        return True

    #TODO: the students will write this function
    def goto(self, target):
        #TODO: Insert mavlink command
        #vehicle.simple_goto(target)
        
        while(_distance_between(target,self.local_position)>0.5):
            time.sleep(0.1)
            
        return True
    
    #TODO: the students will write this function
    def land(self):
        land_pos = self.gps_position
        land_pos[2] = self.gps_home[2]-1.0
            
        #TODO: Insert mavlink command
        #vehicle.simple_goto(land_pos)
    
        while self.gps_location[2]-land_pos[2]<0.1:
            if self.gps_location[2]-self.gps_home[2]<0.1 :
                if self.velocity[2]<0.1 :
                    return True
                
        
        return False

    #TODO: the students will fill out this function
    def disarm_vehicle(self):
        #TODO: 
        #vehicle.armed = True
        while self.armed:
            time.sleep(1)
        
        return True
        
    def decode_mav_msg(self, name, msg):
        # NOTE: this effectively becomes a callback of the main connection thread
        #This will be implemented for the students and sort the mavlink message to different callbacks for different types
        #It may actually just populate the vehicle class data directly
        if name is 'STATUSTEXT':
            print msg.text
        elif name is 'HEARTBEAT':
            self.armed = (msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            # TODO: correctly parse the state information
        #elif name is 'GPS_INT':
            # TODO: populate
        #elif name is 'LOCAL_POSITION':
            # TODO: populate:
        # TODO: add other messages of interest....
        else:
            print name
		

#TODO: The students will write this function (or something similar)
def takeoff_and_fly_box(drone):
	print 'Arming Vehicle'
	drone.arm_vehicle()
	print 'Vehicle Armed'
	
	print 'Launching Vehicle'
	drone.takeoff()
	print 'Successfully launched to %f m height'.format(drone.local_position[2])
	
	box_waypoints = calculate_box(drone.gps_home)
	
	for i in range(4):
		next_waypoint = box_waypoints[i,:]
		print 'Going to next waypoint: (%f,%f,%f)'.format(next_waypoint[1], next_waypoints[2], next_waypoint[3])
		drone.goto(next_waypoint)
		print 'Arrived at waypoint, vehicle position = (%f,%f,%f)'.format(drone.gps_location[1], drone.gps_location[2], drone.gps_location[3])
	
	print 'Landing'
	drone.land()

	print 'Disarming Vehicle'
	drone.disarm_vehicle()
	

def _distance_between(position1,position2):
	sum_square = 0.0
	for i in range(0,2):
		sum_square = sum_square + np.pow(position1[i]-position2[i])
		
	return np.sqrt(sum_square)



# run the script here to just do a couple simple things to see if it is working
if __name__ == "__main__":
    drone = drone_vehicle()
    drone.connect("tcp:127.0.0.1:5760")

    time.sleep(20)
    drone.arm_vehicle()
    drone.takeoff()

    # terminate the connection
    drone.disconnect()
