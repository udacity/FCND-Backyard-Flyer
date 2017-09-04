# -*- coding: utf-8 -*-


import time
import sys
import numpy as np



class drone_vehicle:
	gps_location
	gps_home
	local_position
	motors_armed
	is_armable #Optional if I can that data
	velocity
	
	def __init__(self):
		gps_location = []
		gps_home = []
		motors_armed = []
		velocity = []
		local_position = []
		
	#TODO: this function will be completed for the students
	def connect(self):
		#TODO: Add the vehicle connection information
		
	#The student should set the mode to guided, arm the vehicle (with checks) and return the global location as the home
	def arm_vehicle(self):
		#TODO: Insert mavlink command for setting the mode to Guided
		#mav.mission_item_send(mav.target_system, mav.target_component, mav.seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z, mission_type, force_mavlink1=False):
		
		#TODO: Insert the mavlink command for arming 
		#vehicle.armed = True
		
		while ~vehicle.armed:
			time.pause(1)
			
		self.home_gps = self.gps_location
		return True

	#TODO: the students will write this function
	def takeoff(self):
		takeoff_pos = home_gps
		takeoff_pos[2] = takeoff_pos[2]+1.5
		
		#TODO: Insert mavlink command
		#vehicle.simple_goto(takeoff_pos)
		
		while(self.gps_position < 0.95*takeoff_pos[2]):
			time.pause(0.1)
		
		return True

	#TODO: the students will write this function
	def goto(self, target):
		#TODO: Insert mavlink command
		#vehicle.simple_goto(target)
		
		while(_distance_between(target,self.local_position)>0.5):
			time.pause(0.1)
			
		return True
	
	#TODO: the students will write this function
	def land(self):
		land_pos = self.gps_position
		land_pos[2] = home_gps[2]-1.0
			
		#TODO: Insert mavlink command
		#vehicle.simple_goto(land_pos)
	
		while self.gps_location[2]-land_pos[2]<0.1:
			if self.gps_location[2]-home_gps[2]<0.1 :
				if vehicle.velocity[2]<0.1 :
					return True
				
		
		return False

	#TODO: the students will fill out this function
	def disarm_vehicle(self):
		#TODO: 
		#vehicle.armed = True
		while vehicle.armed:
			time.sleep(1)
		
		return True
		
	def decode_mav_msg(self):
		#This will be implemented for the students and sort the mavlink message to different callbacks for different types
		#It may actually just populate the vehicle class data directly
		

#TODO: The students will write this function (or something similar)
def takeoff_and_fly_box(drone):
	print 'Arming Vehicle'
	drone.arm_vehicle()
	print 'Vehicle Armed'
	
	print 'Launching Vehicle'
	drone.takeoff()
	print 'Successfully launched to %f m height' drone.local_position[2]
	
	box_waypoints = calculate_box(drone.gps_home)
	
	for i in range(4):
		next_waypoint = box_waypoints[i,:]
		print 'Going to next waypoint: (%f,%f,%f)' next_waypoint[1], next_waypoints[2], next_waypoint[3]
		drone.goto(next_waypoint)
		print 'Arrived at waypoint, vehicle position = (%f,%f,%f)' drone.gps_location[1], drone.gps_location[2], drone.gps_location[3]
	
	print 'Landing'
	drone.land()
	
	print 'Disarming Vehicle'
	drone.disarm_vehicle()
	

def _distance_between(position1,position2):
	sum_square = 0.0
	for i in range(0,2):
		sum_square = sum_square + np.pow(position1[i]-position2[i])
		
	return np.sqrt(sum_square)
