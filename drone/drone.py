# -*- coding: utf-8 -*-

import numpy as np
import connection
import math




class Drone:
    
    #This method will be provided to the students
    def __init__(self):
        
        #Global position in degrees
        self._longitude = 0.0
        self._latitude = 0.0
        self._altitude = 0.0
        
        #Reference home position in degrees
        self._home_longitude = 0.0
        self._home_latitude = 0.0
        self._home_altitude = 0.0
        
        #Local positions in meters from the global home
        self._north = 0.0
        self._east = 0.0
        self._down = 0.0
        
        #Locally oriented velocity in m/s
        self._velocity_north = 0.0
        self._velocity_east = 0.0
        self._velocity_down = 0.0
        
        #Vehicle state information
        self._armed = False
        self._guided = False
        self._connected = False
        
        #Euler angles in degrees defined in 3-2-1 rotation
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        
        #Body accelerations
        self._acceleration_x = 0.0
        self._acceleration_y = 0.0
        self._acceleration_z = 0.0
        
        #Gyro rates
        self._gyro_x = 0.0
        self._gyro_y = 0.0
        self._gyro_z = 0.0
        
        #Barometer
        self._baro_altitude = 0.0
        
        self._update_property = {'state_msg':self._update_state,
                            'global_position_msg':self._update_global_position,
                            'local_position_msg':self._update_local_position,
                            'global_home_msg':self._update_global_home,
                            'local_velocity_msg':self._update_local_velocity,
                            'gyro_raw_msg':self._update_gyro_raw,
                            'acceleration_raw_msg':self._update_acceleration_raw,
                            'euler_angle_msg':self._update_euler_angle,
                            'baro_msg':self._update_baro}
		
        
    @property
    def global_position(self):
        return np.array([self._longitude,self._latitude,self._altitude])
    
    def _update_global_position(self,msg):
        self._longitude = msg.longitude
        self._latitude = msg.latitude
        self._altitude = msg.altitude
    
    @property
    def global_home(self):
        return np.array([self._home_longitude,self._home_latitude,self._home_altitude])
    
    def _update_global_home(self,msg):
        self._home_longitude = msg.longitude
        self._home_latitude = msg.latitude
        self._home_altitude = msg.altitude
    
    @property
    def local_position(self):
        return np.array([self._north,self._east,self._down])
    
    def _update_local_position(self,msg):
        self._north = msg.north
        self._east = msg.east
        self._down  = msg.down
    
    @property
    def local_velocity(self):
        return np.array([self._velocity_north,self._velocity_east,self._velocity_down])
    
    def _update_local_velocity(self,msg):
        self._velocity_north = msg.north
        self._velocity_east = msg.east
        self._velocity_down = msg.down
    
    @property
    def armed(self):
        return self._armed
    
    @property
    def guided(self):
        return self._guided
    
    @property
    def connected(self):
        return self._connected
    
    def _update_state(self,msg):
        self._armed = msg.armed
        self._guided = msg.guided
        self._connected = True
    
    @property
    def euler_angle(self):
        return np.array([self._roll,self._pitch,self._yaw])
    
    def _update_euler_angle(self,msg):
        self._roll = msg.roll
        self._pitch = msg.pitch
        self._yaw = msg.yaw
    
    @property
    def acceleration_raw(self):
        return np.array([self._acceleration_x,self._acceleration_y,self._acceleration_z])
    
    def _update_acceleration_raw(self,msg):
        self._acceleration_x = msg.x
        self._acceleration_y = msg.y
        self._acceleration_z = msg.z
    
    @property
    def gyro_raw(self):
        return np.array([self._gryo_x,self._gyro_y,self._gyro_z])
    
    def _update_gyro_raw(self,msg):        
        self._gyro_x = msg.x
        self._gyro_y = msg.y
        self._gyro_z = msg.z
    
    @property
    def barometer(self):
        return np.array(self._baro_altitude)
    
    def _update_barometer(self,msg):
        self._baro_altitude = msg.altitude
    
    #@connection.on_messages('*')
    def on_message_receive(self,msg_name,msg):
        """Sorts incoming messages, updates the drone state variables and runs callbacks"""
        if msg_name in self._update_property.keys():
            self._update_property[msg_name]
            
        if msg_name in self._msg_callbacks.keys():
            self._msg_callbacks[msg_name]

    
    def msg_callback(self,msg_name):
        def decorator(fn):
            self._msg_callbacks[msg_name] = fn        
        return decorator
    
    
    #Command Methods
    def connect(self, device):
        ''' Conect to the specified device'''
        self.connection = connection.Connection(device)
        timeout = self.connection.wait_for_message()
        
        if ~timeout:
            self.connected = True
        
    def arm(self,armed):
        """Send an arm/disarm command to the vehicle
            armed: True- arm, False-disarm
        """
        try:
            self.connection.arm(armed)
        except:
            print("arm command not defined")
    
    def take_control(self,control):
        """Toggle the control of the vehicle
            control: True-computer control, False-Manual control
        """
        try:
            self.connection.take_control(control)
        except:
            print("take_controm command not defined")
    
    def cmd_position(self,north,east,down,heading):
        """ Command the local position and vehicle heading
            north: local north in meters
            east: local east in meters
            down: local down in meters (positive down)
            heading: vehicle yaw in degrees
        """
        try:
            self.connection.cmd_position(north,east,down,heading)
        except:
            print("cmd_position not defined")
    
    def takeoff(self,target_altitude):
        """Command the vehicle to takeoff to the target_alt (in meters)"""
        try:
            self.connection.takeoff(target_altitude)
        except:
            print("takeoff no defined")
    
    def land(self):
        """Command the vehicle to land at its current position"""
        try:
            self.connection.land()
        except:
            print("land not defined")
    
    def cmd_attitude_rate(self,roll_rate,pitch_rate,yaw_rate,collective):
        """Command the vehicle orientation rates
            roll_rate,pitch_rate,yaw_rate: in deg/s
            collective: upward acceleration in m/s**2
        """
        try:
            self.connection.cmd_attitude_rate(roll_rate,pitch_rate,yaw_rate,collective)
        except:
            print("cmd_attitude_rate not defined")
    
    def cmd_velocity(self,velocity_north,velocity_east,velocity_down,heading):
        """Command the vehicle velocity
            north_velocity,east_velocity,down_velocity: in m/s
            heading: in degrees
        """
        try:
            self.connection.cmd_velocity(velocity_north,velocity_east,velocity_down,heading)
        except:
            print("cmd_velocity not defined")
    
    def cmd_motors(self,motor_rpm):
        """Command the rmp of the motors"""
        try:
            self.connection.cmd_motors(motor_rpm)
        except:
            print("cmd_motors not defined")
            

""" Message types

Besides the state message, the other types are defined by the frame they are defined in

"""
    
class Message:
    """Message superclass, all messages will contain a time"""
    def __init__(self,time):
        self._time = time
    
    @property
    def time(self):
        return self._time

class StateMessage(Message):
    """Vehicle state change messages"""
    def __init__(self,time,armed,guided):
        super().__init__(time)
        self._armed = armed
        self._guided = guided

    @property
    def armed(self):
        return self._armed
    
    @property
    def guided(self):
        return self._guided
    
    
class GlobalFrameMessage(Message):
    """Messages defined in the Global (Lat,Lon, Alt) frame"""
    def __init__(self,time,latitude,longitude,altitude):
        super().__init__(time)
        self._longitude = longitude
        self._latitude = latitude
        self._altitude = altitude
    
    @property
    def longitude(self):
        return self._longitude
    
    @property
    def latitude(self):
        return self._latitude
    
    @property
    def altitude(self):
        return self._altitude
    
    @property
    def global_vector(self):
        return np.array([self._longitude,self._latitude,self._altitude])

class LocalFrameMessage(Message):
    """Messages defined in the Local (North,East,Down) frame"""
    def __init__(self,time,north,east,down):
        super().__init__(time)
        self._north = north
        self._east = east
        self._down = down
       
    @property
    def north(self):
        return self._north
    
    @property
    def east(self):
        return self._east
    
    @property
    def down(self):
        return self._down
    
    @property
    def local_vector(self):
        return np.array([self._north,self._east,self._down])

class BodyFrameMessage(Message):
    """Messages defined in the body frame (x,y,z)"""
    def __init__(self,time,x,y,z):
        super().__init__(time)
        self._x = x
        self._y = y
        self._z = z
        
    @property
    def x(self):
        return self._x
    
    @property
    def y(self):
        return self._y
    
    @property
    def z(self):
        return self._z
    
    @property
    def body_vector(self):
        return np.array([self._x,self._y,self._z])
    
class FrameMessage(Message):
    """Messages defining the rotation between frames (Euler angles or Quaternions)"""
    def ___init__(self,time,roll,pitch,yaw):
        super().__init__(time)
        self._roll = roll
        self._pitch = pitch
        self._yaw = yaw
        
        sp = math.sin(math.radians(pitch/2.0))
        cp = math.cos(math.radians(pitch/2.0))
        sr = math.sin(math.radians(roll/2.0))
        cr = math.cos(math.radians(roll/2.0))
        sy = math.sin(math.radians(yaw/2.0))
        cy = math.cos(math.radians(yaw/2.0))
        
        self._qo = cr*cp*cy+sr*sp*sy
        self._q1 = sr*cp*cy-cr*sp*sy
        self._q2 = cr*sp*cy+sr*cp*sy
        self._q3 = cr*cp*sy-sr*sp*cy
    """ I need to figure out the right way to do multiple constructors for a class
    def __init__(self,time,q0,q1,q2,q3):
        super().__init__(time)
        self._qo = q0
        self._q1 = q1
        self._q2 = q2
        self._q3 = q3
        
        self._roll = math.degrees(math.atan2(2.0*(q0*q1+q2*q3),1.0-2.0*(q1**2+q2**2)))
        self._pitch = math.degrees(math.asin(2.0*(q0*q2-q3*q1)))
        self._yaw = math.degrees(math.atain2(2.0*(q0*q3+q1*q2),1.0-2.0*(q2**2+q3**2)))
        
    """
    @property
    def roll(self):
        return self._roll
    
    @property
    def pitch(self):
        return self._roll
    
    @property
    def yaw(self):
        return self._yaw
    
    @property
    def euler_angles(self):
        return np.array([self._roll,self._pitch,self._yaw])
    
    @property
    def quaternions(self):
        return np.array([self._q0,self._q1,self._q2,self.q3])
    
    @property
    def q0(self):
        return self._q0
    
    @property
    def q1(self):
        return self._q1
    
    @property
    def q2(self):
        return self._q2
    
    @property
    def q3(self):
        return self._q3

    
