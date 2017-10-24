"""
Message types

Besides the state message, the other types are defined by the frame they are defined in

"""

# enum for he names to use for setting callbacks
# this is to ensure that all the files are using the same names for the messages
# implementations should use the variable names instead of the strings to minimize typo based errors
MSG_STATE = 'state_msg'
MSG_GLOBAL_POSITION = 'global_position_msg'
MSG_LOCAL_POSITION = 'local_position_msg'
MSG_GLOBAL_HOME = 'global_home_msg'
MSG_VELOCITY = 'local_velocity_msg'
MSG_CONNECTION_CLOSED = 'connection_closed_msg'


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
    def __init__(self,*args):
        if len(*args==4):
            self.init_euler(args[0],args[1],args[2],args[3])
        elif len(*args==5):
            self.init_quaternion(args[0],args[1],args[2],args[3],args[4])
    
    def init_euler(self,time,roll,pitch,yaw):
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

    def init_quaternion(self,time,q0,q1,q2,q3):
        super().__init__(time)
        self._qo = q0
        self._q1 = q1
        self._q2 = q2
        self._q3 = q3
        
        self._roll = math.degrees(math.atan2(2.0*(q0*q1+q2*q3),1.0-2.0*(q1**2+q2**2)))
        self._pitch = math.degrees(math.asin(2.0*(q0*q2-q3*q1)))
        self._yaw = math.degrees(math.atain2(2.0*(q0*q3+q1*q2),1.0-2.0*(q2**2+q3**2)))
        
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

    
