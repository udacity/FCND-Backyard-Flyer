"""
these are just a bunch of classes for the different message types
"""

# enum for he names to use for setting callbacks
MSG_STATE = 'state'
MSG_GLOBAL_POSITION = 'global_position'
MSG_LOCAL_POSITION = 'local_position'
MSG_GLOBAL_HOME = 'global_home'
MSG_VELOCITY = 'velocity'


class State:

    def __init__(self, armed, mode):
        self.armed = armed
        self.mode = mode


class GlobalPosition:

    def __init__(self, lat, lon, alt):
        self.lat = lat
        self.lon = lon
        self.alt = alt


class LocalPosition:

    def __init__(self, north, east, down):
        self.north = north
        self.east = east
        self.down = down

class GlobalHomePosition:

    def __init__(self, lat, lon, alt):
        self.lat = lat
        self.lon = lon
        self.alt = alt


class Velocity:

    def __init__(self, north, east, down):
        self.north = north
        self.east = east
        self.down = down


#class IMU:


#class Attitude: