
from pymavlink import mavutil
import os
import connection


# force use of mavlink v2.0
os.environ['MAVLINK20'] = '1'

class MavlinkConnection(connection.Connection):

    def __init__(self, device):
        super().__init__()
        self.master = mavutil.mavlink_connection(device, source_system=190)
        self._running = True
        #self.read_handle = threading.Thread(
        #    target=self.read_thread)
        #self.read_handle.start()
        self.write_handle = 0

        self.target_system = 0
        self.target_component = 0
        self.callbacks = {}

    def start(self):
        pass

    def arm(self):
        pass

    def disarm(self):
        pass

    def take_control(self):
        pass

    def release_control(self):
        pass

    def cmd_attitude(self, yaw, pitch, roll, collective):
        pass

    def cmd_attitude_rate(self, yaw_rate, pitch_rate, roll_rate, collective):
        pass

    def cmd_velocity(self, vn, ve, vd, heading):
        pass

    def cmd_motors(self, motor1, motor2, motor3, motor4):
        pass

    def cmd_position(self, n, e, d, heading):
        pass

    def takeoff(self, alt):
        pass

    def land(self):
        pass

    def set_home_position(self, lat, lon, alt):
        pass
