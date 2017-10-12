
from pymavlink import mavutil
import os
import connection
import message_types as mt


# force use of mavlink v2.0
os.environ['MAVLINK20'] = '1'

class MavlinkConnection(connection.Connection):

    def __init__(self, device):
        super().__init__()
        self._master = mavutil.mavlink_connection(device, source_system=190)
        self._read_handle = threading.Thread(target=self.read_thread)
        self._read_handle.daemon = True

        self._target_system = 0
        self._target_component = 0

    def read_thread(self):
        while (self._running):
            msg = self.wait_for_message()

            # if no message or a bad message was received, just move along
            if msg is None:
                continue

            # parse out the message based on the type and call
            # the appropriate callbacks
            if msg.get_type == 'GLOBAL_POSITION_INT':
                # parse out the gps position and trigger that callback
                gps = mt.GlobalPosition(msg.lat/1e7, msg.lon/1e7, msg.alt/1000)
                self.notify_message_listeners(mt.MSG_GLOBAL_POSITION, gps)

                # parse out the velocity and trigger that callback
                vel = mt.Velocity(msg.vx/100, msg.vy/100, msg.vx/100)
                self.notify_message_listeners(mt.MSG_VELOCITY, vel)

            # TODO: parse out additional message types



    def wait_for_message(self):
        msg = self._master.recv_match(blocking=True,timeout=10)
        if msg is None:
            # no message received
            return None
        else:
            if(msg.get_type() == 'BAD_DATA'):
                # no message that is useful
                return None
                
            # send a heartbeat message back, since this needs to be constantly sent so the autopilot knows this exists
            if msg.get_type() == 'HEARTBEAT':
                # send -> type, autopilot, base mode, custom mode, system status
                self._master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                           mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                           0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)

            # pass the message along to be handled
            return msg


    def start(self):
        # start the main thread
        self._running = True
        self._read_handle.start()

    def stop(self):
        self._running = False
        self._read_handle.join()

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
