
from pymavlink import mavutil
import os
import threading
import connection
import message_types as mt


# force use of mavlink v2.0
os.environ['MAVLINK20'] = '1'

class MavlinkConnection(connection.Connection):

    def __init__(self, device):
        self._message_listeners = {} #super.__init__()
        #self._master = mavutil.mavlink_connection(device, source_system=190)
        self._read_handle = threading.Thread(target=self.read_thread)
        self._read_handle.daemon = True
        self._running = False

        self._target_system = 0
        self._target_component = 0

    def testcallback(self):
        print("testing callback")
        self.notify_message_listeners('test', [])


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

            elif msg.get_type() == 'HEARTBEAT':
                motors_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                # TODO: parse out if in offboard mode or manual mode (or straight up parse out the control mode...?)
        
                state = mt.State(motors_armed, mode)
                self.notify_message_listeners(mt.MSG_STATE, state)

            elif msg.get_type() == 'LOCAL_POSITION_NED':
                # parse out the local positin and trigger that callback
                pos = mt.LocalPosition(msg.x, msg.y, msg.z)
                self.notify_message_listeners(mt.MSG_LOCAL_POSITION, pos)

                # parse out the velocity and trigger that callback
                vel = mt.Velocity(msg.vx, msg.vy, msg.vz)
                self.notify_message_listeners(mt.MSG_VELOCITY, vel)

            elif msg.get_type() == 'HOME_POSITION':
                # TODO: decode position information
                home = mt.GlobalHomePosition()
                self.notify_message_listeners(mt.MSG_GLOBAL_HOME, home)

            #elif msg.get_type() == 'ATTITUDE':



            # TODO: parse out additional message types



    def wait_for_message(self):
        # NOTE: this returns a mavlink message
        # this function should not be called outside of this class!
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

            # pass the message along to be handled by this class
            return msg


    def start(self):
        # start the main thread
        self._running = True
        self._read_handle.start()

    def stop(self):
        self._running = False
        self._read_handle.join()
        self._master.close()

    def send_long_command(self, command_type, param1, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
        # send a command long helper function
        # TODO: decide how to handle the frame part...
        self._master.mav.command_long_send(
            self._target_system, self._target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, command_type,
            param1, param2, param3, param4, param4, param6, param7)

    def arm(self):
        # send an arm command through mavlink
        self.send_long_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)

    def disarm(self):
        # send a disarm command
        self.send_long_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)

    def take_control(self):
        # TODO: look at PX4 documentation to figure out what custom and base modes they use
        mode = 0
        custom_mode = 0
        base_mode = 0
        self.send_long_command(mavutil.MAV_CMD_DO_SET_MODE, mode, custom_mode, base_mode)

    def release_control(self):
        # TODO: look at PX4 documentation to figure out what custom and base modes they use
        mode = 0
        custom_mode = 0
        base_mode = 0
        self.send_long_command(mavutil.MAV_CMD_DO_SET_MODE, mode, custom_mode, base_mode)

    def cmd_attitude(self, yaw, pitch, roll, collective):
        time_boot_ms = 0  # TODO: figure out if this needs to be sent properly
        # TODO: convert the attitude to a quaternion
        q = [0, 0, 0, 0]
        mask = 0b00000111
        self._master.mav.set_attitude_target_send(
            time_boot_ms, self._target_system, self._target_component, mask,
            q, 0, 0, 0, collective)

    def cmd_attitude_rate(self, yaw_rate, pitch_rate, roll_rate, collective):
        time_boot_ms = 0  # TODO: figure out if this needs to be sent properly
        q = [0, 0, 0, 0]
        mask = 0b10000000
        self._master.mav.set_attitude_target_send(
            time_boot_ms, target_system, target_component, mask,
            q, roll_rate, pitch_rate, yaw_rate, collective)        

    def cmd_velocity(self, vn, ve, vd, heading):
        time_boot_ms = 0  # TODO: figure out if this needs to be sent properly
        mask = 0b0000010111000111
        self._master.mav.set_position_target_local_ned_send(
            time_boot_ms, self._target_system, self._target_component,
            mavlink.mavutil.MAV_FRAME_LOCAL_NED, mask,
            0, 0, 0,
            vn, ve, vd,
            0, 0, 0,
            yaw, 0)

    def cmd_motors(self, motor1, motor2, motor3, motor4):
        pass

    def cmd_position(self, n, e, d, heading):
        time_boot_ms = 0  # TODO: figure out if this needs to be sent properly
        mask = 0b0000010111111000
        self._master.mav.set_position_target_local_ned_send(
            time_boot_ms, self._target_system, self._target_component,
            mavlink.mavutil.MAV_FRAME_LOCAL_NED, mask,
            n, e, d,
            0, 0, 0,
            0, 0, 0,
            yaw, 0)

    def takeoff(self, n, e, d, yaw):
        # for mavlink to PX4 need to specify the NED location for landing
        # since connection doesn't keep track of this info, have drone send it
        # abstract away that part in the drone class
        time_boot_ms = 0  # TODO: figure out if this needs to be sent properly
        mask = 0b0000010111111000  # TODO: figure out the mask elements for a land command
        self._master.mav.set_position_target_local_ned_send(
            time_boot_ms, self._target_system, self._target_component,
            mavlink.mavutil.MAV_FRAME_LOCAL_NED, mask,
            n, e, d,
            0, 0, 0,
            0, 0, 0,
            yaw, 0)

    def land(self, n, e, d, yaw):
        # for mavlink to PX4 need to specify the NED location for landing
        # since connection doesn't keep track of this info, have drone send it
        # abstract away that part in the drone class
        time_boot_ms = 0  # TODO: figure out if this needs to be sent properly
        mask = 0b0000010111111000  # TODO: figure out the mask elements for a land command
        self._master.mav.set_position_target_local_ned_send(
            time_boot_ms, self._target_system, self._target_component,
            mavlink.mavutil.MAV_FRAME_LOCAL_NED, mask,
            n, e, d,
            0, 0, 0,
            0, 0, 0,
            yaw, 0)

    def set_home_position(self, lat, lon, alt):
        self.send_long_command(mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, lat, lon, alt)
