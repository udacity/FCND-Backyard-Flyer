# -*- coding: utf-8 -*-

from pymavlink import mavutil
import os
# force use of mavlink v2.0
os.environ['MAVLINK20'] = '1'

class Connection:
    def __init__(self, device):
        self.master = mavutil.mavlink_connection(device, source_system=190)
        self._running = True
        #self.read_handle = threading.Thread(
        #    target=self.read_thread)
        #self.read_handle.start()
        self.write_handle = 0

        self.target_system = 0
        self.target_component = 0
        self.callbacks = {}
    
    def set_callback(self,msg,fn):
        self.callbacks[msg] = fn

    def wait_for_message(self):
        msg = self.master.recv_match(blocking=True,timeout=10)
        if msg == None:
            return True
        else:
            if(msg.get_type() != 'BAD_DATA'):
                if msg.get_type() in self.callbacks.keys():
                    self.callbacks[msg.get_type()](msg)
        
            if msg.get_type() == 'HEARTBEAT':
                # send -> type, autopilot, base mode, custom mode, system status
                self.master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                           mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                           0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
            return False


    def send_mav_command(self, command_type, param1, param2, param3, param4, x,
                         y, z):
        self.master.mav.command_int_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, command_type, 1, 0, float(param1),
            float(param2), float(param3), float(param4), int(x), int(y), float(z))

    def disconnect(self):
        # stop running the read loop, and wait for the thread to finish
        self._running = False
        #self.read_handle.join()

        # close the mavutil connection (tcp/udp/serial, etc)
        self.master.close()

