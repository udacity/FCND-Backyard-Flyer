import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True

        # initial state
        self.flight_state = States.MANUAL

        # callbacks registration
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def prepare_flight_plan(self, alt):
        self.all_waypoints.append(np.array([15, 15, alt]))
        self.all_waypoints.append(np.array([30, 0, alt]))
        self.all_waypoints.append(np.array([15, -15, alt]))
        self.all_waypoints.append(np.array([0, 0, alt]))

    def get_current_position(self):
        north, east, down = self.local_position
        current_position = np.array([north, east, -down])
        return current_position

    def local_position_callback(self):
        """
        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        flight_plan_is_empty = len(self.all_waypoints) == 0
        current_position = self.get_current_position()

        if self.flight_state == States.TAKEOFF:
            _, _, target_alt = self.target_position
            current_alt = current_position[2]

            if flight_plan_is_empty:
                self.prepare_flight_plan(target_alt)

            drone_reached_altitude = current_alt > target_alt * .95
            if drone_reached_altitude:
                self.waypoint_transition()

        elif self.flight_state == States.WAYPOINT:
            if flight_plan_is_empty:
                self.landing_transition()
            else:
                current_destination = self.all_waypoints[0]
                distance_left = np.linalg.norm(current_destination - current_position)

                target_destination_reached = distance_left < .5
                if target_destination_reached:
                    self.all_waypoints.pop(0)
                    self.waypoint_transition()

    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        if self.flight_state == States.LANDING:
            _, _, alt = self.global_position
            if alt < .2:
                self.disarming_transition()

    def state_callback(self):
        """
        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if self.flight_state == States.MANUAL and self.in_mission == True and self.guided == True:
            self.arming_transition()
            self.takeoff_transition()

    def arming_transition(self):
        """
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control()
        self.arm()
        long, lat, alt = self.global_position
        self.set_home_position(long, lat, alt)
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """       
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        target_alt = 3.0
        self.target_position[2] = target_alt
        self.takeoff(target_alt)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")
        n, e, alt = self.all_waypoints[0]
        self.cmd_position(n, e, alt, 0)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
