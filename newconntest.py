from connection import mavlink_connection
from connection import message_types as mt
import time


class TestClass:

    def __init__(self):
        #device = "" #"tcp:127.0.0.1:5760"
        device = "udp:127.0.0.1:14540"
        self.mavconn = mavlink_connection.MavlinkConnection(device, threaded=True)

        # connection state
        self._connection_alive = True

        # the list of attribute listeners
        self._attribute_listeners = {}

        # state information
        self._armed = False
        self._offboard = False

        # GPS information
        self._lat = 0
        self._lon = 0
        self._alt = 0

        @self.mavconn.on_message('*')
        def connection_listener(_, name, msg):

            if name == mt.MSG_GLOBAL_POSITION:
                self._lat = msg.latitude
                self._lon = msg.longitude
                self._alt = msg.altitude
                self.notify_attribute_listeners('gps', self.gps_position)

            elif name == mt.MSG_CONNECTION_CLOSED:
                self._connection_alive = False

            elif name == 'test':
                self.notify_attribute_listeners('test', 42)

            elif name == mt.MSG_STATE:
                self._armed = msg.armed
                self._offboard = msg.guided
                self.notify_attribute_listeners('state', self.state)

            # TODO: add handling for all other messages here

        # these neeed to be defined within the init method to work properly
        @self.on_attribute('*')
        def attribute_listener_test(self, name, data):
            ''' dummy listener that is registered for all attribute changes '''
            #print("attribute listener triggered")
            if name == 'state':
                #print("state update")
                i = 0

            # TODO: need to make more specific attribute listeners with specific capabilities

        @self.on_attribute('gps')
        def gps_listener_test(self, name, data):            
            # need to be constantly sending commands for PX4 to accept offboard control
            # send a position
            if self.state[0] is False or self.state[1] is False:
                self.mavconn.cmd_position(0, 0, 0, 0)
            else:
                self.mavconn.takeoff(0, 0, -3)
                print("takeoff requested")

        @self.on_attribute('state')
        def state_listener_test(self, name, data):
            if self.state[1] is False:
                self.mavconn.take_control()
                print("requesting offboard")
            elif self.state[0] is False:
                self.mavconn.arm()
                print("arming")


    def on_attribute(self, name):
        """
        decorator for being able to add a listener for a specific attribute
        """

        def decorator(fn):
            if isinstance(name, list):
                for n in name:
                    self.add_attribute_listener(n, fn)
            else:
                self.add_attribute_listener(name, fn)


        return decorator

    def add_attribute_listener(self, name, fn):

        name = str(name)
        if name not in self._attribute_listeners:
            self._attribute_listeners[name] = []
        if fn not in self._attribute_listeners[name]:
            self._attribute_listeners[name].append(fn)

    def remote_attribute_listener(self, name, fn):

        name = str(name)
        if name in self._attribute_listeners:
            if fn in self._attribute_listeners[name]:
                self._attribute_listeners[name].remove(fn)
                if len(self._attribute_listeners[name]) == 0:
                    del self._attribute_listeners[name]

    def notify_attribute_listeners(self, name, data):

        for fn in self._attribute_listeners.get(name, []):
            try:
                fn(self, name, data)
            except Exception as e:
                print("[ERROR] handling attribute listener")

        for fn in self._attribute_listeners.get('*', []):
            try:
                fn(self, name, data)
            except Exception as e:
                print("[ERROR] handling attribute listener")

    def start(self):

        # start running the connection thread in the background
        self.mavconn.start()

        # start this control loop
        self.run_loop()

        # TODO: need to make sure that this order works, not sure if need swap the order and start this run loop first

    def run_loop(self):
        ''' function that effectively is a 5Hz loop '''
        desired_rate = 1/5.0  # NOTE: PX4 needs at least 2(?) Hz

        takeoff = False

        prev_time = 0
        while self._connection_alive:
            # rate limit the loop to a specific rate
            current_time = time.time()
            if current_time - prev_time < desired_rate:
                continue

            # update the time
            prev_time = current_time

            '''
            if self.state[1] is False:
                self.mavconn.take_control()  # request offboard control of the vehicle
                print("requesting offboard control")

            elif self.state[0] is False:
                # TODO: probably a better way of sending an arm command
                # especially since this loop is running 5x faster than the state info is updated
                self.mavconn.arm()  # this would ideally be done by simply calling self.arm(), just not needed for testing connection class atm
                print("sending arm command")

            elif not takeoff:
                self.mavconn.takeoff(0, 0, 0, -3)
                print("sending takeoff command")
                takeoff = True
            '''


            # TODO: add sending of messages through the connection
            

    @property
    def gps_position(self):
        return [self._lat, self._lon, self._alt]

    @property
    def state(self):
        return [self._armed, self._offboard]

    


test = TestClass()
test.start()