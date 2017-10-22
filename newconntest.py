from connection import mavlink_connection
from connection import message_types as mt


class TestClass:

	def __init__(self):
		#device = "" #"tcp:127.0.0.1:5760"
		device = "udp:127.0.0.1:14540"
		self.mavconn = mavlink_connection.MavlinkConnection(device, threaded=True)

        # connection state
        self._connection_alive = True

        # the list of attribute listeners
        self._attribute_listeners = {}

        # GPS information
        self._lat = 0
        self._lon = 0
        self._alt = 0

        @self.mavconn.on_message('*')
        def testing(_, name, msg):
            print("got a message")
            print(name)
            print(msg)

            if name == mt.MSG_GLOBAL_POSITION:
                self._lat = msg.lat
                self._lon = msg.lon
                self._alt = msg.alt
                self.notify_attribute_listeners('gps', self.gps_position)

            elif name == mt.MSG_CONNECTION_CLOSED:
                self._connection_alive = False

            elif name == 'test':
                self.notify_attribute_listeners('test', 42)


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
                errprinter('>>> Exception in message handler for %s' %
                           name)
                errprinter('>>> ' + str(e))

        for fn in self._attribute_listeners.get('*', []):
            try:
                fn(self, name, data)
            except Exception as e:
                errprinter('>>> Exception in message handler for %s' %
                           name)
                errprinter('>>> ' + str(e))

	def start(self):
		
        # test a callback
        self.mavconn.testcallback()
		
        # start running the connection thread in the background
        self.mavconn.start()

        # start this control loop
        self.run_loop()

        # TODO: need to make sure that this order works, not sure if need swap the order and start this run loop first

    def run_loop(self):
        ''' function that effectively is a 50Hz loop '''

        i = 0
        while _connection_alive:
            i += 1

            # TODO: add some rate limiting here
            # TODO: add sending of messages through the connection
            

    @property
    def gps_position(self):
        return [self._lat, self._lon, self._alt]


    @on_attribute('*')
    def attribute_listener_test(self, name, data):
        ''' dummy listener that is registered for all attribute changes '''
        print("attribute listener triggered")
        print(name)


test = TestClass()
test.start()