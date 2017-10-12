from abc import ABCMeta, abstractmethod


class Connection(metaclass=ABCMeta):

    def __init__(self):
        # every connection type will have a set of listeners
        # this is a dictionary, keyed by listener type
        self._message_listeners = {}

    def on_message(self, name):
        """
        decorator for being able to add a listener for a specific message type

        e.g. on how you would use in in the drone class, where you have created
        a variable 'conn = Connection()'

        @conn.on_message('attitude')
        def att_listener(_, name, att):
            # do whatever with the att, which will be of type attitude
        """

        def decorator(fn):
            if isinstance(name, list):
                for n in name:
                    self.add_message_listener(n, fn)
            else:
                self.add_message_listener(name, fn)


        return decorator

    def add_message_listener(self, name, fn):

        name = str(name)
        if name not in self._message_listeners:
            self._message_listeners[name] = []
        if fn not in self._message_listeners[name]:
            self._message_listeners[name].append(fn)

    def remove_message_listener(self, name, fn):

        name = str(name)
        if name in self._message_listeners:
            if fn in self._message_listeners[name]:
                self._message_listeners[name].remove(fn)
                if len(self._message_listeners[name]) == 0:
                    del self._message_listeners[name]

    def notify_message_listeners(self, name, msg):

        for fn in self._message_listeners.get(name, []):
            try:
                fn(self, name, msg)
            except Exception as e:
                errprinter('>>> Exception in message handler for %s' %
                           msg.get_type())
                errprinter('>>> ' + str(e))

        for fn in self._message_listeners.get('*', []):
            try:
                fn(self, name, msg)
            except Exception as e:
                errprinter('>>> Exception in message handler for %s' %
                           msg.get_type())
                errprinter('>>> ' + str(e))

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def arm(self):
        pass

    @abstractmethod
    def disarm(self):
        pass

    @abstractmethod
    def take_control(self):
        pass

    @abstractmethod
    def release_control(self):
        pass

    @abstractmethod
    def cmd_attitude(self, yaw, pitch, roll, collective):
        pass

    @abstractmethod
    def cmd_attitude_rate(self, yaw_rate, pitch_rate, roll_rate, collective):
        pass

    @abstractmethod
    def cmd_velocity(self, vn, ve, vd, heading):
        pass

    @abstractmethod
    def cmd_motors(self, motor1, motor2, motor3, motor4):
        pass

    @abstractmethod
    def cmd_position(self, n, e, d, heading):
        pass

    @abstractmethod
    def takeoff(self, alt):
        pass

    @abstractmethod
    def land(self):
        pass

    @abstractmethod
    def set_home_position(self, lat, lon, alt):
        pass
