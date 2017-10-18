from connection import mavlink_connection


class TestClass:

	def __init__(self):
		#device = "" #"tcp:127.0.0.1:5760"
		device = "udp:127.0.0.1:14540"
		self.mavconn = mavlink_connection.MavlinkConnection(device, threaded=False)


		@self.mavconn.on_message('*')
		def testing(_, name, msg):
		    print("got a message")
		    print(name)
		    print(msg)

	def start(self):
		self.mavconn.testcallback()
		self.mavconn.start()

test = TestClass()
test.start()