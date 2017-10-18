from connection import mavlink_connection

#device = "" #"tcp:127.0.0.1:5760"
device = "udp:127.0.0.1:14540"
mavconn = mavlink_connection.MavlinkConnection(device, threaded=False)


@mavconn.on_message('*')
def testing(_, name, msg):
    print("got a message")
    print(name)
    print(msg)


mavconn.testcallback()
mavconn.start()