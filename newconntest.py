from connection import mavlink_connection

device = "tcp:127.0.0.1:5760"
mavconn = mavlink_connection.MavlinkConnection(device)


@mavconn.on_message('*')
def testing(_, name, msg):
    print("got a message")
    print(name)
    print(msg)


mavconn.testcallback()