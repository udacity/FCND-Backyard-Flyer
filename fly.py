import socketio
import eventlet
import eventlet.wsgi
from flask import Flask
import mavlink
import time
import sys

class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

sio = socketio.Server()
app = Flask(__name__)

f = fifo()
mav = mavlink.MAVLink(f)

@sio.on('telemetry')
def telemetry(sid, data):
    print("Message received ...")
    msg = mavlink.MAVLink_heartbeat_message(2, 0, 0, 0, 0, 3)
    packed = msg.pack(mav)
    chars = [c for c in packed]
    print(chars)
    sio.emit("command", data={'mavmsg': chars}, skip_sid=True)


@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)


if __name__ == '__main__':
    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)

    # msg = mavlink.MAVLink_heartbeat_message(2, 0, 0, 0, 0, 3)
    # packed = msg.pack(mav)
    # print("Packed message ...", packed)
    # print([c for c in packed])

    # msg = mavlink.MAVLink_system_time_message(100, 50)
    # packed = msg.pack(mav)
    # print("Packed message ...", packed)



