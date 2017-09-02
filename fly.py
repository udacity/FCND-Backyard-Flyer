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
prev_data = None

@sio.on('heartbeat')
def heartbeat(sid, data):
    print("HEARTBEAT RECEIVED ...")
    print(data)
    # msg = mav.decode(bytes(data["mavmsg"], 'utf-8'))
    # print(msg)

@sio.on('telemetry')
def telemetry(sid, data):
    if data:
        print("Message received ... ", data)
        prev_data = data
        y = data["y"]

        if y > 5:
            sio.emit("command", data={"mavmsg": "land"}, skip_sid=True)
        elif y < 0.2:
            sio.emit("command", data={"mavmsg": "takeoff"}, skip_sid=True)

        # msg = mavlink.MAVLink_heartbeat_message(2, 0, 0, 0, 0, 3)
        # packed = msg.pack(mav)
        # chars = [c for c in packed]
        # print("Sending MAVLink message ...", chars)
        # sio.emit("command", data={'mavmsg': chars}, skip_sid=True)


@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)


if __name__ == '__main__':
    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)