"""example for event driven programming - not OOP (no states)

@author: Adrien Perkins
"""

# imports
from drone import Drone
from connection import message_types as mt
import time
import numpy as np


# variable needed to be able to share information between callbacks
ready_to_fly = False
taken_off = False
target_position = np.array([0.0, 0.0, 0.0])


# create an instance of our drone
# 
# NOTE: the instance is created before defining the functions as the callbacks
# need to be able to access the drone object to be able to send commands
drone = Drone(threaded=False)



def state_callback(msg_name, msg):
    """callback for updates to the drone state information
    
    handles actions to take place when the drone's state information is updated.

    student code should:
        1. take control of the drone if the drone is not in guided mode
        2. arm the drone if the drone is not armed (enables takeoff)
    
    Args:
        msg_name: the name of the msg (in this case will be mt.MSG_STATE)
        msg: the StateMessage object containing state message information
    """

    # TODO: for a first example could very easily remove all these elements and
    # provide them with a drone that is already armed and ready to fly

    # need some globals in order for this method to work
    global ready_to_fly

    print("state information gotten -> {}, {}".format(msg.guided, msg.armed))

    # 1. take control of the drone
    if not msg.guided:
        print("taking control")
        drone.take_control()

    # 2. need to then arm the drone
    elif not msg.armed:
        print("arming")
        drone.arm()

    # 3. finally ready to fly
    else:
        print("ready to fly")
        ready_to_fly = True


def local_pos_callback(msg_name, msg):
    """callback for updates to the drone's local position
    
    handles actions to take place when the drone's location position is updated.

    student code should:
        1. once ready to take off, command the drone to takeoff
        2. once at the desired altitude, command the drone to fly to a point
    
    Args:
        msg_name: the name of the msg (in this case will be mt.MSG_LOCAL_POSITION)
        msg: the LocalFrameMessage object containing the local position information
    """

    # NOTE: if we remove the armed/control parts, can remove this statement as
    # the drone will already be ready to fly
    
    # need some globals for this to work
    global ready_to_fly, taken_off, target_position

    if not ready_to_fly:
        return

    # before we can fly different points, need to takeoff first
    if not taken_off:
        # check to see if we are at the desired altitude
        if (-1.0 * msg.down) > (0.95 * target_position[2]):
            # we are in the air!
            print("takeoff completed!")

            # update this flag
            taken_off = True
        else:
            # takeoff
            target_altitude = 3
            target_position[2] = target_altitude
            drone.takeoff(target_altitude)

    # now we are in the air and can fly to a given point
    else:
        print("commanding waypoints")
        dn = (msg.north - target_position[0])
        de = (msg.east - target_position[1])
        dd = (msg.down - target_position[2])
        distance_to_target = np.sqrt(dn * dn + de * de + dd * dd)
        if distance_to_target < 0.5:
            # TODO: go to next target
            # drone.cmd_position()
            pass

    # NOTE: this example gets pretty crazy pretty quickly as you want to check 
    # differnt states and do different things (e.g. move to a position after 
    # having taken off will require even more complicated checks - this is 
    # where the idea of the state machine comes into play)


# add our callbacks to the drone
print("registering callbacks...")
drone.add_message_listener(mt.MSG_STATE, state_callback)
drone.add_message_listener(mt.MSG_LOCAL_POSITION, local_pos_callback)

# start running our drone.
# this effectively waits for the incoming messages to trigger the callbacks
# that we have defined above.  For each state, when the transition condition
# to go from one state to another is met, it will command the drone
# accordingly and transition the states, all as was defined in the functions
# above.
# this is out dispatch loop
print("starting loop....")
drone.start()
