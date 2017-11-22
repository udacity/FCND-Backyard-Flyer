# Flying Car - Backyard Flyer Project
This project is intended to walk you through the first steps of autonomously flying a drone. You will be using flying a quadcopter in simulation developed in Unity. After completing this assigment, you will be fimiliar with passing commands and receiving incoming data from the drone. You will also set up a state machine using event-driven programming.

The python code you write is similar to how the drone would be controlled from a ground station computer or an onboard companion computer. Since communication with the drone is done using MAVLink, you will be able to use your code to control an PX4 quadcopter autopilot with very little modification!

## Task
The required task is to command the drone to fly a 10m box at a 3m altitude. This box will flown in two ways: manual control and autonomous control.

Manual control of the drone is done using the instructions found with the simulator.

Autonomous control will be done using an event-driven state machine. First, you will need to fill in the appropriate callbacks. Each callback will check against transition criteria dependent on the current state. If the transition criteria are met, it will transition to the next state and pass along any required commands to the drone.

Telemetry data from the drone is logged for review after the flight. You will use the logs to plot the trajectory of the drone and analyze the performance of the task. For more information check out the Flight Log section below...

## Getting Started

You'll need Python 3. Anaconda now comes standard with Python 3 and several pre-installed packages. In addition to the standard Anaconda packages, this project will also require the following:

* pymavlink (Python implementation of MAVLink commands)

* utm (For conversion between Local and Global coordinate frames)

* lxml

* future

The instructions below walk through the steps for getting a Python environment set up with the appropriate dependencies.


Make sure you have [Anaconda](https://www.anaconda.com/download/) installed, this is required to create the project environment.

Creating the environment:

```sh
conda env create -f environment.yml
```

If the environment was installed succesfully you should output similar to the following:

```sh
Installing collected packages: future, lxml, pymavlink, utm
Successfully installed future-0.16.0 lxml-4.0.0 pymavlink-2.2.4 utm-0.4.2
#
# To activate this environment, use:
# > source activate backyard-flyer
#
# To deactivate an active environment, use:
# > source deactivate
#
```

You can view the environments you have installed with:

```sh
$ conda env list

# Sample output
# conda environments:
#
backyard-flyer           /usr/local/anaconda3/envs/backyard-flyer
root                  *  /usr/local/anaconda3
```

In order to use the environment you must activate it. Activating the environment:

```sh
source activate backyard-flyer
```

Deactivating the environment:

```sh
source deactivate
```

Once you've installed the environment you can cleanup unused packages and tarballs:

```sh
conda clean -tpy
```

Removing the environment:

```sh
conda env remove -n backyard-flyer
```

## Drone Simulator

The next step is to download the simulator build that's appropriate for your operating system. Here are the links for [Linux](https://github.com/udacity/FlyingCarND-Sim "Linux"), [Mac](https://github.com/udacity/FlyingCarND-Sim "Mac"), or [Windows](https://github.com/udacity/FlyingCarND-Sim "Windows").

You can manually fly the drone using the instructions provided in the simulator's readme.

## Drone API

A wrapper `Drone` superclass was written to handle all the communication between Python and the drone simulator. The `Drone` class contains commands to be passed to the simulator and allows students to register callbacks/listeners on messages coming from the simulator. The goal of this project is to design a subclass from the Drone class implementing a state machine to autonomously fly a box. A subclass is started for you in `backyard_flyer.py`

### Incoming Message Types

The following incoming message types are available for the Backyard Flyer Project:

* state_msg: Information about whether the vehicle is armed and in guided mode
* global_position_msg: latitude, longitude, altitude
* local_position_msg: local north, local east, local down
* local_velocity_msg: local north velocity, local east velocity, local vertical velocity (positive up)

All message types also contain the time. More information about the properties of each message can be found in `message_types.py`. The data for these messages are retrieved using callbacks.

### Registering Callbacks

The incoming message data is receiving using callbacks. These methods are only called when a message of their respective type is received. There are two ways to register a callback:

1. Using the `@msg_callback(msg_type)` decorator (preferred):

Callbacks registered using decorators need to be defined (and decorated) within the 'callback' method. The callbacks method is called on initialization to register all the defined callbacks.

~~~
def callbacks(self):
	""" Define your callbacks within here"""
	super().callbacks()
	
    @self.msg_callback(message_types.MSG_GLOBAL_POSITION)
	def global_position_listener(name, global_position):
		# do whatever with the global_position, which will be of type GlobalPosition
~~~



2. Registering the callback for the respective message: 

~~~
self.add_message_listener(message_types.MSG_GLOBAL_POSITION, self.global_position_listener)

def global_position_listener(self, name, global_position):
	# do whatever you want with the global_position, which will be of type GlobalPosition
~~~

A callback for all message types can be registered uisng, '*':

~~~
@self.msg_callback('*')
def all_msg_listener(name, msg):
	# this is a listener for all message types, so break out the msg as defined by the name
~~~

or

~~~
self.add_message_listener('*',self.all_msg_listener)
def all_msg_listener(self,name, msg):
	# this is a listener for all message types, so break out the msg as defined by the name
~~~
        

### Vehicle Attributes

Besides being passed to appropriate callbacks, the message data is also saved into the following attributes of the Drone class:

* global_position: latitude (deg), longitude (deg), altitude (m)
* local_position: north (m), east (m), down (m)
* local_velocity: north velocity (m/s), east velocity (m/s), vertical velocity (m/s, positive down)
* armed: True/False
* guided: True/False

Vehicle attribute can be used if information is required from multiple messages. For example:

~~~
@self.msg_callback(message_types.MSG_GLOBAL_POSITION)
	def global_position_listener(name, global_position):
		if msg.global_position[2] < 0.05: #Checks the global altitude
        	if self.local_velocity[2] < 0.05 #Checks the latest drone velocity, since it isn't part of the message
~~~


### Outgoing Commands

The following commands are implemented for the Backyard Flyer Project:

* connect(): Starts receiving messages from the drone. Blocks the code until the first message is received
* start(): Start receiving messages from the drone. If the connection is not threaded, this will block the code.
* arm(): Arms the motors of the quad, the rotors will spin slowly. The drone cannot takeoff until armed first
* disarm(): Disarms the motors of the quad. The quadcopter cannot be disarmed in the air
* take_control(): set the command mode of the quad to guided
* release_control(): set the command mode of the quad to manual
* cmd_position(north, east, down, heading): command the vehicle to travel to the local position (north, east, down). Also commands the quad to maintain a specified heading
* takeoff(target_altitude): takeoff from the current location to the specified global altitude
* land(): land in the current position
* stop(): terminate the connection with the drone and close the telemetry log

These can be called directly from other methods within the drone class:

~~~
self.arm() #seconds an arm command to the drone
~~~


### Manual Flight

To log data while flying manually, run the `drone.py` script as shown below:

~~~
python drone.py
~~~

Run this script after starting the simulator. It connects to the simulator using the Drone class and runs until tcp connection is broken. The connection will timeout if it doesn't receive a heartbeat message once every 10 seconds. The GPS data is automatically logged.

To stop logging data, stop the simulator first and the script will automatically terminate after approximately 10 seconds.

Alternatively, the drone can be manually started/stopped from a python/ipython shell:

~~~
from drone import Drone
drone = Drone()
drone.start(threaded=True)
~~~

If `threaded` is set to `False`, the code will block and the drone logging can only be stopped by terminating the simulation. If the connection is threaded, the drone can be commanded using the commands described above, and the connection can be stopped (and the log properly closed) using:

~~~
drone.stop()
~~~

### Message Logging

The telemetry data is automatically logged in "Logs\TLog.txt". Each row contains a comma seperated representation of each message. The first row is the incoming message type. The second row is the time. The rest of the rows contains all the message properties. The types of messages relevant to this project are:

* state_msg: time (ms), armed (bool), guided (bool)
* global_position_msg: time (ms), longitude (deg), latitude (deg), altitude (m)
* global_home_msg: time (ms), longitude (deg), latitude (deg), altitude (m)
* local_position_msg: time (ms), north (m), east (m), down (m)
* local_velocity_msg: time (ms), north (m), east (m), down (m) 


#### Reading Telemetry Logs

Logs can be read using:

~~~
t_log = Drone.read_telemetry_data(filename)
~~~

The data is stored as a dictionary of message types. For each message type, there is a list of numpy arrays. For example, to access the longitude and latitude from a global_position_msg:

~~~
#Time is always the first entry in the list
time = t_log['global_position_msg'][0][:]
longitude = t_log['global_position_msg'][1][:]
latitude = t_log['global_position_msg'][2][:]
~~~

The data between different messages will not be time synced since they are recorded at different times.


## Autonomous Control State Machine

After getting familiar with how the drone flies, you will fill in the missing pieces of a state machine to fly the drone autonomously. The state machine is run continuously until either the mission is ended or the Mavlink connection is lost.

The six states predefined for the state machine:
* MANUAL: the vehicle is being controlled by the user
* ARMING: the vehicle is in guided mode and being armed
* TAKEOFF: the vehicle is taking off from the ground
* WAYPOINT: the vehicle is flying to a specified target position
* LANDING: the vehicle is landing on the ground
* DISARMING: the vehicle is disarming

While the drone is in each state, you will need to check transition criteria with a registered callback. If the transition criteria are met, you will set the next state and pass along any commands to the drone. For example:

~~~
@self.on_message(mt.MSG_STATE)
def state_callback(msg_name, msg):
	if self.state == States.DISARMING:
    	if ~msg.armed:
        	self.release_control()
        	self.in_mission = False
        	self.state = States.MANUAL
~~~
This is a callback on the state message. It only checks anything if it's in the DISARMING state. If it detects that the vehicle is successfully disarmed, it sets the mode back to manual and terminates the mission.       




### Running the State Machine
After filling in the appropriate callbacks, you will run the mission:

~~~
python backyard_flyer.py
~~~

Similar to the manual flight, the GPS data is automatically logged to the specified log file.



### Reference Frames

Two different reference frames are used. Global positions are defined [longitude, latitude, altitude (pos up)]. Local reference frames are defined [North, East, Down (pos down)] and is relative to a nearby global home provided. Both reference frames are defined in a proper right-handed reference frame . The global reference frame is what is provided by the Drone's GPS, but degrees are difficult to work with on a small scale. Conversion to a local frame allows for easy calculation of m level distances. Two convenience function are provided to convert between the two frames. These functions are wrappers on `utm` library functions.

~~~
#Convert a local position (north,east,down) relative to the home position to a global position (lon,lat,up)
def local_to_global(local_position, global_home):

#Convert a global position (lon,lat,up) to a local position (north,east,down) relative to the home position
def global_to_local(global_position, global_home):
~~~



## Submission Requirements

* Filled in backyard_flyer.py

* An x-y (East-North or Long-Lat) plot of the vehicle trajectory while manually flying the box

* An x-y (East-North or Long-Lat) plot of the vehicle trajectory from autonomously flying the box

* A short write-up (.md or .pdf)

## Project Walkthrough

TODO: Film a YouTube step through of the project

## Modifications for PX4

TODO: This would be nice to have, but isn't a top priority


