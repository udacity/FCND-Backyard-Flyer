# Flying Car - Backyard Flyer Project
This project is intended to walk you through the first steps of taking control autonomously flying a drone. You will be using a simulation of a quadcopter developed in Unity. After completing this assigment, you will be familiar communicating with the drone via Mavlink and analyzing flight log files. The python code you write is similar to how the drone would be controlled from a ground station computer or an onboard companion computer. Since communication with the drone is done using Mavlink, you will be able to use your code to control an Arudpilot quadcopter with very little modifications!

## Task
The required task is to command the drone to fly a 10m box at a 3m altitude. This box will flown in two ways: manual control and autonomous control.

Manual control of the drone is done using the instructions found with the simulator.

Autonomously controlling will be done using a event-driven state machine. First, you will need to fill in the appropriate state method which is run on the intiation of each state. Next you'll write the state transition callbacks. In these callbacks, you will check need to check an appropriate transition criterion to transition to the next state.

Telemetry data from the drone is logged for review after the flight. You will use the logs to plot the trajectory of the drone and analyze the performance of the task. For more information check out the Flight Log section below...

## Getting Started

You'll need Python 3. Anaconda now comes standard with Python 3 and several pre-installed packages. In addition to the standard Anaconda packages, this project will also required:

* pymavlink (Python implementation of Mavlink commands)

* utm (For conversion between Local and Global coordinate frames)

* lxml

* future

The instructions below walk through the instructions for getting a Python environment set up with the appropriate dependencies.


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
# > source activate flyingcarnd-p0
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
flyingcarnd-p0           /usr/local/anaconda3/envs/flyingcarnd-p0
root                  *  /usr/local/anaconda3
```

In order to use the environment you must activate it. Activating the environment:

```sh
source activate flyingcarnd-p0
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
conda env remove -n flyingcarnd-p0
```

## Drone Simulator

The next step is to download the simulator build that's appropriate for your operating system. Here are the links for [Linux](https://github.com/udacity/FlyingCarND-Sim "Linux"), [Mac](https://github.com/udacity/FlyingCarND-Sim "Mac"), or [Windows](https://github.com/udacity/FlyingCarND-Sim "Windows").

You can manually fly the drone using the instructions provided in the simulator's readme.

## Logging

The telemetry data received from the drone is logged in a csv format into the Logs directory using logger.py. The default log name is navLog.txt. The log name can be changed in drone.py. If a log file already exists with the same name, the previous log will be overwritten.

The GPS data is set to automatically log whenever the Drone class receives a GPS message. The default format for the logging is:


* Column 1: Longitude (degrees)
* Column 2: Latitude (degrees)
* Column 3: Altitude, positive up (meters)
* Column 4: North Velocity (m/s)
* Column 5: East Velocity (m/s)
* Column 6: Down Velocity, positive down (m/s)
* Column 7: Heading
* Column 8: self.target_position[0] (user assigned)
* Column 9: self.target_position[1] (user assigned)
* Column 10: self.target_position[2] (user assigned)


### Manual Flight Logging

To log GPS data while flying manually, run the drone.py script as shown below:

~~~
conda drone.py
~~~

Run this script after starting the simulator. It connects to the simulator using the Drone class and runs until tcp connection is broken. The connection will timeout if it doesn't receive a heartbeat message once every 10 seconds. The GPS data is automatically logged.

To stop logging data, stop the simulator first and the script will automatically terminate after 10 seconds.

### Reading Logs

Logs can be read using:

~~~
import logger
nav_log = logger.read_log(filename)
~~~

Filename should be replaced with the appropriate path to the log. The data from the log is returned as a numpy 2D array. To plot a specific value, you can use the pyplot from from the matplotlib package:

~~~
from matplotlib import pyplot
pyplot.plot(nav_log[:,0])
~~~


## Autonomous Control State Machine

After getting familiar with how the drone flies, you will fill in the missing pieces of a state machine to fly the drone autonomously. The state machine is run continuously until either the mission is ended or the Mavlink connection is lost.

The six states predefined for the state machine:
* MANUAL: the vehicle is being controlled by the user
* ARMING: the vehicle is in guided mode and being armed
* TAKEOFF: the vehicle is taking off from the ground
* WAYPOINT: the vehicle is flying to a specified target position
* LANDING: the vehicle is landing on the ground
* DISARMING: the vehicle is disarming

The state machine methods are seperated into two different types: state methods and state callbacks. The state methods are meant to be run only when transitioning to their respective state. The state callbacks are checked while the drone is in the respective state every time a new Mavlink message is received.
        

### State Methods
The state methods required to be filled in are:

~~~

	#Initiate the ARMING state, put the vehicle in guided mode and arm
    def arm(self):
        #TODO: fill out this method
        return
    
    #Initiate the TAKEOFF state, command the vehicle the target altitude (m)
    def takeoff(self,altitude=3.0):
        #TODO: fill out this method
        return
    
    #Initiate the WAYPOINT state, command the vehicle to the position specified as Lat, Long, Alt
    def waypoint(self,target):
        #TODO: fill out this method
        return
    
    #Initiate the LANDING state, command the vehicle to specified altitude (m)
    def land(self, altitude=0.0):
        #TODO: fill out this method
        return
    
    #Initiate the DIARMING state, command the vehicle to disarm
    def disarm(self):
        #TODO; fill out this method
        return        
~~~

Each of these sets the current state and sends one (or more) Mavlink commands to the drone (a description of Mavlink commands if found below). These methods are meant to be run once and only on transition to the respective state. An example of this when transitioning to the MANUAL state is provided:

~~~
	def manual(self):
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE, 0, 0,
                             0, 0, 0, 0, 0)
        self.state = States.MANUAL
        return        
~~~

The method sends a the command 'MAV_CMD_NAV_GUIDED_ENABLE'. The first parameter is set to '0', commanding the drone to disable 'GUIDED' mode. It also set the state to MANUAL, so the appropriate callback is checked.

#### State Callbacks
The next step is to fill in the appropriate callback. These functions are called every time a new Mavlink message is received from the drone while in its respective state. You will need to fill in the following callbacks:

~~~

	# Save the current position as the home position and transition to the next state when the vehicle is armed and in guided mode
    def arming_callback(self):
        #TODO: fill out this callback
        return
        
    
    # Transition to the next state when the target altitude is reach
    def takeoff_callback(self):
        # TODO: fill out this callback
        return
    
        
    # Transition to the next state when the target waypoint is reached (within 1m)
    def waypoint_callback(self):
        #TODO: fill out this callback
        return                
       

    # Transition to the next state when the drone is on the ground
    def landing_callback(self):
        #TODO: fill out this callback
        return
    
    # Transition to the next state when the drone is disarmed
    def disarming_callback(self):
        #TODO: fill in this callback
        return
~~~

For convenience the Mavlink messages are parsed and sorted in the decode_mav_msg callback into different variables of the drone class. The following class variables hold the latest drone information receive from Mavlink:


* global_position: numpy array [Longitude (deg), Latitude (deg), Altitude (m)]
* motors_armed: bool
* global_velocity: numpy array [North Velocity (m/s), East Velocity (m/s), Up Velocity (m/s)]
* heading: float (Vehicle heading (deg))
* guided: bool (True: Guided Mode, False: Manual mode)
* connected: bool

An example of the 'disarming' callback would be:

~~~

	# Transition to the next state when the drone is disarmed
    def disarming_callback(self):
        if ~self.armed:
            self.manual(0)
        return
~~~
This callback checks to see if the drone is armed. When it detects that the drone is no longer armed, it transitions to the MANUAL state.

### Running the State Machine
After filling in the state methods and callbacks, you will run the mission:

~~~
conda drone.py -r
~~~

Similar to the manual flight, the GPS data is automatically logged to the specified log file.


## Mavlink
Mavlink provides a common message protocol for communication with your drone. More information about difference message structure can be found [here](http://mavlink.org/messages/common "Mav Msg") 

The Connection class found in connection.py is a wrapper around the pymavlink library in order to make working with mavlink easier for you. Messages are sent, receive, and decoded with the Connection class. An instance of the Connection class is initialized in the Drone class __init__ method. The Connection class works on an different thread than the Drone class, so messages can continue to be sent and receiving even while blocking code.

Commands are sent using the 'send_mav_command' method of the Connection class. The first input is the type of command specified by the enum mavutil.mavlink. The other next 7 inputs are all parameters specific to a command (the last three always correspond to x, y, z if applicable).

Not all autopilots implement all commands in the same way. The simulator accepts a limited set of MAV_CMDs. For a list of MAV_CMDs implemented in the simulator, see below.

Telemetry data received into the Drone class in the 'decode_mav_msg' callback. This callbacks assigns the data to the appropriate variables of the Drone class depending on the message type. For this project, only limited telemetry is available from the Drone.


### MAV_CMDs

Only the following MAV_CMDs are implemented for communication with the drone with the 'send_mav_command':

* MAV_CMD_COMPONENT_ARM_DISARM (parameter1 = 1 armed, 0 disarmed)
* MAV_CMD_NAV_GUIDED_ENABLE (parameter1 = 1 enable, 0 disable)
* MAV_CMD_NAV_LOITER_UNLIM (Command the vehicle to the x, y, z position defined in the global frame (longitude, latitue, altitude)
* MAV_CMD_NAV_TAKEOFF (Ascend straight up to the altitude specified by the z parameter)
* MAV_CMD_NAV_LAND (Descend straight down to the altitude specified by the z parameter)

NOTE: Floating point (32 bit) numbers do not have the precision required for a GPS latitude/longitude. Latitude/Longitude are passed via Mavlink using Integer data types. Degrees latitude/longitude are scalled by 10^7 prior to conversion to integer. All other parameters passed using MAV_CMD are passed as floats.

Other MAV_CMDs passed to the simulator will be ignored

### MAV_MSGs

There are two types of Mavlink telemetry messages currently being sent from the simulator:

* HEARTBEAT (Contains status information about the drone)
* GLOBAL_POSITION_INT (longitude (deg*10^7), latitude (deg*10^7), altitude (mm), rel_alt (mm), north velocity (m/s*100), east velocity (m/s*100, down velocity (m/s*100), heading (deg*100))

NOTE: All fields in the GLOBAL_POSITION_INT are passed as integer types. The longitude, latitude, and altitudes are passed as integers (32 bit), the velocities are passed as shorts (16 bit) and the heading is passed as an unsigned short (16 bit). All values are scaled as shown.

### Reference Frames

Two different reference frames are used. Global positions are defined [longitude, latitude, altitude (pos up)]. Local reference frames are defined [North, East, Down (pos down)] and is relative to a nearby global home provided. Both reference frames are defined in a proper right-handed reference frame . The global reference frame is what is provided by the Drone's GPS, but degrees are difficult to work with on a small scale. Conversion to a local frame allows for easy calculation of m level distances. Two convenience function are provided to convert between the two frames. These functions are wrappers on utm library functions.

~~~
#Convert a local position (north,east,down) relative to the home position to a global position (lon,lat,up)
def local_to_global(local_position, global_home):

#Convert a global position (lon,lat,up) to a local position (north,east,down) relative to the home position
def global_to_local(global_position, global_home):
~~~



## Submission Requirements

* Filled in drone.py

* An x-y (East-North or Long-Lat) plot of the vehicle trajectory while manually flying the box

* An x-y (East-North or Long-Lat) plot of the vehicle trajectory from autonomously flying the box

* A short write-up (.md or .pdf)

## Project Walkthrough

TODO: Film a YouTube step through of the project

## Modifications for Ardupilot

TODO: This would be nice to have, but isn't a top priority


