# Flying Car Project 0 - Backyard Flyer
This project is intended to walk you through the first steps of taking control autonomously flying a drone. You will be using a simulation of a quadcopter developed in Unity. After completing this assigment, you will be familiar communicating with the drone via Mavlink and analyzing flight log files. The python code you write is similar to how the drone would be controlled from a ground station computer or an onboard companion computer. Since communication with the drone is done using Mavlink, you will be able to use your code to control an Arudpilot quadcopter with very little modifications!

## Task
The required task is to command the drone to fly a 10m box at a 3m altitude. This box will flown in two ways: manual control and autonomous control.

Manual control of the drone is done using the instructions found with the simulator.

Autonomously controlling will be done using a set of command states. First, you will need to fill in the code for each of the command states. These command states pass along the appropriate Mavlink command to the drone and determine when the command has been executed. Next, using these command states, you will write a function to command the vehicle the fly the box.

Telemetry data from the drone is logged for review after the flight. You will use the logs to plot the trajectory of the drone and analyze the performance of the task. For more information check out the Flight Log section below...

## Getting Started

You'll need Python 3. Anaconda now comes standard with Python 3 and several pre-installed packages. In addition to the standard Anaconda packages, this project will also required:

* pymavlink (Python implementation of Mavlink commands)

* utm (For conversion between Local and Global coordinate frames)

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

The next step is to download the simulator build that's appropriate for your operating system. You can find the build on the [releases page](https://github.com/udacity/FlyingCarND-Sim/releases).

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


### Manual Flight Logging

To log GPS data while flying manually, run the manual_flight.py script. It connects to the simulator using the Drone class and runs until tcp connection is broken. After starting the simulator run the manual_flight.py:

~~~
conda manual_flight.py
~~~

The connection will output "heartbeat" once every second while it is still connected to the simulator (you make have to close the simulator completely for the connection to terminate).

### Reading Logs

Logs can be read using:

~~~
import logger
nav_log = logger.read_log(filename)
~~~

Filename should be replaced with the appropriate path to the log. The data from the log is returned as a numpy 2D array. To plot a specific value, you can use the pyplot from from the matplotlib package:

~~~
from matplotlib import pyplot
pyplot.nav_log[:,0]
~~~


## Autonomous Control

After getting familiar with how the drone flies, you will next write a python script to command the drone to fly a box. To do this, you first need to to fill in the command states of the Drone class.

### Drone Class

The Drone class is found in drone.py. Several of the methods are filled in, but you will need to fill in the methods with ("TODO: Fill in this method"). The methods required are:

~~~

	#Sets the mode to guided, arms the vehicle (with checks) and save the home position as the position it is armed
    def arm_vehicle(self):
        #TODO: fill out this method
        return True

    #Command the vehicle to a specific height and return True when it gets to the specified altitude
    def takeoff(self):
        # TODO: fill out this method
        return True

    # Command the vehicle to the target position and return True when it has arrived
    def goto(self, target):
        #TODO: fill out this method
        return True

    # Lands the vehicle in the current location and returns true when the vehicle is on the ground
    def land(self):
        #TODO: fill out this method
        return True
        
    #Disarms the vehicle, and returns true when the motors report armed
    def disarm_vehicle(self):
        #TODO: fill out this method   
        return True
        
~~~

Each of these command states requires one (or more) mavlink commands sent to the vehicle and appropriate state checks to see when the command has been properly executed. An example of this for the 'disarm' command would be

~~~

    def disarm_vehicle(self):
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                     0, 0, 0, 0, 0, 0)

        while self.motors_armed:
            time.sleep(1)
        
        #Give back control of the vehicle
        self.connection.send_mav_command(mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE, 0, 0,
                                     0, 0, 0, 0, 0)

        return True
        
~~~

The vehicle sends a the command 'MAV_CMD_COMPONENT_ARM_DISARM', waits until the motors are armed and then disables guided mode using the command 'MAV_CMD_NAV_GUIDED_ENABLED' (the first parameter set to 0 disables guided mode).


### Mavlink
Mavlink provides a common message protocol for communication with your drone. More information about difference message structure can be found [here](http://mavlink.org/messages/common "Mav Msg") 

The Connection class found in connection.py is a wrapper around the pymavlink library in order to make working with mavlink easier for you. Messages are sent, receive, and decoded with the Connection class. An instance of the Connection class is initialized in the Drone class __init__ method. The Connection class works on an different thread than the Drone class, so messages can continue to be sent and receiving even while blocking code.

Commands are sent using the 'send_mav_command' method of the Connection class. The first input is the type of command specified by the enum mavutil.mavlink. The other next 7 inputs are all parameters specific to a command (the last three always correspond to x, y, z if applicable).

Not all autopilots implement all commands in the same way. The simulator accepts a limited set of MAV_CMDs. For a list of MAV_CMDs implemented in the simulator, see below.

Telemetry data received into the Drone class in the 'decode_mav_msg' callback. This callbacks assigns the data to the appropriate variables of the Drone class depending on the message type. For this project, only limited telemetry is available from the Drone

#### Message Format

We use an edited Mavlink V2 message format. More details on the V1 and V2 formats can be found on [this page](https://mavlink.io/en/protocol.html).

The exact representation we use is:

```sh
uint8_t magic;              ///< protocol magic marker
uint8_t len;                ///< Length of payload
uint8_t incompat_flags;     ///< flags that must be understood
uint8_t compat_flags;       ///< flags that can be ignored if not understood
uint8_t seq;                ///< Sequence of packet
uint8_t sysid;              ///< ID of message sender system/aircraft
uint8_t compid;             ///< ID of the message sender component
uint8_t msgid 0:7;          ///< first 8 bits of the ID of the message
uint8_t msgid 8:15;         ///< middle 8 bits of the ID of the message
uint8_t msgid 16:23;        ///< last 8 bits of the ID of the message
uint8_t payload[max 253];   ///< A maximum of 253 payload bytes
uint16_t checksum;          ///< X.25 CRC
```

10 bytes for the header, 0-253 bytes for the payload (exact size depends on the message), and 2 bytes for the checksum.

#### MAV_CMDs

Only the following MAV_CMDs are implemented for communication with the drone with the 'send_mav_command':

* MAV_CMD_COMPONENT_ARM_DISARM (parameter1 = 1 armed, 0 disarmed)
* MAV_CMD_NAV_GUIDED_ENABLE (parameter1 = 1 enable, 0 disable)
* MAV_CMD_NAV_LOITER_UNLIM (Command the vehicle to the x, y, z position defined in the global frame (longitude, latitue, altitude)
* MAV_CMD_NAV_TAKEOFF (Ascend straight up to the altitude specified by the z parameter)
* MAV_CMD_NAV_LAND (Descend straight down to the altitude specified by the z parameter)

Other MAV_CMDs passed to the simulator will be ignored

#### MAV_MSGs

There are two types of Mavlink telemetry messages currently being sent from the simulator:

* HEARTBEAT (Contains status information about the drone)
* GLOBAL_POSITION_INT (longitude, latitude, altitude, north velocity, east velocity, down velocity)


#### Reference Frames

TODO: Information about local vs global reference frames

### Command Script

TODO:

## Project Walkthrough

## Modifications for Ardupilot


