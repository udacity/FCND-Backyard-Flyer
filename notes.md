# Notes

The project requires two components:

1. The Robo sim communicates with ROS. So we either need to do create a Mavlink to ROS bridge (ros-mavlink might help with this) or communicate with the Sim directly using Mavlink.
2. 3D environment. I think Nick will provide some sort of file and then it would be loaded in Unity? If it's that simple that would be awesome but I'm skeptical.

## Protocols

MAVLink is designed to support sending continuous telemetry streams including position, velocity, attitude and similar key states of a drone.

With UDP hearbeat messages are sent every 1s to notify if the ground control and/or drone is still active.

The nice thing about using Websockets instead of UDP is the connection is kept open the entire time so we don't have to continously send UDP requests.

## Communication Flow

Communication flow for the qgroundcontrol station can be found [here](https://dev.qgroundcontrol.com/en/communication_flow.html). 

#### Mission Protocol



#### Parameter Protocol

#### Command Protocol

#### Camera Protocol

#### Gimbal Protocol

