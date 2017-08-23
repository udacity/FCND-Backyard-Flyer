#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
# export ip address
export ROS_IP=`echo $(hostname -I)`

exec "$@"
