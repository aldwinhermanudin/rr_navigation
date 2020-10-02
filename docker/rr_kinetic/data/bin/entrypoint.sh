#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/root/catkin_ws/devel/setup.bash"

source "/data/bin/env_setup.sh"
source "/data/bin/set_ros_master_uri.sh" "$@"

# set /etc/hosts
cat /data/hosts >> /etc/hosts

cd "/root/catkin_ws"

exec "/bin/bash"
