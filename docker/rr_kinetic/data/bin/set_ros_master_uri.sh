#!/bin/bash

ros_master_uri=""
if [ -z "$1" ]; then
    ros_master_uri="http://172.17.0.1:11311"
    echo "No argument supplied. Setting ROS_MASTER_URI to $ros_master_uri."
else
    ros_master_uri=$1
    echo "Setting ROS_MASTER_URI to $ros_master_uri."
fi

export ROS_MASTER_URI=$ros_master_uri


