#!/bin/bash
export CYCLONEDDS_URI='/home/vicos/cyclonedds.xml'

export ROS_DOMAIN_ID=55
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source /opt/ros/jazzy/setup.bash
source /home/vicos/colcon_ws/install/setup.bash


