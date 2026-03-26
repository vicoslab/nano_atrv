#!/bin/bash
source /home/vicos/ros_env.sh

#log clean
rm -rf /home/vicos/.ros/log/*

#run
ros2 launch nano_atrv_bringup boot.launch.xml
