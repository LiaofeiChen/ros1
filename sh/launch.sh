#!/bin/bash

clear
rm -rf build devel
export PYTHONPATH=$PYTHONPATH:/home/rosuser_docs/ROS/ROSProject/UAV/ros1_quad/ros1_quad_ws/src
# 编译工作空间
catkin_make
# source ~/.bashrc
source devel/setup.bash
# roslaunch sim_centralized_control launch.launch
roslaunch pypkg01_multi_agent launch.launch
