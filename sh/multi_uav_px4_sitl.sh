#!/bin/bash


mkdir -p /home/rosuser_docs/ROS/ROSProject/UAV/ros1_quad/ros1_quad_ws/build/px4_sitl_default/tmp/instance_0
cd /home/rosuser_docs/ROS/ROSProject/UAV/ros1_quad/PX4-Autopilot
./build/px4_sitl_default/bin/px4 \
  -i 0 \
  -w /home/rosuser_docs/ROS/ROSProject/UAV/ros1_quad/ros1_quad_ws/build/px4_sitl_default/tmp/instance_0 \
  -s /home/rosuser_docs/ROS/ROSProject/UAV/ros1_quad/ros1_quad_ws/src/centralized_control/px4_config/ROMFS/px4fmu_common/init.d-posix/rcS_01

