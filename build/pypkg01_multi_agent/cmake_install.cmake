# Install script for directory: /home/rosuser_docs/ROS/ROSProject/UAV/ros1_quad/ros1_quad_ws/src/pypkg01_multi_agent

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/rosuser_docs/ROS/ROSProject/UAV/ros1_quad/ros1_quad_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/rosuser_docs/ROS/ROSProject/UAV/ros1_quad/ros1_quad_ws/build/pypkg01_multi_agent/catkin_generated/installspace/pypkg01_multi_agent.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pypkg01_multi_agent/cmake" TYPE FILE FILES
    "/home/rosuser_docs/ROS/ROSProject/UAV/ros1_quad/ros1_quad_ws/build/pypkg01_multi_agent/catkin_generated/installspace/pypkg01_multi_agentConfig.cmake"
    "/home/rosuser_docs/ROS/ROSProject/UAV/ros1_quad/ros1_quad_ws/build/pypkg01_multi_agent/catkin_generated/installspace/pypkg01_multi_agentConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pypkg01_multi_agent" TYPE FILE FILES "/home/rosuser_docs/ROS/ROSProject/UAV/ros1_quad/ros1_quad_ws/src/pypkg01_multi_agent/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pypkg01_multi_agent" TYPE PROGRAM FILES "/home/rosuser_docs/ROS/ROSProject/UAV/ros1_quad/ros1_quad_ws/build/pypkg01_multi_agent/catkin_generated/installspace/test01.py")
endif()

