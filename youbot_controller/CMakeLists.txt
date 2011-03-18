cmake_minimum_required(VERSION 2.6.3)

project(youbot_controller)

set (ROS_ROOT $ENV{ROS_ROOT} )
if (ROS_ROOT)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  rosbuild_find_ros_package( rtt )
  set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/install )
endif()

find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

orocos_component(youbot_controller src/controller.cpp)
orocos_install_headers(src/controller.hpp)
orocos_generate_package()
