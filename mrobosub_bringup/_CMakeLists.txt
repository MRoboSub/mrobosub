cmake_minimum_required(VERSION 2.8.3)

project(mrobosub_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mrobosub_msgs
)
