cmake_minimum_required(VERSION 2.8.3)

project(mrobosub_planning)

find_package(catkin REQUIRED COMPONENTS
    mrobosub_msgs)

catkin_python_setup()

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS rospy mrobosub_msgs)
