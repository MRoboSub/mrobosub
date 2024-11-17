cmake_minimum_required(VERSION 2.8.3)

project(mrobosub_safety)

find_package(catkin REQUIRED COMPONENTS std_msgs)

# catkin_python_setup()
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS rospy std_msgs)
