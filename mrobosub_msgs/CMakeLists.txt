cmake_minimum_required(VERSION 2.8.3)
project(mrobosub_msgs)

find_package(catkin REQUIRED 
    COMPONENTS std_msgs sensor_msgs message_generation)


add_message_files(
    DIRECTORY msg
    FILES 
    HydrophoneAngle.msg
    HeadingRequest.msg
    Dvl.msg
    MotorState.msg
    Imu_INS.msg
    Imu_PIMU.msg
)

add_service_files(
    DIRECTORY srv
    FILES 
    BinCamPos.srv
    ObjectPosition.srv
    PathmarkerAngle.srv
    HsvFilterImage.srv
)

generate_messages(
    DEPENDENCIES
    sensor_msgs
)


catkin_package(
    # LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS std_msgs message_runtime
)

