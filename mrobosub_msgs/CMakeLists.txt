cmake_minimum_required(VERSION 2.8.3)
project(mrobosub_msgs)

find_package(catkin REQUIRED 
    COMPONENTS std_msgs message_generation)


add_message_files(
    DIRECTORY msg
    FILES 
    HydrophoneAngle.msg
    HeadingRequest.msg
)

add_service_files(
    DIRECTORY srv
    FILES 
    BinCamPos.srv
    ObjectPosition.srv
    PathmarkerAngle.srv
)

generate_messages()


catkin_package(
    # LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS std_msgs message_runtime
)

