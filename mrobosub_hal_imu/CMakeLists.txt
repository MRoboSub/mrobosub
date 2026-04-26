cmake_minimum_required(VERSION 3.8)
project(mrobosub_hal_imu)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
find_package(Threads REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mrobosub_msgs REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/inertial-sense-sdk/src
  ${CMAKE_CURRENT_SOURCE_DIR}/inertial-sense-sdk/src/yaml-cpp
  ${LIBUSB_INCLUDE_DIRS}
)

add_subdirectory(inertial-sense-sdk)

add_executable(imu_node src/imu_node.cpp)

ament_target_dependencies(imu_node rclcpp mrobosub_msgs)

target_link_libraries(imu_node 
    InertialSenseSDK 
    ${LIBUSB_LIBRARIES}
    Threads::Threads
)

install(TARGETS imu_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()