cmake_minimum_required(VERSION 3.8)
project(mrobosub_hal_imu)

add_subdirectory(inertial-sense-sdk)

include_directories(
  inertial-sense-sdk/src
  inertial-sense-sdk/src/libusb/libusb
  inertial-sense-sdk/src/libusb/linux
  inertial-sense-sdk/src/yaml-cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(mrobosub_msgs REQUIRED)


add_executable(imu_node src/imu_node.cpp)
ament_target_dependencies(imu_node rclcpp mrobosub_msgs)
target_link_libraries(imu_node InertialSenseSDK)

install(TARGETS
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
