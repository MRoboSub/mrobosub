cmake_minimum_required(VERSION 3.8)
project(mrobosub_localization_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(BOOST REQUIRED 
  COMPONENTS thread
)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Add source files to executable
add_executable(localization 
  src/localization.cpp
)

# Ament automates a lot of the CMake process. 
# We can use ament to include+link rclcpp in one line
ament_target_dependencies(localization
  rclcpp  
  tf2_ros
  geometry_msgs
  nav_msgs
)

target_include_directories(localization PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
)

# Expose the /include directory
target_include_directories(localization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Require C99 and C++17
target_compile_features(localization PUBLIC 
  c_std_99 cxx_std_17
)

# Manually link with gtsam
target_link_libraries(localization
  Eigen3::Eigen
  gtsam
  ${Boost_LIBRARIES}
)


# Copy to the `install/` folder.
install(
  TARGETS localization
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

# If other packages in the workspace need mrobosub_localization_cpp
# ament_export_dependencies(gtsam)
ament_package()
