# Why is this not in .bashrc (see issue #148)
export PYTHON_VERSION="$(python --version 2>/dev/null | awk '{v=$2; split(v,a,"."); print "python" a[1]"."a[2]}')"
export MYPYPATH="/opt/ros/$ROS_DISTRO/lib/$PYTHON_VERSION/site-packages:/opt/ros/$ROS_DISTRO/local/lib/$PYTHON_VERSION/dist-packages"

# ROS 2 build alias
alias build="(cd ~/ros2_ws/; colcon build --symlink-install); source ~/ros2_ws/install/local_setup.bash"

# Typos and abbreviations
alias ronosde="ros2 node"
alias gs="git status"

# Service calls for frequently called services
alias bot_cam="ros2 service call /bot_cam/on std_srvs/srv/SetBool \"{data: true}\"; ros2 service call /zed/on std_srvs/srv/SetBool \"{data: false}\""
alias zed="ros2 service call /zed/on std_srvs/srv/SetBool \"{data: true}\"; ros2 service call /bot_cam/on std_srvs/srv/SetBool \"{data: false}\"" 
alias cams_off="ros2 service call /zed/on std_srvs/srv/SetBool \"{data: false}\"; ros2 service call /bot_cam/on std_srvs/srv/SetBool \"{data: false}\""
alias arm="ros2 service call /thruster_mixing/enable std_srvs/srv/SetBool \"{data: true}\""
alias disarm="ros2 service call /thruster_mixing/enable std_srvs/srv/SetBool \"{data: false}\""
alias stop_motors="ros2 service call /emergency_stop_motors std_srvs/srv/SetBool \"{data: true}\""

alias zero_depth="ros2 service call /depth/zero std_srvs/srv/SetBool \"{data: true}\""

# ros2 service call /localization/zero_state std_srvs/srv/SetBool "{data: true}"

# Quickly start the (default) state machine
alias captain="ros2 launch mrobosub_planning captain_launch.xml"

# Control the droppers
alias close_droppers="ros2 topic pub /left_servo/angle std_msgs/msg/Int32 \"data: 90\" & ros2 topic pub /right_servo/angle std_msgs/Int32 \"data: 90\""
alias open_droppers="ros2 topic pub /left_servo/angle std_msgs/msg/Int32 \"data: 60\" & ros2 topic pub /right_servo/angle std_msgs/Int32 \"data: 120\""

# Not sure what this does.
alias temps="watch -n 2 sensors"

# to publish to output wrench directly (i.e., bypass PID):
# ros2 topic pub /output_wrench/heave std_msgs/msg/Float64 "{data: 0.4}"

# to set a param
# ros2 param set <node_name> <param_name> <value>

# bringup (first command to bring up all imp nodes on the sub)
alias bringup="ros2 launch mrobosub_bringup bringup_launch.xml"

# to run motor tests
alias motor_test="ros2 launch mrobosub_tests motor_test_launch.xml"
alias motor_test_all="ros2 launch mrobosub_tests motor_test_all_launch.xml"

alias start_tc="ros2 launch mrobosub_hal thruster_controller_launch.xml"
alias start_esp="ros2 launch mrobosub_hal esp32_launch.xml"
alias start_mta="ros2 launch mrobosub_tests motor_test_all_launch.xml"
alias arduino="ros2 launch mrobosub_hal arduino_launch.xml"
alias localize="ros2 launch mrobosub_localization localization_launch.xml"
alias start_imu="ros2 launch mrobosub_hal_imu imu_launch.xml"
alias thruster_mixing="ros2 launch mrobosub_fcu thruster_mixing_launch.xml"

alias start_zed="ros2 launch mrobosub_hal zed_launch.xml"
alias start_botcam="ros2 launch mrobosub_hal botcam_launch.xml"

alias quartermaster="ros2 launch mrobosub_bringup quartermaster_launch.xml"

# ros2 launch mrobosub_gnc heave_launch.xml
