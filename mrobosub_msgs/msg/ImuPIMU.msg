std_msgs/Header header
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Vector3 angular_acceleration
float64 dt # Time difference since last measurement in seconds (Used to calculate velocity and accel from diffs)