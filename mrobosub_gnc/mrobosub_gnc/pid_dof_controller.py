import rclpy
import sys
import argparse

from mrobosub_lib import Node
from std_msgs.msg import Float64
import math

class PidDofControlNode(Node):
    """
    Subscribers
    - /target_pose/{dof_name} (deg)
    - /pose/{dof_name} (deg)
    - /target_twist/{dof_name} (power)

    Publishers
    - /output_wrench/{dof_name}
    """

    def __init__(self, dof_name: str):
        super().__init__(f"{dof_name}_control")
        self.get_logger().info(f"Starting PID control node for: {dof_name}")

        self.declare_params()
        self.add_post_set_parameters_callback(self.set_params)

        self.output = 0.0 # current output of PID algo

        self.prev_valid = False
        self.prev_pose = 0 # for D term
        self.prev_time = self.get_clock().now()

        self.accumulated_error = 0.0 # accumulated integral of error
    
        self.pose = 0.0
        self.target_pose = 0.0 # setpoint

        self.pid_enabled = False

        self.output_pub = self.create_publisher(
            Float64, f"/output_wrench/{dof_name}", qos_profile=10
        )
        self.create_subscription(
            Float64,
            f"/target_pose/{dof_name}",
            self.target_pose_callback,
            qos_profile=1,
        )
        self.create_subscription(
            Float64, f"/pose/{dof_name}", self.pose_callback, qos_profile=1
        )
        self.create_subscription(
            Float64, f"/target_twist/{dof_name}", self.target_twist, qos_profile=1
        )

    def set_params(self, _params = None):    
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.angular = self.get_parameter('angular').value
        self.integral_windup_limit = self.get_parameter('integral_windup_limit').get_parameter_value().double_value # absolute value of max integral accumulated
        # ideally the parameter being passed in should be positive, but if negative, we use abs() whenever we use self.integral_windup_limit so it's ok
        # integral term does not go outside [ - self.integral_windup_limit, + self.integral_windup_limit ]

        self.integral_deadband = self.get_parameter('integral_deadband').get_parameter_value().double_value
        # integral term only changes when pose within [setpoint - integral_deadband, setpoint + integral_deadband]
        # when pose outside this range, integral term stays constant

    def declare_params(self):
        self.declare_parameter("kp", 1.0)
        self.declare_parameter("kd", 1.0)
        self.declare_parameter("ki", 1.0)
        self.declare_parameter("angular", False)
        self.declare_parameter("integral_windup_limit", 1.0)
        self.declare_parameter("integral_deadband", 1.0)
        self.set_params()

    # returns val wrapped to [-180, 180)
    def wrap_to_180(self, val: float):
        val = val % 360  # val is now in [0, 360)
        if(val > 180):
            val = val - 360
        return val

    def target_pose_callback(self, target_pose: Float64):
        self.pid_enabled = True
        self.target_pose = target_pose.data
        if self.angular:
            self.target_pose = self.wrap_to_180(self.target_pose)

    def elapsed_ms(self, start, end):
        return (end - start).nanoseconds / 1e6

    def pose_callback(self, pose: Float64):
        self.pose: float = pose.data
        if self.angular:
            self.pose = self.wrap_to_180(self.pose)

        if not self.pid_enabled:
            return

        error: float = self.target_pose - self.pose

        if self.angular:
            error = self.wrap_to_180(error) # now error is in [-180, 180)

        if not self.prev_valid:
            self.prev_valid = True
            self.accumulated_error = 0

        else:
            cur_time = self.get_clock().now()
            delta_time = self.elapsed_ms(self.prev_time, cur_time)
            pose_diff: float = (self.pose - self.prev_pose)

            if self.angular:
                pose_diff = self.wrap_to_180(pose_diff)
                # as long as we don't turn over 180 degrees in a single timestep (which should not happen), this will be accurate

            derivative_term: float = - pose_diff / delta_time

            # update self.accumulated_error
            if abs(error) <= self.integral_deadband:
                self.accumulated_error += error * delta_time # tentatively update self.accumulated error

                # if tentative value of self.accumulated_error is more than our max accumulated error (windup_limit), then cap it at +/- windup_limit
                if abs(self.accumulated_error) > abs(self.integral_windup_limit):
                    self.accumulated_error = math.copysign(self.integral_windup_limit, self.accumulated_error)

            self.output = error * self.kp + derivative_term * self.kd + self.accumulated_error * self.ki
            self.output_pub.publish(Float64(data=self.output))

        # setting up values for the next iteration
        self.prev_time = self.get_clock().now() 
        self.prev_pose = self.pose  

    def target_twist(self, target_twist: Float64):
        self.pid_enabled = False
        self.prev_valid = False
        self.output_pub.publish(target_twist)

    def destroy_node(self):
        self.output_pub.publish(Float64(data=0.0))
        super().destroy_node()

def main():
    rclpy.init()

    parser = argparse.ArgumentParser()
    parser.add_argument("dof_name", type=str, help="Name of the DOF")
    args = parser.parse_args(sys.argv[1:2])

    node = PidDofControlNode(args.dof_name)
    rclpy.spin(node)

if __name__ == "__main__":
    main()