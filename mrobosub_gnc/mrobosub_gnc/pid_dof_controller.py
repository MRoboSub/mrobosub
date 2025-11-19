import rclpy
import sys
import argparse

from mrobosub_lib import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Float64

INTEGRAL_DEADBAND = 15 # integral term only changes when pose within [setpoint - INTEGRAL_DEADBAND, setpoint + INTEGRAL_DEADBAND]
# when outside this range, keep integral term constant

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

        self.output = 0 # current output of PID algo

        self.previous_valid = False
        self.previous_pose = 0 # for D term
        self.prev_time = self.get_clock().now()

        self.accumulated_error = 0 # accumulated integral of error
    
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
        ) # subscribe to twist

    def set_params(self, _params = None):
        self.feed_forward = self.get_parameter('feed_forward').get_parameter_value().double_value
    
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.angular = self.get_parameter('angular').value
        self.integral_windup_limit = self.get_parameter('integral_windup_limit').get_parameter_value().double_value # absolute value of max integral accumulated
        # integral term does not get updated if update will take it outside [ - self.integral_windup_limit, self.integral_windup_limit ]

    def declare_params(self):
        self.declare_parameter("feed_forward", 10)
        self.declare_parameter("kp", 1)
        self.declare_parameter("kd", 1)
        self.declare_parameter("ki", 1)
        self.declare_parameter("angular", False)
        self.declare_parameter("integral_windup_limit", 1)
        self.set_params()

    def target_pose_callback(self, target_pose: Float64):
        self.pid_enabled = True
        self.target_pose = target_pose.data

    # returns the value in range [-180, 180)
    def convert_to_180_180_range(self, val):
        val = val % 360
        if(val > 360):
            val = 360 - val
        return val

    def elapsed_ms(self, start, end):
        return (end - start).nanoseconds / 1e6

    def pose_callback(self, pose: Float64):
        self.pose: float = pose.data
        error: float = self.target_pose - self.pose

        if self.angular:
            # if input error % 360 will be in range [0, 360)
            # if input error % 360 is < 180 then output +ve value
            # if input error % 360 is >=180 then output -ve value
            error = self.convert_to_180_180_range(error)
            # now output error is in [-180, 180)

        if not self.previous_valid:
            self.previous_valid = True
            self.previous_pose = self.pose
            self.accumulated_error = 0

        else:
            cur_time = self.get_clock().now()
            delta_time = self.elapsed_ms(self.prev_time, cur_time)
            pose_diff: float = (self.pose - self.previous_pose) # don't do modulo anything, this should be the absolute value with sign and everything
            derivative_term: float = pose_diff / delta_time

            # update self.accumulated error
            if(abs(error) <= INTEGRAL_DEADBAND):
                new_accumulated_error: float = self.accumulated_error + error * delta_time
                if(abs(new_accumulated_error) <= abs(self.integral_windup_limit)):
                    self.accumulated_error = new_accumulated_error

            effort: float = error * self.kp + derivative_term * self.kd + self.accumulated_error * self.ki
            self.pid_callback(effort)

        self.prev_time = self.get_clock().now() # setting this up for the next iteration     

    def target_twist(self, target_twist: Float64):
        self.pid_enabled = False
        self.previous_valid = False
        self.pub_output(target_twist.data)

    def pid_callback(self, effort: float): # wth is this function for
        self.output = (
            effort
            + self.get_parameter("feed_forward").get_parameter_value().double_value
        )
        self.pub_output(self.output)

    def pub_output(self, output: float):
        self.output_pub.publish(Float64(data=output))

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