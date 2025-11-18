import rclpy
import sys
import argparse

from mrobosub_lib import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Float64
from rclpy

INTEGRAL_DEADBAND = 1 # integral term only changes when pose within [setpoint - INTEGRAL_DEADBAND, setpoint + INTEGRAL_DEADBAND]
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

        self.declare_parameter(
            "feed_forward",
            rclpy.Parameter.Type.DOUBLE,
            descriptor=ParameterDescriptor(description="Feed forward control value"),
        )

        # self.subscriber = self.create_subscription(
        #     Float64,
        #     f"/{dof_name}_pid/control_effort",
        #     self.control_effort_callback,
        #     qos_profile=1,
        # )
        self.output = 0 # current output of PID algo

        self.previous_valid = False
        self.previous_pose = 0 # for D term
        self.prev_time = self.get_clock().now()

        self.accumulated_error = 0 # accumulated integral of error
    
        self.pose = 0
        self.target_pose = 0 # setpoint

        self.pid_enabled = False

        self.Kp = 0 # todo, parse rosparams for each of these
        self.Kd = 0
        self.Ki = 0
        self.angular = True
        self.integral_windup_limit = ... # max integral accumulated
        # integral term does not get updated if update will take it outside [ self.integral_windup_limit, self.integral_windup_limit ]

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
        self.pose = pose.data
        error = self.target_pose - self.pose

        if(self.angular):
            # if input error % 360 will be in range [0, 360)
            # if input error % 360 is < 180 then output +ve value
            # if input error % 360 is >=180 then output -ve value
            error = self.convert_to_180_180_range(error)
            # now output error is in [-180, 180)
            add to the accumulated error

        else:
            pass



        if not self.previous_valid:
            self.previous_valid = True
            self.prev_time = self.get_clock().now()
            self.previous_pose = self.pose
            self.accumulated_error = 0

        else:
            cur_time = self.get_clock().now()
            delta_time = self.elapsed_ms(self.prev_time, cur_time)
            derivative_diff = (self.pose - self.previous_pose) # don't do modulo anything, this should be the absolute value with sign and everything
            derivative_term = derivative_diff / delta_time

            


            pid = error * self.Kp + derivative_term * self.Kd + self.accumulated_error * self.Ki 

            self.prev_time = cur_time # setting this up for the next iteration

        # calculate and ouptut value here!!!               
       


    def target_twist(self, target_twist: Float64):
        self.pid_enabled = False
        self.pub_output(target_twist.data)

    def pid_callback(self, effort: float): # wth is this function for
        output = (
            effort
            + self.get_parameter("feed_forward").get_parameter_value().double_value
        )
        self.pub_output(output)

    def pub_output(self, output: float):
        self.output_pub.publish(Float64(data=output))

    def destroy_node(self):
        self.output_pub.publish(Float64(data=0.0))
        super().destroy_node()
 
    def control_effort_callback(self, effort: Float64):
        self.effort = effort.data
        self.pid_callback(effort.data)

def main():
    rclpy.init()

    parser = argparse.ArgumentParser()
    parser.add_argument("dof_name", type=str, help="Name of the DOF")
    args = parser.parse_args(sys.argv[1:2])

    node = PidDofControlNode(args.dof_name)
    rclpy.spin(node)


if __name__ == "__main__":
    main()