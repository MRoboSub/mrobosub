import rclpy
from mrobosub_lib import Node

from mrobosub_msgs.msg import MotorState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MotorTestAll(Node):
    STOP_POWER = 0
    FORWARD_POWER = 0.1
    MOTOR_TIME_S = 8
    NUM_MOTORS = 8

    def __init__(self):
        super().__init__("motor_test")
        self.on = True
        qos_profile = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.pub = self.create_publisher(MotorState, "/motor_output", qos_profile)
        self.timer = self.create_timer(self.MOTOR_TIME_S, self.loop)
    
    def loop(self):

        msg = MotorState()

        # run all motors for 10 seconds, then stop all motors forever
        if self.on:
            self.get_logger().info("Running all motors for 8 seconds")
            for i in range(self.NUM_MOTORS):
                msg.motors[i] = self.FORWARD_POWER
                self.on = False
            self.pub.publish(msg)
                
        else:
            self.get_logger().info("Motor test complete")
            for i in range(self.NUM_MOTORS):
                msg.motors[i] = self.STOP_POWER
            self.pub.publish(msg)

            # kill the node
            self.timer.cancel()
            self.destroy_node()
            rclpy.shutdown()

def main():
    rclpy.init()
    node = MotorTestAll()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
