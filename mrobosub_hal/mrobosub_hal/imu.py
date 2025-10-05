import rclpy

from inertial_sense_ros2.msg import DIDINS1, PIMU

from mrobosub_lib import Node
from mrobosub_msgs.msg import ImuINS, ImuPIMU


class IMU(Node):
    """
    This node reads did_ins and pimu values from the /ins_eul_uvw_ned and /pimu topics respectively,
    which are published to by the third-party inertial_sense_ros code. Each time there's a fresh value
    published to these topics, it unpacks the value into objects of our custom data types (Imu_INS and Imu_PIMU)
    and publishes this to /imu_INS or /imu_PIMU respectively, from where it is read by downstream nodes.
    """

    def __init__(self):
        super().__init__("imu")
        self.get_logger().info("Launched imu node")

        self.did_ins_sub = self.create_subscription(
            DIDINS1, "/did_ins1", self.did_ins_callback, 1
        )  # for DID_INS1
        self.pimu_sub = self.create_subscription(
            PIMU, "/pimu", self.did_pimu_callback, 1
        )  # for DID_PIMU
        self.did_ins_pub = self.create_publisher(ImuINS, "/imu_INS", qos_profile=1)
        self.pimu_pub = self.create_publisher(ImuPIMU, "/imu_PIMU", qos_profile=1)

    def did_ins_callback(self, msg: DIDINS1):
        (x, y, z) = msg.theta
        m = ImuINS()
        m.header = msg.header
        m.theta.x = x
        m.theta.y = y
        m.theta.z = -z

    def did_pimu_callback(self, msg: PIMU):
        m = ImuPIMU()
        m.header = msg.header
        m.dtheta = msg.dtheta
        m.dvel = msg.dvel
        m.dt = msg.dt

        self.pimu_pub.publish(m)


def main():
    rclpy.init()
    node = IMU()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
