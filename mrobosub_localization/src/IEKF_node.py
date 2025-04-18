from mrobosub_lib.lib import Node
from mrobosub_msgs.msg import Imu_INS, Imu_PIMU, Dvl, Pose
from std_msgs.msg import Float64
import numpy as np
import rospy
from IEKF import IEKF, Vec3, SensorNoise


class IekfNode(Node):
    def __init__(self):
        super().__init__("iekf_node")

        I_pose = np.eye(5)
        covariance = np.eye(15)

        process_noise = np.zeros((15, 15))
        process_noise[0:3, 0:3] = np.eye(3) * 1  # Rotation noise
        process_noise[3:6, 3:6] = np.eye(3) * 5  # Velocity noise
        process_noise[6:9, 6:9] = np.eye(3) * 1  # Position noise
        process_noise[9:12, 9:12] = np.eye(3) * 0.1  # Bias noise
        process_noise[12:15, 12:15] = np.eye(3) * 1  # Bias noise

        depth_measurement_noise = np.zeros((3, 3))
        depth_measurement_noise[2, 2] = 0.05  # Depth measurement noise

        dvl_measurement_noise = np.eye(3) * 0.5
        ahrs_measurement_noise = np.eye(3) * 0.02

        measurement_noise: SensorNoise = {
            "depth": depth_measurement_noise,
            "dvl": dvl_measurement_noise,
            "ahrs": ahrs_measurement_noise,
        }

        self.iekf = IEKF(
            initial_state=I_pose,
            initial_covariance=covariance,
            process_noise=process_noise,
            measurement_noise=measurement_noise,
        )

        self.pose_pub = rospy.Publisher("/pose", Pose, queue_size=1)
        self.imu_sub = rospy.Subscriber("/imu_PIMU", Imu_PIMU, self.imu_callback)
        self.ahrs_sub = rospy.Subscriber("/imu_INS", Imu_INS, self.ahrs_callback)
        self.dvl_sub = rospy.Subscriber("/dvl/raw_dvl", Dvl, self.dvl_callback)
        self.depth_sub = rospy.Subscriber(
            "/depth/raw_depth", Float64, self.depth_callback
        )

    def imu_callback(self, msg: Imu_PIMU):
        self.iekf.predict(
            {
                "linear_acceleration": Vec3.from_matrix(msg.dvel),
                "angular_velocity": Vec3.from_matrix(msg.dtheta),
            },
            msg.dt,
        )

    def ahrs_callback(self, msg: Imu_INS):
        self.iekf.update_ahrs(msg.theta)

    def dvl_callback(self, msg: Dvl):
        self.iekf.update_dvl(Vec3(msg.velocityA, msg.velocityB, msg.velocityC))

    def depth_callback(self, msg: Float64):
        self.iekf.update_depth(msg.data)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.pose_pub.publish(self.iekf.state)
            rate.sleep()


if __name__ == "__main__":
    IekfNode().run()
