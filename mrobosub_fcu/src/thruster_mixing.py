from tf.transformations import euler_matrix
import numpy as np
from math import radians
import rospy
from std_msgs.msg import Float64
from typing_extensions import Callable

from mrobosub_lib.lib import Node

# degrees
THRUSTERS_YAW = map(radians, [-45, 45, 45, -45, 0, 0, 0, 0])
THRUSTERS_PITCH = map(radians, [0, 0, 0, 0, 90, 90, 90, 90])
# meters
THRUSTERS_SURGE = [0.2921, 0.2921, -0.2921, -0.2921, 0.127, 0.127, -0.127, -0.127]
THRUSTERS_SWAY = [0.267, -0.267, 0.267, -0.267, 0.267, -0.267, 0.267, -0.267]
THRUSTERS_TRANSLATIONS = np.array([THRUSTERS_SURGE, THRUSTERS_SWAY, [0.0] * 8]).T
# newtons
THRUSTER_MAX_FORCE = 1.0

THRUSTERS_ROTATIONS = np.array(
    euler_matrix(yaw, pitch, 0.0, "rzyx")[:3, :3]
    for yaw, pitch in zip(THRUSTERS_YAW, THRUSTERS_PITCH)
)
THRUSTERS_FORCE = (
    THRUSTERS_ROTATIONS @ np.array([THRUSTER_MAX_FORCE, 0.0, 0.0])[None, :, None]
).squeeze()
THRUSTERS_TORQUE = np.cross(THRUSTERS_TRANSLATIONS, THRUSTERS_FORCE)
THRUSTER_ALLOCATION_MATRIX = np.hstack((TTHRUSTERS_FORCE, THRUSTERS_TORQUE))
INV_TAM = np.linalg.pinv(THRUSTER_ALLOCATION_MATRIX)

np.set_printoptions(suppress=True, precision=3)
print(f"{THRUSTERS_ROTATIONS=}")
print(f"{THRUSTERS_FORCE=}")
print(f"{THRUSTERS_TORQUE=}")
print(f"{INV_TAM=}")
# this order must match with the order of dofs in TAM
DOFS = "surge", "sway", "heave", "yaw", "pitch", "roll"
NUM_MOTORS = INV_TAM.shape[1]

RATE = 100  # hz


class ThrusterMixing(Node):
    def __init__(self) -> None:
        self.wrench_subs = {
            dof: rospy.Subscriber(
                f"/output_wrench/{dof}", Float64, self.make_wrench_callback(dof)
            )
            for dof in DOFS
        }
        self.wrench = {dof: 0 for dof in DOFS}
        self.motor_pubs = [
            rospy.Publisher(f"/motor_output/{i}", Float64, queue_size=1)
            for i in range(NUM_MOTORS)
        ]

    def run(self):
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / RATE), self.update)
        rospy.spin()

    def make_wrench_callback(self, dof: str) -> Callable[[Float64]]:
        # direction dofs are in newtons, angle dofs are in newton-meters
        def callback(msg: Float64):
            self.wrench[dof] = msg.value

        return callback

    def update(self):
        wrench = np.array(self.wrench.values())
        output = INV_TAM @ wrench
        for i, value in enumerate(output):
            self.motor_pubs[i].publish(value)


if __name__ == "__main__":
    ThrusterMixing().run()
