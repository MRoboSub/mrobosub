#!/usr/bin/env python

from tf.transformations import euler_matrix
import numpy as np
from math import radians
import rospy
from std_msgs.msg import Float64
from typing import Any
from typing_extensions import Callable

from mrobosub_lib.lib import Node
from mrobosub_msgs.msg import MotorState

# degrees
THRUSTERS_YAW = map(radians, [-45, 45, 45, -45, 0, 0, 0, 0])
THRUSTERS_PITCH = map(radians, [0, 0, 0, 0, 90, 90, 90, 90])
# meters
THRUSTERS_SURGE = [0.2921, 0.2921, -0.2921, -0.2921, 0.127, 0.127, -0.127, -0.127]
THRUSTERS_SWAY = [0.267, -0.267, 0.267, -0.267, 0.267, -0.267, 0.267, -0.267]
THRUSTERS_TRANSLATIONS = np.array([THRUSTERS_SURGE, THRUSTERS_SWAY, [0.0] * 8]).T
# newtons
THRUSTER_MAX_FORCE = 1.0

SUB_FRAME = np.diag([1, -1, -1])
THRUSTERS_ROTATIONS = np.array(
    [
        euler_matrix(yaw, pitch, 0.0, "rzyx")[:3, :3] @ SUB_FRAME
        for yaw, pitch in zip(THRUSTERS_YAW, THRUSTERS_PITCH)
    ]
)

THRUSTERS_FORCE = (
    THRUSTERS_ROTATIONS @ np.array([THRUSTER_MAX_FORCE, 0.0, 0.0])[None, :, None]
).squeeze()
THRUSTERS_TORQUE = np.cross(THRUSTERS_TRANSLATIONS, THRUSTERS_FORCE)
THRUSTER_ALLOCATION_MATRIX = np.hstack((THRUSTERS_FORCE, THRUSTERS_TORQUE))
INV_TAM = np.linalg.pinv(THRUSTER_ALLOCATION_MATRIX).T

np.set_printoptions(suppress=True, precision=3)
print(f"{THRUSTERS_ROTATIONS=}")
print(f"{THRUSTERS_FORCE=}")
print(f"{THRUSTERS_TORQUE=}")
print(f"{INV_TAM=}")
print(f"{INV_TAM.shape=}")

# this order must match with the order of dofs in TAM
DOFS = "surge", "sway", "heave", "roll", "pitch", "yaw"
NUM_MOTORS = INV_TAM.shape[0]

RATE = 100  # hz


class ThrusterMixing(Node):
    def __init__(self) -> None:
        super().__init__("thruster_mixing")
        self.wrench = {dof: 0 for dof in DOFS}
        self.wrench_subs = {
            dof: rospy.Subscriber(
                f"/output_wrench/{dof}", Float64, self.make_wrench_callback(dof)
            )
            for dof in DOFS
        }
        self.motor_pubs = [
            rospy.Publisher(f"/motor_output/{i}", Float64, queue_size=1)
            for i in range(NUM_MOTORS)
        ]
        self.all_motor_pub = rospy.Publisher("/motor_output", MotorState, queue_size=1)
        self.scale_pub = rospy.Publisher("/motor_output/scale", Float64, queue_size=1)

    def run(self):
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / RATE), self.update)
        rospy.spin()

    def make_wrench_callback(self, dof: str) -> Callable[[Float64], None]:
        # direction dofs are in newtons, angle dofs are in newton-meters
        def callback(msg: Float64):
            self.wrench[dof] = msg.data

        return callback

    def motor_force_curve(self, demanded_force: float) -> float:
        """Returns required motor output power for a certain demanded torque"""
        # This should probably be nonlinear according to the datasheet or experimental data
        # https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/
        return demanded_force / THRUSTER_MAX_FORCE

    def update(self, _timer_event: Any):
        wrench = np.array(list(self.wrench.values()))
        forces = INV_TAM @ wrench

        max_demand = np.max(forces)
        if max_demand > THRUSTER_MAX_FORCE:
            forces /= max_demand
        scaled = THRUSTER_ALLOCATION_MATRIX.T @ forces
        scale = np.nan_to_num(np.mean(scaled[wrench != 0] / wrench[wrench != 0]), nan=1.)
        self.scale_pub.publish(scale)

        state = MotorState()
        for i, force in enumerate(forces):
            output = self.motor_force_curve(force)
            self.motor_pubs[i].publish(output)
            setattr(state, f"motor{i}", output)
        self.all_motor_pub.publish(state)


if __name__ == "__main__":
    ThrusterMixing().run()
