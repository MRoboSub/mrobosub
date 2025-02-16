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
from dataclasses import dataclass


@dataclass
class ThrusterDescriptor:
    id: int
    yaw: float  # degrees
    pitch: float  # degrees
    roll: float  # degrees
    surge: float  # meters
    sway: float  # meters
    heave: float  # meters


THRUSTER_MAX_FORCE = 1.0  # newtons
CORNER_THRUSTER_SURGE = 0.2921
CENTER_THRUSTER_SURGE = 0.127
THRUSTER_SWAY = 0.267
THRUSTERS = [
    ThrusterDescriptor(
        id=0,
        yaw=-45,
        pitch=0,
        roll=0,
        surge=CORNER_THRUSTER_SURGE,
        sway=THRUSTER_SWAY,
        heave=0,
    ),
    ThrusterDescriptor(
        id=1,
        yaw=45,
        pitch=0,
        roll=0,
        surge=CORNER_THRUSTER_SURGE,
        sway=-THRUSTER_SWAY,
        heave=0,
    ),
    ThrusterDescriptor(
        id=2,
        yaw=45,
        pitch=0,
        roll=0,
        surge=-CORNER_THRUSTER_SURGE,
        sway=THRUSTER_SWAY,
        heave=0,
    ),
    ThrusterDescriptor(
        id=3,
        yaw=-45,
        pitch=0,
        roll=0,
        surge=-CORNER_THRUSTER_SURGE,
        sway=-THRUSTER_SWAY,
        heave=0,
    ),
    ThrusterDescriptor(
        id=4,
        yaw=0,
        pitch=90,
        roll=0,
        surge=CENTER_THRUSTER_SURGE,
        sway=THRUSTER_SWAY,
        heave=0,
    ),
    ThrusterDescriptor(
        id=5,
        yaw=0,
        pitch=90,
        roll=0,
        surge=CENTER_THRUSTER_SURGE,
        sway=-THRUSTER_SWAY,
        heave=0,
    ),
    ThrusterDescriptor(
        id=6,
        yaw=0,
        pitch=90,
        roll=0,
        surge=-CENTER_THRUSTER_SURGE,
        sway=THRUSTER_SWAY,
        heave=0,
    ),
    ThrusterDescriptor(
        id=7,
        yaw=0,
        pitch=90,
        roll=0,
        surge=-CENTER_THRUSTER_SURGE,
        sway=-THRUSTER_SWAY,
        heave=0,
    ),
]
THRUSTERS.sort(key=lambda thruster: thruster.id)

SUB_FRAME = np.diag([1, -1, -1])
THRUSTERS_ROTATIONS = np.array(
    [
        euler_matrix(radians(thruster.yaw), radians(thruster.pitch), 0.0, "rzyx")[
            :3, :3
        ]
        @ SUB_FRAME
        for thruster in THRUSTERS
    ]
)
THRUSTERS_TRANSLATION = np.array(
    [[thruster.surge, thruster.sway, thruster.heave] for thruster in THRUSTERS]
)

THRUSTERS_FORCE = (
    THRUSTERS_ROTATIONS @ np.array([THRUSTER_MAX_FORCE, 0.0, 0.0])[None, :, None]
).squeeze()
THRUSTERS_TORQUE = np.cross(THRUSTERS_TRANSLATION, THRUSTERS_FORCE)
THRUSTER_ALLOCATION_MATRIX = np.hstack((THRUSTERS_FORCE, THRUSTERS_TORQUE))
INV_TAM = np.linalg.pinv(THRUSTER_ALLOCATION_MATRIX).T

np.set_printoptions(suppress=True, precision=3)
print(f"{THRUSTERS_ROTATIONS=}")
print(f"{THRUSTERS_TRANSLATION=}")
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
        self.motor_pub = rospy.Publisher("/motor_output", MotorState, queue_size=1)
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
        raw_demand = demanded_force / THRUSTER_MAX_FORCE
        return np.clip(raw_demand, -1.0, 1.0)

    def update(self, _timer_event: Any):
        wrench = np.array(list(self.wrench.values()))
        forces = INV_TAM @ wrench

        max_demand = np.max(forces)
        if max_demand > THRUSTER_MAX_FORCE:
            forces /= max_demand
        scaled = THRUSTER_ALLOCATION_MATRIX.T @ forces
        scale = np.nan_to_num(
            np.mean(scaled[wrench != 0] / wrench[wrench != 0]), nan=1.0
        )
        self.scale_pub.publish(scale)

        state = MotorState()
        for i, force in enumerate(forces):
            output = self.motor_force_curve(force)
            setattr(state, f"motor{i}", output)
        self.motor_pub.publish(state)


if __name__ == "__main__":
    ThrusterMixing().run()
