#!/usr/bin/env python

from typing import Any, Tuple, Optional
from typing_extensions import Callable, TYPE_CHECKING
from tf.transformations import euler_matrix
import numpy as np
from math import radians
import rospy
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

from mrobosub_lib.lib import Node
from mrobosub_msgs.msg import MotorState
from dataclasses import dataclass

if TYPE_CHECKING:
    import numpy.typing as npt


@dataclass
class ThrusterDescriptor:
    id: int
    yaw: float  # degrees
    pitch: float  # degrees
    roll: float  # degrees
    surge: float  # meters
    sway: float  # meters
    heave: float  # meters


THRUSTER_MAX_CURRENT_DRAW = 15.0  # amps
SUB_MAX_CURRENT_DRAW = 8.0  # amps
CORNER_THRUSTER_SURGE = 0.2921
CENTER_THRUSTER_SURGE = 0.127
THRUSTER_SWAY = 0.267
THRUSTERS = [
    ThrusterDescriptor(
        id=0,
        yaw=-45,  # yaw of the motor axis, we use z down coordinate frame
        pitch=0,  # pitch of the motor axis
        roll=0,  # roll of the motor axis
        surge=CORNER_THRUSTER_SURGE,  # x position of motor relative to robot origin
        sway=THRUSTER_SWAY,  # y position of motor relative to robot origin
        heave=0,  # z position of motor relative to robot origin
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


# this order must match with the order of dofs in TAM
DOFS = "surge", "sway", "heave", "roll", "pitch", "yaw"
NUM_MOTORS = 8

RATE = 100  # hz

POS_OUTPUT_FIT_CONSTANTS = np.array(
    [
        7.56924697e-01,
        6.36963573e-01,
        -1.07355204e-01,
        -3.36043212e-02,
        -3.57989822e-02,
        5.66506822e-03,
        3.46542474e-03,
        -6.55502962e-04,
        9.43572014e-04,
        -9.97867702e-05,
    ]
)
NEG_OUTPUT_FIT_CONSTANTS = np.array(
    [
        -6.31593255e-01,
        8.30153860e-01,
        8.33952711e-02,
        5.77598731e-02,
        -4.77313219e-02,
        -4.18002921e-03,
        7.27259340e-03,
        7.45132556e-04,
        1.23669564e-03,
        7.04993222e-05,
    ]
)
POS_CURRENT_FIT_CONSTANTS = np.array(
    [
        -2.29807257e00,
        1.41276903e01,
        7.62148798e-02,
        -2.29819557e01,
        -1.10520151e00,
        1.33898496e-02,
        7.22859467e00,
        2.79722052e00,
        -5.63116993e-03,
        -4.46630658e-04,
    ]
)
NEG_CURRENT_FIT_CONSTANTS = np.array(
    [
        -3.18412975e00,
        -1.63211291e01,
        2.47631145e-01,
        -2.77774741e01,
        1.21451522e00,
        5.36855892e-04,
        -9.67907823e00,
        2.93657755e00,
        4.55541983e-03,
        -1.17926955e-04,
    ]
)
POS_POWER_FIT_CONSTANTS = np.array(
    [
        9.53215518e-01,
        1.10333738e00,
        -3.30202018e-01,
        -1.99208517e00,
        1.03546119e-01,
        2.82231393e-02,
        -8.42297013e-01,
        4.47743134e-01,
        -6.28233791e-03,
        -7.06918724e-04,
    ]
)
NEG_POWER_FIT_CONSTANTS = np.array(
    [
        -1.91033060e00,
        -1.05256570e00,
        4.56828578e-01,
        -1.58452791e-01,
        2.59661426e-01,
        -3.32207446e-02,
        -1.07869796e00,
        -2.58917607e-01,
        -8.14000406e-03,
        7.63760288e-04,
    ]
)


class ThrusterMixing(Node):
    def __init__(self) -> None:
        super().__init__("thruster_mixing")
        self.calculate_TAM()
        self.wrench = {dof: 0 for dof in DOFS}
        self.wrench_subs = {
            dof: rospy.Subscriber(
                f"/output_wrench/{dof}", Float64, self.make_wrench_callback(dof)
            )
            for dof in DOFS
        }
        self.motor_pub = rospy.Publisher("/motor_output", MotorState, queue_size=1)
        self.scale_pub = rospy.Publisher("/motor_output/scale", Float64, queue_size=1)
        self.current_pub = rospy.Publisher(
            "/motor_output/current", Float64, queue_size=1
        )
        self.enabled = True
        self.enable_service = rospy.Service(
            "/thruster_mixing/enable", SetBool, self.handle_enable_request
        )

    def handle_enable_request(self, request: SetBoolRequest):
        self.enabled = request.data
        return SetBoolResponse(True, f"Set enabled to {self.enabled}")

    def run(self):
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / RATE), self.update)
        rospy.spin()

    def make_wrench_callback(self, dof: str) -> Callable[[Float64], None]:
        # direction dofs are in newtons, angle dofs are in newton-meters
        def callback(msg: Float64):
            self.wrench[dof] = msg.data

        return callback

    def calculate_TAM(self) -> None:
        self.thruster_max_force = abs(
            np.dot(self.fit_matrix(-1.0, self.voltage), POS_POWER_FIT_CONSTANTS)
        )
        thrusters_force = (
            THRUSTERS_ROTATIONS @ np.array([1.0, 0.0, 0.0])[None, :, None]
        ).squeeze()
        thrusters_torque = np.cross(THRUSTERS_TRANSLATION, thrusters_force)
        self.thruster_allocation_matrix = np.hstack((thrusters_force, thrusters_torque))
        self.inv_tam = np.linalg.pinv(self.thruster_allocation_matrix).T

    def fit_matrix(self, a: float, b: float) -> "npt.NDArray":
        return np.array([1, a, b, a**2, a * b, b**2, a**3, a**2 * b, a * b**2, b**3])

    @property
    def voltage(self) -> float:
        return 14.8

    def calculate_output(self, force: float) -> float:
        if abs(force) < 0.01:
            return 0.0
        fit_mat = self.fit_matrix(force, self.voltage)
        if force > 0:
            output = np.dot(fit_mat, POS_OUTPUT_FIT_CONSTANTS)
            return max(output, 0.0)
        else:
            output = np.dot(fit_mat, NEG_OUTPUT_FIT_CONSTANTS)
            return min(output, 0.0)

    def calculate_outputs(self, demanded_forces: "npt.NDArray") -> "npt.NDArray":
        return np.array([self.calculate_output(force) for force in demanded_forces])

    def expected_current_draw(self, demanded_outputs: "npt.NDArray") -> "npt.NDArray":
        current_draws = []
        for output in demanded_outputs:
            if abs(output) < 0.1:
                current_draws.append(0.0)
            else:
                current_draw = np.dot(
                    self.fit_matrix(output, self.voltage),
                    (
                        POS_CURRENT_FIT_CONSTANTS
                        if output > 0
                        else NEG_CURRENT_FIT_CONSTANTS
                    ),
                )
                current_draws.append(max(current_draw, 0.0))
        return np.array(current_draws)

    def validate_outputs(self, demanded_forces: "npt.NDArray") -> Optional[MotorState]:
        outputs = self.calculate_outputs(demanded_forces)
        current_draws = self.expected_current_draw(outputs)
        if (
            (-1.0 <= outputs).all()
            and (outputs <= 1.0).all()
            and np.max(current_draws) < THRUSTER_MAX_CURRENT_DRAW
            and np.sum(current_draws) < SUB_MAX_CURRENT_DRAW
        ):
            return MotorState(*outputs)
        return None

    def calculate_scaled_outputs(
        self, demanded_forces: "npt.NDArray"
    ) -> Tuple[MotorState, float]:
        outputs = self.validate_outputs(demanded_forces)
        if outputs is not None:
            return (outputs, 1.0)

        lb_outputs = MotorState(*([0.0] * NUM_MOTORS))
        lower_bound = 0.0
        upper_bound = 1.0
        NUM_ITERS = 5
        for _ in range(NUM_ITERS):
            scale = (lower_bound + upper_bound) / 2.0
            outputs = self.validate_outputs(demanded_forces * scale)
            if outputs is None:
                upper_bound = scale
            else:
                lower_bound = scale
                lb_outputs = outputs

        return (lb_outputs, lower_bound)

    def update(self, _timer_event: Any):
        if not self.enabled:
            return
        wrench = np.array(list(self.wrench.values()))
        forces = self.inv_tam @ wrench

        scale = 1.0
        max_demand = np.max(forces)
        if max_demand > self.thruster_max_force:
            scale /= max_demand / self.thruster_max_force
            forces *= scale

        outputs, scale_factor = self.calculate_scaled_outputs(forces)
        if scale_factor != 1.0:
            print("Scale: ", scale_factor)
        scale *= scale_factor
        est_current = np.sum(
            self.expected_current_draw(
                np.array([getattr(outputs, f"motor{i}") for i in range(NUM_MOTORS)])
            )
        )

        self.current_pub.publish(est_current)
        self.scale_pub.publish(scale)
        self.motor_pub.publish(outputs)


if __name__ == "__main__":
    ThrusterMixing().run()
