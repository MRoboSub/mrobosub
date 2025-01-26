from typing import Literal
from typing_extensions import TypeAlias, Annotated
import numpy as np
import numpy.typing as npt
from scipy.linalg import expm, block_diag
from constants import Constants
# from mrobosub_msgs import IMUMessageType, DVLMessageType, DepthMessageType # type: ignore
import rospy # type: ignore

Vec3: TypeAlias = Annotated[npt.NDArray[np.float64], Literal[3]]
Mat3x3: TypeAlias = Annotated[npt.NDArray[np.float64], Literal[3, 3]]
AdjointType: TypeAlias = Annotated[npt.NDArray[np.float64], Literal[9, 9]]

class State:
    __slots__ = 'matrix',

    def __init__(self, matrix: np.ndarray):
        self.matrix = matrix

    @staticmethod
    def identity() -> "State":
        return State(np.eye(5))

    @staticmethod
    def from_components(rotation: Mat3x3, velocity: Vec3, position: Vec3) -> "State":
        state = State.identity()
        state.rotation = rotation
        state.velocity = velocity
        state.position = position
        return state

    @property
    def rotation(self) -> Mat3x3:
        return self.matrix[:3, :3]

    @rotation.setter
    def rotation(self, new_rotation: Mat3x3):
        self.matrix[:3, :3] = new_rotation

    @property
    def velocity(self) -> Vec3:
        return self.matrix[:3, 3]
    
    @velocity.setter
    def velocity(self, new_velocity: Vec3):
        self.matrix[:3, 3] = new_velocity

    @property
    def position(self) -> Vec3:
        return self.matrix[:3, 4]

    @position.setter
    def position(self, new_position: Vec3):
        self.matrix[:3, 4] = new_position

    def __repr__(self) -> str:
        return f'rotation={self.rotation}, ' \
            f'velocity={self.velocity}, ' \
            f'position={self.position}'

def make_skew_sym(vec: Vec3) -> Mat3x3:
    return np.array([
        [0, -vec[2], vec[1]],
        [vec[2], 0, -vec[0]],
        [-vec[1], vec[0], 0]
    ])  # type: ignore

def calc_adjoint(state: State) -> AdjointType:
    rotation = state.rotation
    vel_cross = make_skew_sym(state.velocity)
    pos_cross = make_skew_sym(state.position)
    ZERO = np.zeros((3, 3))
    return np.block(
        [
            [rotation, ZERO, ZERO],
            [vel_cross @ rotation, rotation, ZERO],
            [pos_cross @ rotation, ZERO, rotation],
        ]
    )

def carat(xi):
    """Moves a 9 vector to the Lie Algebra se_2(3).

    Args:
        xi (9 ndarray) : Parametrization of Lie algebra

    Returns:
        xi^ (5,5 ndarray) : Element in Lie Algebra se_2(3)"""
    w_cross = make_skew_sym(xi[0:3])
    v = xi[3:6].reshape(-1, 1)
    p = xi[6:9].reshape(-1, 1)
    return np.block([[w_cross, v, p], [np.zeros((2, 5))]])

def calc_right_invariant_error(state: State, constants: Constants):
    _ = np.zeros((3, 3))
    gskew = make_skew_sym(constants.gravity)
    rot = state.rotation
    vxr = make_skew_sym(state.velocity) @ rot
    pxr = make_skew_sym(state.position) @ rot
    right_invariant_error = np.block([
        [_, _, _, -rot, _],
        [gskew, _, _, -vxr, -rot],
        [_, np.eye(3), _, -pxr, -rot],
        [_, _, _, _, _],
        [_, _, _, _, _],
    ])

    return right_invariant_error

def change_of_basis(basis: np.ndarray, value: np.ndarray) -> np.ndarray:
    return basis @ value @ basis.T


class IEKF:
    def __init__(self, constants: Constants, initial_state: State, init_cov: np.ndarray) -> None:
        self.constants = constants

        self.pred_state = initial_state
        
        self.pred_cov = init_cov
        self.pred_acc_bias = np.zeros((3,))
        self.pred_gyro_bias = np.zeros((3,))

        self.pred_biased_acc = np.zeros((3,))
        self.pred_biased_gyro = np.zeros((3,))

        self.last_imu_time = 0.

    @property
    def adj_xb(self) -> np.ndarray:
        return block_diag(calc_adjoint(self.pred_state), np.eye(6))

    def predict(self) -> State:
        return self.pred_state

    def add_imu_measurement(self, measured_acc: Vec3, measured_gyro: Vec3, dt: float):
        # Add noise to IMU acceleration measurement
        # pred_acc_noise = np.zeros(3)
        pred_acc_noise = np.random.multivariate_normal(np.zeros(3), self.constants.cov_acc_noise)
        # pred_acc_bias_noise = np.zeros(3)
        pred_acc_bias_noise = np.random.multivariate_normal(np.zeros(3), self.constants.cov_acc_bias_noise)
        pred_acc = measured_acc + pred_acc_noise + pred_acc_bias_noise

        # Add noise to IMU gyro measurement
        # pred_gyro_noise = np.zeros(3)
        pred_gyro_noise = np.random.multivariate_normal(np.zeros(3), self.constants.cov_gyro_noise)
        # pred_gyro_bias_noise = np.zeros(3)
        pred_gyro_bias_noise = np.random.multivariate_normal(np.zeros(3), self.constants.cov_gyro_bias_noise)
        pred_gyro = measured_gyro + pred_gyro_noise + pred_gyro_bias_noise

        pred_prev_state = self.pred_state

        self.pred_acc_bias = np.zeros((3,))
        self.pred_gyro_bias = np.zeros((3,))
        self.pred_biased_acc = pred_acc - self.pred_acc_bias
        pred_global_acc = self.pred_state.rotation @ self.pred_biased_acc + self.constants.gravity
        self.pred_biased_gyro = pred_gyro - self.pred_gyro_bias

        # Calculate updated state
        pred_rotation = pred_prev_state.rotation @ expm(make_skew_sym(self.pred_biased_gyro * dt))
        pred_vel = pred_prev_state.velocity + pred_global_acc * dt
        pred_pos = pred_prev_state.position + pred_prev_state.velocity * dt + 0.5 * pred_global_acc * dt**2

        self.pred_state = State.from_components(pred_rotation, pred_vel, pred_pos)

        phi = expm(calc_right_invariant_error(self.pred_state, self.constants) * dt)
        self.pred_cov = change_of_basis(phi, (self.pred_cov + change_of_basis(self.adj_xb, self.constants.state_covariance) * dt))

    def add_dvl_measurement(self, dvl_velocity: Vec3):
        # Add noise to measurement
        pred_dvl_noise = np.random.multivariate_normal(np.zeros(3), self.constants.cov_dvl_noise)
        pred_dvl_vel = dvl_velocity + pred_dvl_noise

        # Convert predicted velocity into IMU reference frame
        pred_vel_in_imu_frame = np.array([*self.constants.dvl_rotation @ pred_dvl_vel + make_skew_sym(self.constants.dvl_translation) @ self.pred_biased_gyro, -1, 0])

        # Note: Currently just using the latest ang velocity reading from the IMU
        # TODO: Convert to using a queue to align IMU & DVL measurements if performance is bad
        cov_pred_vel_in_imu_frame = change_of_basis(self.constants.dvl_rotation, self.constants.cov_dvl_noise) \
            + change_of_basis(make_skew_sym(self.constants.dvl_translation), self.constants.cov_gyro_noise + self.constants.cov_gyro_bias_noise)

        # Calculate the measurement covariance
        measurement_covariance = np.linalg.inv(self.pred_cov[3:6, 3:6] + change_of_basis(self.pred_state.rotation, cov_pred_vel_in_imu_frame))

        # Calculate the Kalman gains
        kalman_gains = self.pred_cov[:, 3:6] @ measurement_covariance
        state_kalman_gain, bias_kalman_gain = np.split(kalman_gains, [9], axis=0)

        #Update the current state estimate to include the DVL measurement
        PI = np.block([np.eye(3), np.zeros((3, 2))])
        self.pred_state = State(expm(carat(state_kalman_gain @ (self.pred_state.matrix @ pred_vel_in_imu_frame)[:3])) @ self.pred_state.matrix)

        (gyro_bias_update, acc_bias_update) = (bias_kalman_gain @ (self.pred_state.matrix @ pred_vel_in_imu_frame)[:3]).reshape((2, 3))
        self.pred_gyro_bias = self.pred_gyro_bias + gyro_bias_update
        self.pred_acc_bias = self.pred_acc_bias + acc_bias_update

        H = np.block([np.zeros((3,3)), np.eye(3), np.zeros((3, 9))])
        self.pred_cov = (np.eye(15) - kalman_gains @ H) @ self.pred_cov

    def add_depth_measurement(self, depth_measurement: float):
        H = np.block([np.zeros((3,6)), np.eye(3), np.zeros((3, 6))])
        depth_measurement_from_imu = depth_measurement + (self.pred_state.rotation @ self.constants.depth_translation)[2]
        cov_squiggle = np.linalg.inv(change_of_basis(H @ self.adj_xb, self.pred_cov))
        foo = np.zeros((3, 3))
        foo[2, 2] = 1. / self.constants.std_depth_noise**2
        measurement_covariance = cov_squiggle - cov_squiggle @ np.linalg.inv(change_of_basis(self.pred_state.rotation.T, foo) + cov_squiggle) @ cov_squiggle
        pseudo_measurement = np.array([self.pred_state.matrix[0,4], self.pred_state.matrix[1,4], depth_measurement_from_imu, 0, 1])

        kalman_gains = self.pred_cov @ self.adj_xb.T @ H.T @ measurement_covariance
        state_kalman_gain, bias_kalman_gain = np.split(kalman_gains, [9], axis=0)

        # breakpoint()
        #Update the current state estimate to include the DVL measurement
        self.pred_state = State(expm(carat(state_kalman_gain @ (np.linalg.inv(self.pred_state.matrix) @ pseudo_measurement)[:3])) @ self.pred_state.matrix)

        (gyro_bias_update, acc_bias_update) = (bias_kalman_gain @ (np.linalg.inv(self.pred_state.matrix) @ pseudo_measurement)[:3]).reshape((2, 3))
        self.pred_gyro_bias = self.pred_gyro_bias + gyro_bias_update
        self.pred_acc_bias = self.pred_acc_bias + acc_bias_update
        
        self.pred_cov = (np.eye(15) - kalman_gains @ H @ self.adj_xb) @ self.pred_cov
