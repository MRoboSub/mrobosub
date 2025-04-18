import numpy as np
import numpy.typing as npt
from scipy.linalg import expm, logm, block_diag
from scipy.spatial.transform import Rotation
from numpy.linalg import inv
from typing import TypedDict, NamedTuple

FloatMat = npt.NDArray[np.floating]


class Vec3(NamedTuple):
    x: float
    y: float
    z: float

    def __add__(self, other: "Vec3") -> "Vec3":
        return Vec3(
            x=self.x + other.x,
            y=self.y + other.y,
            z=self.z + other.z,
        )

    def __sub__(self, other: "Vec3") -> "Vec3":
        return Vec3(
            x=self.x - other.x,
            y=self.y - other.y,
            z=self.z - other.z,
        )

    def __mul__(self, other: float) -> "Vec3":
        return Vec3(
            x=self.x * other,
            y=self.y * other,
            z=self.z * other,
        )

    def __rmul__(self, other: float) -> "Vec3":
        return self * other

    def __truediv__(self, other: float) -> "Vec3":
        return self * (1 / other)

    @staticmethod
    def from_matrix(matrix: FloatMat) -> "Vec3":
        return Vec3(matrix[0], matrix[1], matrix[2])

    def as_matrix(self) -> FloatMat:
        return np.array([self.x, self.y, self.z])


class SensorNoise(TypedDict):
    depth: FloatMat
    dvl: FloatMat
    ahrs: FloatMat


class ControlInput(TypedDict):
    linear_acceleration: Vec3
    angular_velocity: Vec3


class IEKF:
    def __init__(
        self,
        initial_state: FloatMat,
        initial_covariance: FloatMat,
        process_noise: FloatMat,
        measurement_noise: SensorNoise,
    ):
        self.state = initial_state
        self.covariance = initial_covariance

        self.process_noise = process_noise
        self.depth_measurement_noise = measurement_noise["depth"]
        self.dvl_measurement_noise = measurement_noise["dvl"]
        self.ahrs_noise = measurement_noise["ahrs"]

        self.bias = np.zeros((2, 3))

        self.dvl_rotation_body = Rotation.from_euler(
            "xyz", [6, 3, 90], degrees=True
        ).as_matrix()
        self.dvl_position_body = self._skew(np.array([-0.17137, 0.00922, -0.33989]))

    def predict(self, control_input: ControlInput, dt: float):
        acceleration = (
            control_input["linear_acceleration"].as_matrix() - self.lin_acc_bias
        )
        gyroscope = control_input["angular_velocity"].as_matrix() - self.ang_vel_bias
        self.last_ang_vel = gyroscope

        gravity = np.array([0, 0, -9.81])

        zeros = np.zeros((3, 3))
        I = np.eye(3)
        g_cross = self._skew(gravity)

        w_skew = self._skew(gyroscope)

        pred_rotation = self.rotation @ expm(w_skew * dt)
        pred_velocity = self.velocity + (self.rotation @ acceleration + gravity) * dt
        pred_position = (
            self.position
            + self.velocity * dt
            + 0.5 * (self.rotation @ acceleration + gravity) * (dt**2)
        )

        new_state = np.eye(5)
        new_state[:3, :3] = pred_rotation
        new_state[:3, 3] = pred_velocity
        new_state[:3, 4] = pred_position

        adjoint = block_diag(self._adjoint(new_state), np.eye(6))

        error_dyanmics = np.block(
            [
                [zeros, zeros, zeros, -pred_rotation, zeros],
                [g_cross, zeros, zeros, -adjoint[3:6, 0:3], -pred_rotation],
                [zeros, I, zeros, -adjoint[6:9, 0:3], zeros],
                [zeros, zeros, zeros, zeros, zeros],
                [zeros, zeros, zeros, zeros, zeros],
            ]
        )

        error_update = expm(error_dyanmics * dt)

        if np.any(np.isnan(error_update)) or np.any(np.isinf(error_update)):
            print("Issue in error_update")
        if np.any(np.isnan(self.covariance)) or np.any(np.isinf(self.covariance)):
            print("Issue in covariance")

        try:
            with np.errstate(all="raise"):
                middle = self.covariance + adjoint @ self.process_noise @ adjoint.T * dt
                self.covariance = error_update @ (middle) @ error_update.T
        except FloatingPointError:
            print("Floating Point Error in covariance update")

        self.state = new_state

        return self.state

    def update_depth(self, depth: float):
        zero = np.zeros((3, 3))
        measurement_jacobian = np.block([zero, zero, np.eye(3), zero, zero])

        pred_measurement = measurement_jacobian @ block_diag(
            self._adjoint(self.inverse_state), np.eye(6)
        )

        measurement = np.array([self.position[0], self.position[1], depth, 0, 1])

        # Make innovation vector
        V = (self.inverse_state @ measurement)[:3]

        inverseNoise = np.zeros((3, 3))
        inverseNoise[-1, -1] = 1 / self.depth_measurement_noise[2, 2]

        pred_meas_cov = inv(pred_measurement @ self.covariance @ pred_measurement.T)
        meas_cov_inv = (
            pred_meas_cov
            - pred_meas_cov
            @ inv(self.rotation.T @ inverseNoise @ self.rotation + pred_meas_cov)
            @ pred_meas_cov
        )

        # Kalman gain
        kalman_gain = self.covariance @ pred_measurement.T @ meas_cov_inv
        state_gain = kalman_gain[:9]
        bias_gain = kalman_gain[9:]

        self.state = expm(self._wedge(state_gain @ V)) @ self.state
        self.bias = self.bias + (bias_gain @ V).reshape(2, 3)

        self.covariance = (
            np.eye(15) - kalman_gain @ pred_measurement
        ) @ self.covariance
        return self.state, self.covariance

    def update_dvl(self, z: Vec3):

        measurement = (
            self.dvl_rotation_body @ z.as_matrix()
            + self.dvl_position_body @ self.last_ang_vel
        )

        zeros = np.zeros((3, 3))
        measurement_jacobian = np.block([zeros, np.eye(3), zeros, zeros, zeros])

        pred_measurement = measurement_jacobian @ block_diag(
            self._adjoint(self.state), np.eye(6)
        )

        full_measurement = np.array(
            [measurement[0], measurement[1], measurement[2], -1, 0]
        )

        # Make innovation vector
        V = (self.state @ full_measurement)[:3]

        meas_cov = inv(
            pred_measurement @ self.covariance @ pred_measurement.T
            + self.rotation @ self.dvl_measurement_noise @ self.rotation.T
        )

        kalman_gain = self.covariance @ pred_measurement.T @ meas_cov
        state_gain = kalman_gain[:9]
        bias_gain = kalman_gain[9:]

        self.state = expm(self._wedge(state_gain @ V)) @ self.state
        self.bias = self.bias + (bias_gain @ V).reshape(2, 3)

        self.covariance = (
            np.eye(15) - kalman_gain @ pred_measurement
        ) @ self.covariance

        return self.state, self.covariance

    def update_ahrs(self, z: FloatMat):
        R_measured = Rotation.from_euler("zyx", z[::-1]).as_matrix()
        # roll, pitch, yaw = z[0], z[1], z[2]
        # R_x = np.array(
        #     [
        #         [1, 0, 0],
        #         [0, np.cos(roll), -np.sin(roll)],
        #         [0, np.sin(roll), np.cos(roll)],
        #     ]
        # )
        #
        # R_y = np.array(
        #     [
        #         [np.cos(pitch), 0, np.sin(pitch)],
        #         [0, 1, 0],
        #         [-np.sin(pitch), 0, np.cos(pitch)],
        #     ]
        # )
        #
        # R_z = np.array(
        #     [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
        # )
        #
        # R_measured = R_z @ R_y @ R_x

        zeros = np.zeros((3, 3))
        measurement_jacobian = np.block([np.eye(3), zeros, zeros, zeros, zeros])

        pred_measurement = measurement_jacobian @ block_diag(
            self._adjoint(self.inverse_state), np.eye(6)
        )

        orientation_error = logm(self.rotation.T @ R_measured)

        if orientation_error.shape[0] > 3:
            orientation_error = orientation_error[0:3, 0:3]
        V = self._vee(orientation_error)

        pred_meas_cov = pred_measurement @ self.covariance @ pred_measurement.T

        # Singular Matrix Error
        regularizationTerm = np.eye(pred_meas_cov.shape[0]) * 1e-3
        meas_cov_inv = inv(pred_meas_cov + self.ahrs_noise + regularizationTerm)

        # Kalman gain
        kalman_gain = self.covariance @ pred_measurement.T @ meas_cov_inv
        state_gain = kalman_gain[:9]
        bias_gain = kalman_gain[9:]

        self.state = expm(self._wedge(state_gain @ V)) @ self.state
        self.bias = self.bias + (bias_gain @ V).reshape(2, 3)

        self.covariance = (
            np.eye(15) - kalman_gain @ pred_measurement
        ) @ self.covariance

        return self.state, self.covariance

    def _skew(self, v):
        return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

    def _adjoint(self, state):
        rotation = state[:3, :3]
        velocity = state[:3, 3]
        position = state[:3, 4]

        adj = np.zeros((9, 9))

        adj[:3, :3] = rotation
        adj[3:6, 3:6] = rotation
        adj[6:9, 6:9] = rotation

        adj[3:6, :3] = self._skew(velocity) @ rotation
        adj[6:9, :3] = self._skew(position) @ rotation

        return adj

    def _wedge(self, x):
        so_3 = self._skew(x[0:3])
        vel = x[3:6].reshape(-1, 1)
        pos = x[6:9].reshape(-1, 1)

        return np.block([[so_3, vel, pos], [np.zeros((2, 5))]])

    def _vee(self, x):
        return np.array([x[2, 1], x[0, 2], x[1, 0]])

    @property
    def rotation(self) -> FloatMat:
        return self.state[:3, :3]

    @property
    def velocity(self) -> FloatMat:
        return self.state[:3, 3]

    @property
    def position(self) -> FloatMat:
        return self.state[:3, 4]

    @property
    def inverse_state(self) -> FloatMat:
        rotation_transpose = self.rotation.T
        # breakpoint()
        return np.block(
            [
                [
                    rotation_transpose,
                    (-rotation_transpose @ self.velocity).reshape(3, 1),
                    (-rotation_transpose @ self.position).reshape(3, 1),
                ],
                [np.zeros((1, 3)), 1, 0],
                [np.zeros((1, 3)), 0, 1],
            ]
        )

    @property
    def ang_vel_bias(self) -> FloatMat:
        return self.bias[0]

    @property
    def lin_acc_bias(self) -> FloatMat:
        return self.bias[1]
