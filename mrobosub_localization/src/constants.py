from typing import Protocol
import numpy as np
import numpy.typing as npt
from scipy.linalg import block_diag
from scipy.spatial.transform import Rotation

class Constants(Protocol):
    imu_hz: float
    gravity: npt.NDArray

    std_acc_noise: float
    std_acc_bias_noise: float
    std_gyro_noise: float
    std_gyro_bias_noise: float

    cov_acc_noise: npt.NDArray
    cov_acc_bias_noise: npt.NDArray
    cov_gyro_noise: npt.NDArray
    cov_gyro_bias_noise: npt.NDArray

    state_covariance: npt.NDArray

    std_dvl_noise: float
    cov_dvl_noise: npt.NDArray
    dvl_translation: npt.NDArray
    dvl_rotation: npt.NDArray

    std_depth_noise: float
    cov_depth_noise: npt.NDArray
    depth_translation: npt.NDArray

    measurement_covariance: npt.NDArray

class DefaultConstants:
    imu_hz = 20.
    gravity = np.array([0, 0, -9.81])

    # IMU
    std_acc_noise = 20 * 10**-6 * 9.8 * np.sqrt(imu_hz)
    std_acc_bias_noise = 0.0001 * np.sqrt(imu_hz)  # See https://arxiv.org/pdf/1402.5450.pdf
    std_gyro_noise = 0.005 * np.pi / 180 * np.sqrt(imu_hz)
    std_gyro_bias_noise = 0.000618 * 8 / 18 * np.sqrt(imu_hz)

    cov_acc_noise = np.eye(3) * (std_acc_noise**2)
    cov_acc_bias_noise = np.eye(3) * (std_acc_bias_noise**2)
    cov_gyro_noise = np.eye(3) * (std_gyro_noise**2)
    cov_gyro_bias_noise = np.eye(3) * (std_gyro_bias_noise**2)

    state_covariance = block_diag(
        cov_gyro_noise,
        cov_acc_noise,
        np.zeros((3, 3)),
        cov_gyro_bias_noise,
        cov_acc_bias_noise,
    )

    # DVL
    std_dvl_noise = 0.0101 * 2.6
    cov_dvl_noise = np.eye(3) * (std_dvl_noise**2)
    # dvl_translation = np.array([0, 0, 0])
    # dvl_rotation = np.eye(3)
    dvl_translation = np.array([-0.17137, 0.00922, -0.33989])
    dvl_rotation = Rotation.from_euler('xyz', [6, 3, 90], degrees=True).as_matrix()

    # Depth Sensor
    std_depth_noise = 51 * 1 / 100 * 1 / 2
    cov_depth_noise = np.eye(3) * (std_depth_noise**2)
    depth_translation = np.array([0, 0, 0])

    measurement_covariance = np.diag([std_dvl_noise, std_dvl_noise, std_dvl_noise, std_depth_noise]) ** 2
