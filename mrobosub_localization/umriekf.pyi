from typing import Any
import numpy as np

class State:
    def __init__(self, matrix: np.ndarray) -> None: ...
    @staticmethod
    def from_components(
        rotation: np.ndarray, velocity: np.ndarray, position: np.ndarray
    ) -> "State": ...
    @staticmethod
    def identity() -> "State": ...
    def inverse(self) -> "State": ...
    @property
    def matrix(self) -> np.ndarray:
        """
        The inherent matrix that this class wraps
        Shape is (5,5)
        """
        ...
    @matrix.setter
    def matrix(self, value: np.ndarray): ...
    @property
    def rotation(self) -> np.ndarray:
        """
        The rotation component of this class's matrix
        Shape is (3,3)
        """
        ...
    @rotation.setter
    def rotation(self, value: np.ndarray): ...
    @property
    def velocity(self) -> np.ndarray:
        """
        The velocity component of this class's matrix
        Shape is (3,)
        """
        ...
    @velocity.setter
    def velocity(self, value: np.ndarray): ...
    @property
    def position(self) -> np.ndarray:
        """
        The position component of this class's matrix
        Shape is (3,)
        """
        ...
    @position.setter
    def position(self, value: np.ndarray): ...

class IEKF:
    def __init__(
        self, constants: dict[str, Any], init_state: State, init_cov: np.ndarray
    ) -> None:
        """init_cov must have shape (15, 15)"""
        ...
    def predict(self) -> State:
        """returns the most recent prediction of iekf state"""
        ...
    def add_imu_measurement(self, measured_acc: np.ndarray, measured_gyro: np.ndarray):
        """
        runs the update step of the iekf with an imu measurement
        measured_acc must have shape (3,)
        measured_gyro must have shape (3,)
        """
        ...
    def add_dvl_measurement(self, dvl_velocity: np.ndarray):
        """
        runs the update step of the iekf with a dvl measurement
        dvl_velocity must have shape (3,)
        """
        ...
    def add_depth_measurement(self, depth_measurement: float):
        """runs the update step of the iekf with a depth measurement"""
        ...
    def reload_constants(self, constants: dict[str, Any]):
        """updates the saved constants within the IEKF from the provided dictionary"""
        ...
    @property
    def pred_acc_bias(self) -> np.ndarray:
        """the most recent acceleration bias estimate"""
        ...
    @property
    def pred_gyro_bias(self) -> np.ndarray:
        """the most recent gyroscope bias estimate"""
        ...
