from tf.transformations import rotation_from_euler
import numpy as np

THRUSTER_YAW = [-45, 45, 45, -45, 0, 0, 0, 0]
THRUSTER_PITCH = [0, 0, 0, 0, 90, 90, 90, 90]
THRUSTER_SURGE = [0.2921, 0.2921, -0.2921, -0.2921, 0.127, 0.127, -0.127, -0.127]
THRUSTER_SWAY = [26.67, -26.67, 26.67, -26.67, 26.67, -26.67, 26.67, -26.67]
THRUSTER_TRANSLATIONS = np.array([THRUSTER_SURGE, THRUSTER_SWAY, [0.] * 8])

THRUSTER_ROTATIONS = np.array([rotation_from_euler(yaw, pitch, 0.0, 'rzyx') for yaw, pitch in zip(THRUSTER_YAW, THRUSTER_PITCH)])
THRUSTER_FORCE = THRUSTER_ROTATIONS @ np.array([1., 0., 0.])[:, None, None]
THRUSTER_TORQUE = np.cross(THRUSTER_TRANSLATIONS, THRUSTER_FORCE)
