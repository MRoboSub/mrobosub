from typing import Tuple


def pixels_to_angles(frame, x_pos: int, y_pos: int, fov_x=110, fov_y=70) -> Tuple[int, int]:
    height, width = frame.shape[0:2]
    d_x = x_pos - (width / 2)
    d_y = y_pos - (height / 2)
    theta_x = (d_x * fov_x) / width
    theta_y = (d_y * fov_y) / height
    return theta_x, theta_y