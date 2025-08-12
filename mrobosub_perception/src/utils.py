from typing import Tuple
import numpy as np
import cv2

def pixels_to_angles(frame, x_pos: int, y_pos: int, fov_x=110, fov_y=70) -> Tuple[int, int]:
    height, width = frame.shape[0:2]
    d_x = x_pos - (width / 2)
    d_y = y_pos - (height / 2)
    theta_x = (d_x * fov_x) / width
    theta_y = (d_y * fov_y) / height
    return theta_x, theta_y

def crop_to_circle(image: np.ndarray, radius: int) -> np.ndarray:
    # Find the dimensions of the image
    h, w = image.shape[:2]

    # Create a black mask
    mask = np.zeros((h, w), dtype=np.uint8)

    # Find the center of the image
    cx, cy = w // 2, h // 2

    # Create a white circle
    c_image  = mask # happens in place
    c_center = (cx, cy)
    c_radius = radius
    c_color  = (255, 255, 255)
    c_thickness = -1 # -1 will fill the circle
    cv2.circle(c_image, c_center, c_radius, c_color, thickness=c_thickness)
   
    # bitwise AND the mask onto the original image
    result = cv2.bitwise_and(image, image, mask=mask)
    
    return result


def generate_rectify_maps(image: np.ndarray, f: int) -> Tuple[np.ndarray, np.ndarray]:
    # THe theory behind this function is that the image is distorted by a fisheye lens which produces
    # an orthogonal distortion. We need to undistort this raw image to get a rectified image.
    # See https://en.wikipedia.org/wiki/Fisheye_lens.

    # Also this just generates the maps, as the maps just rely on the image dimensions and the
    # f(ocal length), so they can be calculated ahead of time and be reused for every remap.
    
    h, w = image.shape[:2]
    cx, cy = w // 2, h // 2
   
    # allows us to vectorize our computations
    x_u, y_u = np.meshgrid(np.arange(w), np.arange(h))

    x_rel = x_u - cx
    y_rel = y_u - cy

    # calculate the distance r_u from the center from the image
    r_u = np.sqrt(x_rel**2 + y_rel**2)
    
    # the angle phi from the verticle
    phi = np.arctan2(y_rel, x_rel)

    # calculate theta from r_u and f
    theta = np.arctan(r_u / f)

    # Use the formula for the distorted radius for a orthogonal distortion
    r_d = f * np.sin(theta)

    # calculate the corresponding point in the distorted image in the distorted image
    x_d = cx + r_d * np.cos(phi)
    y_d = cy + r_d * np.sin(phi)

    map_x = x_d.astype(np.float32)
    map_y = y_d.astype(np.float32)

    return map_x, map_y