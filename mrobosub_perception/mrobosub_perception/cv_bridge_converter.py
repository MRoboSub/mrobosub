import numpy as np
import cv2
from sensor_msgs.msg import Image

# Maps ROS encoding strings -> (numpy dtype, channels)
ENCODING_TO_DTYPE_CHANNELS = {
    "rgb8":    (np.uint8, 3),
    "bgr8":    (np.uint8, 3),
    "rgba8":   (np.uint8, 4),
    "bgra8":   (np.uint8, 4),
    "mono8":   (np.uint8, 1),
    "8UC1":    (np.uint8, 1),
    "mono16":  (np.uint16, 1),
    "16UC1":   (np.uint16, 1),
    "32FC1":   (np.float32, 1),
}

# cv2.cvtColor codes for converting between common encodings
_CVT_CODES = {
    ("rgb8", "bgr8"): cv2.COLOR_RGB2BGR,
    ("bgr8", "rgb8"): cv2.COLOR_BGR2RGB,
    ("rgb8", "mono8"): cv2.COLOR_RGB2GRAY,
    ("bgr8", "mono8"): cv2.COLOR_BGR2GRAY,
    ("mono8", "rgb8"): cv2.COLOR_GRAY2RGB,
    ("mono8", "bgr8"): cv2.COLOR_GRAY2BGR,
    ("rgba8", "bgr8"): cv2.COLOR_RGBA2BGR,
    ("bgra8", "rgb8"): cv2.COLOR_BGRA2RGB,
    ("rgb8", "rgba8"): cv2.COLOR_RGB2RGBA,
    ("bgr8", "bgra8"): cv2.COLOR_BGR2BGRA,
}


def imgmsg_to_cv2(msg: Image, desired_encoding: str = None) -> np.ndarray:
    """Convert a sensor_msgs/Image to an OpenCV (numpy) image, no cv_bridge."""
    if msg.encoding not in ENCODING_TO_DTYPE_CHANNELS:
        raise ValueError(f"Unsupported encoding: {msg.encoding}")

    dtype, channels = ENCODING_TO_DTYPE_CHANNELS[msg.encoding]
    dtype = np.dtype(dtype)
    if msg.is_bigendian:
        dtype = dtype.newbyteorder(">")

    img = np.frombuffer(msg.data, dtype=dtype)

    if channels == 1:
        img = img.reshape(msg.height, msg.width)
    else:
        img = img.reshape(msg.height, msg.width, channels)

    # Handle row padding (step != width * bytes_per_pixel)
    expected_step = msg.width * channels * dtype.itemsize
    if msg.step != expected_step:
        # reshape accounting for stride, then slice off padding
        img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.step // dtype.itemsize)
        img = img[:, :msg.width * channels]
        img = img.reshape(msg.height, msg.width, channels) if channels > 1 else img.reshape(msg.height, msg.width)

    if desired_encoding is None or desired_encoding == msg.encoding:
        return img

    key = (msg.encoding, desired_encoding)
    if key not in _CVT_CODES:
        raise ValueError(f"No conversion path from {msg.encoding} to {desired_encoding}")

    return cv2.cvtColor(img, _CVT_CODES[key])


def cv2_to_imgmsg(cv_image: np.ndarray, encoding: str = "bgr8", header=None) -> Image:
    """Convert an OpenCV (numpy) image to a sensor_msgs/Image, no cv_bridge."""
    if encoding not in ENCODING_TO_DTYPE_CHANNELS:
        raise ValueError(f"Unsupported encoding: {encoding}")

    dtype, expected_channels = ENCODING_TO_DTYPE_CHANNELS[encoding]
    dtype = np.dtype(dtype)

    if cv_image.dtype != dtype:
        raise ValueError(f"Image dtype {cv_image.dtype} doesn't match encoding {encoding} (expects {dtype})")

    if cv_image.ndim == 2:
        actual_channels = 1
        height, width = cv_image.shape
    else:
        height, width, actual_channels = cv_image.shape

    if actual_channels != expected_channels:
        raise ValueError(f"Image has {actual_channels} channels, encoding {encoding} expects {expected_channels}")

    msg = Image()
    if header is not None:
        msg.header = header
    msg.height = height
    msg.width = width
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = width * expected_channels * dtype.itemsize
    msg.data = np.ascontiguousarray(cv_image).tobytes()
    return msg