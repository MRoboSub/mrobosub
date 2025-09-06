import rclpy


def thruster_mixing():
    from .thruster_mixing import ThrusterMixing

    rclpy.init()

    node = ThrusterMixing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
