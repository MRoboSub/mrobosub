"""
Console-based teleop: text commands via keyboard input.

Useful for debugging and when no physical controller is available.
"""

from math import degrees
import rclpy

from .common import ALL_DOFS, DOF
from .teleop_base import TeleopBase


def parse_command(args: list[str], dof: DOF) -> tuple[str, float] | None:
    """
    Parse command args into (mode, value).

    Returns (mode, value) or None if invalid.
    mode is "twist" or "pose".
    value is always in the units expected by the topic (degrees for angles).

    Formats:
    - surge/sway (twist only): "surge 0.5"
    - heave (twist+pose): "heave twist 0.5" or "heave pose 1.0"
    - yaw/roll/pitch (twist+pose, angles): "yaw twist degrees 45" or "yaw pose radians 0.78"
    """
    if len(args) < 1:
        return None

    mode = "twist"
    value_str: str
    idx = 0

    # Handle twist|pose for DOFs that support both
    if dof.has_pose:
        if args[0].lower().startswith("p"):
            mode = "pose"
        idx = 1
        if len(args) <= idx:
            return None

    # Handle radians|degrees for angle DOFs
    if dof.is_angle:
        if len(args) < idx + 2:
            return None
        unit, value_str = args[idx].lower(), args[idx + 1]
        try:
            value_float = float(value_str)
        except ValueError:
            return None
        if unit.startswith("r"):  # radians -> degrees
            value_float = degrees(value_float)
        return (mode, value_float)

    # Non-angle: single value
    value_str = args[idx]
    try:
        value = float(value_str)
    except ValueError:
        return None
    return (mode, value)


def main() -> None:
    rclpy.init()
    node = TeleopBase("console_teleop")

    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0)
            command = input(
                "Enter command: dof [twist|pose] [radians|degrees] value (q to quit)\n"
            )
            parts = command.strip().split()
            if not parts or parts[0][:1].lower() == "q":
                print("Quitting!")
                break

            dof_key = parts[0][:2].lower()
            try:
                dof = ALL_DOFS[dof_key]
            except KeyError:
                print(f'Unknown DOF "{dof_key}". Options: {list(ALL_DOFS.keys())}')
                continue

            result = parse_command(parts[1:], dof)
            if result is None:
                dof.print_usage()
                continue

            mode, value = result
            print(f"Command: {dof.name} {mode} {value:.2f}")

            if mode == "twist":
                node.publish_twist(dof.short_name, value)
            else:
                node.publish_pose(dof.short_name, value)

    except KeyboardInterrupt:
        pass
    finally:
        node.reset_all_twist()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
