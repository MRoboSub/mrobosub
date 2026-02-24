"""
Base teleop node: publishers for target_pose and target_twist.

Shared by console, joystick, and replay teleop nodes.
"""

from std_msgs.msg import Float64

from mrobosub_lib import Node

from .common import ALL_DOFS


class TeleopBase(Node):
    """
    Base class for teleop nodes.

    Publishes to:
    - /target_twist/{dof} for all DOFs
    - /target_pose/{dof} for heave, yaw, roll, pitch
    """

    def __init__(self, node_name: str = "teleop_base") -> None:
        super().__init__(node_name)
        self._dof_publishers: dict = {}
        for dof in ALL_DOFS.values():
            dof_pubs = self._dof_publishers[dof.short_name] = {}
            dof_pubs["twist"] = self.create_publisher(
                Float64, f"/target_twist/{dof.name}", qos_profile=1
            )
            if dof.has_pose:
                dof_pubs["pose"] = self.create_publisher(
                    Float64, f"/target_pose/{dof.name}", qos_profile=1
                )

    def publish_twist(self, dof_short_name: str, value: float) -> None:
        """Publish a twist (velocity) command for the given DOF."""
        if dof_short_name in self._dof_publishers:
            self._dof_publishers[dof_short_name]["twist"].publish(Float64(data=float(value)))

    def publish_pose(self, dof_short_name: str, value: float) -> None:
        """Publish a pose (position) setpoint for the given DOF."""
        if dof_short_name in self._dof_publishers:
            pubs = self._dof_publishers[dof_short_name]
            if "pose" in pubs:
                pubs["pose"].publish(Float64(data=float(value)))

    def reset_all_twist(self) -> None:
        """Publish zero to all twist topics (soft stop / reset)."""
        for dof_short_name in self._dof_publishers:
            self.publish_twist(dof_short_name, 0.0)
