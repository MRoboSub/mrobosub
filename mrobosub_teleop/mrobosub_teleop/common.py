"""
Shared teleop definitions: DOF metadata and topic names.

Used by console teleop, joystick teleop, and replay.
"""

from dataclasses import dataclass, field
from typing import Dict


@dataclass
class DOF:
    """Degree of freedom metadata for teleop control."""

    name: str
    has_pose: bool  # Can accept target_pose setpoints (PID)
    is_angle: bool  # Uses radians/degrees

    short_name: str = field(init=False)

    def __post_init__(self) -> None:
        self.name = self.name.lower()
        self.short_name = self.name[:2]

    def __hash__(self) -> int:
        return hash(self.name)

    def print_usage(self) -> None:
        """Print command usage for this DOF."""
        parts = [self.name]
        if self.has_pose:
            parts.append("[twist|pose]")
        if self.is_angle:
            parts.append("[radians|degrees]")
        parts.append("value")
        print(f"{self.name} usage: {' '.join(parts)}")


# Heave: twist and pose (depth)
# Surge, sway: twist only (horizontal translation)
# Yaw, roll, pitch: twist and pose, angle units
ALL_DOFS_LIST = [
    DOF("heave", has_pose=True, is_angle=False),
    DOF("surge", has_pose=False, is_angle=False),
    DOF("sway", has_pose=False, is_angle=False),
    DOF("yaw", has_pose=True, is_angle=True),
    DOF("roll", has_pose=True, is_angle=True),
    DOF("pitch", has_pose=True, is_angle=True),
]

ALL_DOFS: Dict[str, DOF] = {dof.short_name: dof for dof in ALL_DOFS_LIST}

# DOFs that use PID (per requirements: Yaw, Heave; pitch/roll optional)
PID_DOFS = {"heave", "yaw"}

# DOFs that use passthrough (surge, sway)
PASSTHROUGH_DOFS = {"surge", "sway"}
