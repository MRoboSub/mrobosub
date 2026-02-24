
"""Unit tests for teleop command parsing and DOF logic."""

import math
import pytest

from mrobosub_teleop.console_teleop import parse_command
from mrobosub_teleop.common import ALL_DOFS, ALL_DOFS_LIST


class TestParseCommandSurgeSway:
    """Surge and sway: twist only, single value."""

    def test_surge_twist_valid(self):
        result = parse_command(["0.5"], ALL_DOFS["su"])
        assert result == ("twist", 0.5)

    def test_surge_twist_negative(self):
        result = parse_command(["-0.3"], ALL_DOFS["su"])
        assert result == ("twist", -0.3)

    def test_sway_twist_valid(self):
        result = parse_command(["0.2"], ALL_DOFS["sw"])
        assert result == ("twist", 0.2)

    def test_surge_empty_args(self):
        result = parse_command([], ALL_DOFS["su"])
        assert result is None

    def test_surge_invalid_value(self):
        result = parse_command(["not_a_number"], ALL_DOFS["su"])
        assert result is None


class TestParseCommandHeave:
    """Heave: twist and pose, no angle units."""

    def test_heave_twist(self):
        result = parse_command(["twist", "0.5"], ALL_DOFS["he"])
        assert result == ("twist", 0.5)

    def test_heave_pose(self):
        result = parse_command(["pose", "1.0"], ALL_DOFS["he"])
        assert result == ("pose", 1.0)

    def test_heave_twist_partial_match(self):
        result = parse_command(["t", "0.1"], ALL_DOFS["he"])
        assert result == ("twist", 0.1)

    def test_heave_pose_partial_match(self):
        result = parse_command(["p", "2.0"], ALL_DOFS["he"])
        assert result == ("pose", 2.0)

    def test_heave_missing_value(self):
        result = parse_command(["twist"], ALL_DOFS["he"])
        assert result is None


class TestParseCommandYawRollPitch:
    """Yaw, roll, pitch: twist/pose + radians/degrees."""

    def test_yaw_twist_degrees(self):
        result = parse_command(["twist", "degrees", "45"], ALL_DOFS["ya"])
        assert result == ("twist", 45.0)

    def test_yaw_twist_radians(self):
        result = parse_command(["twist", "radians", "0.5"], ALL_DOFS["ya"])
        expected_deg = math.degrees(0.5)
        assert result == ("twist", pytest.approx(expected_deg, rel=1e-5))

    def test_yaw_pose_degrees(self):
        result = parse_command(["pose", "degrees", "90"], ALL_DOFS["ya"])
        assert result == ("pose", 90.0)

    def test_yaw_pose_radians(self):
        result = parse_command(["pose", "r", "1.57"], ALL_DOFS["ya"])
        assert result == ("pose", pytest.approx(math.degrees(1.57), rel=1e-5))

    def test_yaw_missing_unit_or_value(self):
        assert parse_command(["twist", "45"], ALL_DOFS["ya"]) is None
        assert parse_command(["twist", "degrees"], ALL_DOFS["ya"]) is None

    def test_yaw_invalid_value(self):
        result = parse_command(["twist", "degrees", "x"], ALL_DOFS["ya"])
        assert result is None


class TestCommon:
    """DOF definitions and constants."""

    def test_all_dofs_have_short_names(self):
        for dof in ALL_DOFS_LIST:
            assert len(dof.short_name) == 2
            assert dof.short_name == dof.name[:2]

    def test_surge_sway_no_pose(self):
        assert ALL_DOFS["su"].has_pose is False
        assert ALL_DOFS["sw"].has_pose is False

    def test_heave_yaw_have_pose(self):
        assert ALL_DOFS["he"].has_pose is True
        assert ALL_DOFS["ya"].has_pose is True

    def test_angle_dofs(self):
        assert ALL_DOFS["ya"].is_angle is True
        assert ALL_DOFS["ro"].is_angle is True
        assert ALL_DOFS["pi"].is_angle is True
        assert ALL_DOFS["su"].is_angle is False
