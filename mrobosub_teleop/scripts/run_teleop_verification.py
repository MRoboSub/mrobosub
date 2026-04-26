#!/usr/bin/env python3
# Copyright 2025 Michigan Robotic Submarine
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Standalone verification script for teleop.

Runs unit tests and integration tests, then prints a summary.
Usage:
  cd /home/ubuntu/ros2_ws && source install/setup.bash
  python3 src/mrobosub_teleop/scripts/run_teleop_verification.py

Or via colcon test:
  colcon test --packages-select mrobosub_teleop --event-handlers console_direct+
"""

import os
import sys
import subprocess


def get_package_dir():
    """Return the mrobosub_teleop package directory."""
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def run_pytest(paths, extra_args=None):
    """Run pytest on specified test paths."""
    pkg_dir = get_package_dir()
    cmd = [sys.executable, "-m", "pytest", "-v", "--tb=short"]
    if isinstance(paths, str):
        paths = [paths]
    cmd.extend(paths)
    if extra_args:
        cmd.extend(extra_args)
    return subprocess.run(cmd, cwd=pkg_dir)


def main():
    print("=" * 60)
    print("mrobosub_teleop Verification")
    print("=" * 60)

    # Run unit tests first (no ROS required for parse_command tests)
    print("\n[1/2] Running unit tests (parse_command, DOF definitions)...")
    result = run_pytest("test/test_teleop_unit.py")
    if result.returncode != 0:
        print("\nFAILED: Unit tests did not pass.")
        return result.returncode

    # Run integration tests in subprocess for fresh rclpy context
    print("\n[2/2] Running integration tests (joystick teleop with mock Joy)...")
    result = run_pytest("test/test_joystick_integration.py")
    if result.returncode != 0:
        print("\nFAILED: Integration tests did not pass.")
        print("Make sure ROS2 is sourced: source install/setup.bash")
        return result.returncode

    print("\n" + "=" * 60)
    print("All tests PASSED.")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
