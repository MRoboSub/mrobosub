# Teleop Testing Guide

This document explains how to run the teleop verification tests and interpret the results.

## Quick Start

```bash
cd /home/ubuntu/ros2_ws
source install/setup.bash
python3 src/mrobosub_teleop/scripts/run_teleop_verification.py
```

Or run tests directly with pytest:

```bash
cd /home/ubuntu/ros2_ws
source install/setup.bash
cd src/mrobosub_teleop
python3 -m pytest test/test_teleop_unit.py test/test_joystick_integration.py -v
```

## What the Tests Verify

### Unit Tests (`test_teleop_unit.py`)

| Test Class | What It Verifies |
|------------|------------------|
| **TestParseCommandSurgeSway** | Surge/sway commands parse correctly (`surge 0.5`, `sway -0.3`); invalid input rejected |
| **TestParseCommandHeave** | Heave twist/pose parsing (`heave twist 0.5`, `heave pose 1.0`) |
| **TestParseCommandYawRollPitch** | Angle DOFs with degrees/radians (`yaw twist degrees 45`, `yaw pose radians 0.78`) |
| **TestCommon** | DOF definitions (short names, has_pose, is_angle) |

### Integration Tests (`test_joystick_integration.py`)

**Button-driven mode** (joystick_teleop):
| Test | What It Verifies |
|------|------------------|
| **test_joystick_teleop_button_increase** | surge.increase button + axis 0.6 produces target_twist/surge ≈ 0.3 |
| **test_joystick_teleop_estop_zeros_output** | Estop button zeros all target_twist outputs |
| **test_joystick_teleop_rising_edge_only** | Button with zero axis produces zero output |

**Continuous mode** (joystick_teleop_continuous):
| Test | What It Verifies |
|------|------------------|
| **test_joystick_teleop_continuous_surge_output** | Axis 1 = 0.6 maps directly to target_twist/surge ≈ 0.3 |
| **test_joystick_teleop_continuous_deadzone** | Input 0.05 (below deadzone 0.1) produces zero output |
| **test_joystick_teleop_continuous_estop** | Estop button zeros all outputs |

## Interpreting Results

### All Pass

```
============================== 23 passed in 0.63s ==============================
```

- **Console teleop** command parsing works for all DOF types.
- **Joystick teleop** (button-driven and continuous) correctly map Joy to target_twist; estop zeros output.

### Unit Test Failures

- **Parse failures**: Check `console_teleop.parse_command` and supported command formats.
- **DOF failures**: Check `common.py` DOF definitions.

### Integration Test Failures

- **Button mode**: Check `joystick_controls.yaml` and button/axis mappings.
- **Continuous mode**: Check `joystick_continuous.yaml`; axis mapping, scale, deadzone (0.1), estop button (4).

### rclpy Shutdown Warnings

Messages like `cannot use Destroyable because destruction was requested` at the end of a run are common with rclpy in tests and can be ignored.

## Running via colcon test

```bash
colcon test --packages-select mrobosub_teleop --event-handlers console_direct+
```

This runs all package tests including flake8, pep257, and copyright. For teleop-only verification, use the verification script or pytest commands above.
