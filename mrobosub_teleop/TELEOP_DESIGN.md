# mrobosub Teleop Redesign

This document aligns the teleop with the new requirements and provides an implementation plan.

## Requirements Summary

| Requirement | Description |
|-------------|-------------|
| **PID control** | Yaw, Heave use PID; pitch/roll don't need control |
| **World-relative control** | Optional: surge/sway relative to zero yaw (world frame) |
| **Record/replay** | Record and replay teleop commands |
| **State machine integration** | Zeroing, resetting, soft stopping (same as current) |
| **New controller** | Physical joystick/gamepad support |
| **Estop** | Emergency stop integration |

---

## Current Architecture (What Exists)

### Topic Flow
```
Teleop (all_dof_teleop)  →  /target_twist/{dof}   →  GNC (passthrough: surge, sway)
                         →  /target_pose/{dof}    →  GNC (PID: heave, yaw, roll, pitch)
                         →  /output_wrench/{dof}  →  Thruster mixing → Motors
```

### GNC Controllers
- **Heave, Yaw, Roll, Pitch**: `pid_dof_controller` (target_pose + target_twist)
- **Surge, Sway**: `passthrough_dof_controller` (target_twist only)

### State Machine
- **Captain**: Runs state machine, publishes to same target topics
- **Soft stop**: `/captain/soft_stop` (Trigger) → transitions to Stop state
- **Stop state**: Calls `reset_target_twist()` 20 times

### Quartermaster
- **Zero state**: `/localization/zero_state` (Trigger) when starting
- **Soft stop**: Triggered by hall effect button on RUNNING → AMBIENT

### Estop
- **Thruster controller**: `/emergency_stop_motors` (SetBool) → cuts motor output

### Localization
- Publishes: `/pose/heave`, `/pose/yaw`, `/pose/pitch`, `/pose/roll`
- Does NOT publish `/pose/x`, `/pose/y` (no xy position yet)

---

## Proposed Architecture

### 1. Layered Design

```
┌─────────────────────────────────────────────────────────────────────────┐
│  INPUT LAYER                                                             │
│  joystick_teleop (Joy)  │  console_teleop (keyboard)  │  replay_teleop   │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  TELEOP CORE (shared)                                                    │
│  - Command generation (world-relative transform if enabled)                │
│  - Estop / soft stop handling                                            │
│  - Publishes to /target_twist/* and /target_pose/*                       │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  GNC (unchanged)                                                         │
│  PID: heave, yaw  │  Passthrough: surge, sway  │  (roll, pitch optional)  │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2. Module Structure

```
mrobosub_teleop/
├── mrobosub_teleop/
│   ├── __init__.py
│   ├── common.py              # DOF definitions, config, shared publishers
│   ├── teleop_base.py         # Base class with publish logic, estop handling
│   ├── joystick_teleop.py     # Button-driven: buttons + axis scale
│   ├── joystick_teleop_continuous.py  # Continuous: stick directly maps to velocity
│   ├── console_teleop.py      # Refactored keyboard input (debugging)
│   └── replay_teleop.py       # Optional: replay recorded commands
├── launch/
│   ├── joystick_teleop_launch.xml         # Button-driven
│   ├── joystick_teleop_continuous_launch.xml  # Continuous (stick = velocity)
│   └── console_teleop_launch.xml
├── params/
│   ├── joystick_controls.yaml   # Button-driven config
│   ├── joystick_continuous.yaml # Continuous mode: axis→twist, deadzone
│   └── joystick_mapping.yaml   # Legacy axis mapping
└── test/
```

### 3. Requirement Mapping

#### PID Control (Yaw, Heave)
- **Already done**: GNC uses PID for heave and yaw
- **Action**: Teleop continues publishing to `/target_pose/heave`, `/target_pose/yaw` for setpoints; `/target_twist/*` for direct override
- **Optional**: Remove or disable roll/pitch controllers if not needed (simplify GNC launch)

#### World-Relative Control
- **Input**: Joystick left stick = surge (forward), sway (strafe)
- **Body frame** (default): surge = forward in sub heading, sway = right in sub heading
- **World frame** (optional): surge = forward in world north (yaw=0), sway = right in world east
- **Implementation**: Subscribe to `/pose/yaw`, rotate joystick vector by `-yaw` before publishing:
  ```python
  world_surge = joy_surge * cos(yaw_rad) + joy_sway * sin(yaw_rad)
  world_sway = -joy_surge * sin(yaw_rad) + joy_sway * cos(yaw_rad)
  ```
- **Parameter**: `world_relative: bool` (default False)

#### Record/Replay
- **Where**: At teleop output level (record what we publish to target_*)
- **Rationale**: Lower level (e.g. output_wrench) would bypass PID; target_* is the right abstraction
- **Format**: Timestamped Float64 messages per topic, or a custom `TeleopCommand` msg
- **Replay**: Publish recorded commands at same rate; can run in loop or once

#### State Machine Integration
- **Same as current**: Teleop publishes to same topics as Captain
- **Zeroing**: Quartermaster calls `/localization/zero_state` on startup; teleop doesn't need to know
- **Reset**: When state machine stops, Captain calls `reset_target_twist()`; teleop should stop publishing or publish zeros when not actively controlling
- **Soft stop**: Teleop can subscribe to state topic or service; when soft stop received, stop publishing new commands (or publish zeros)

#### New Controller (Joystick)
- **Use**: `ros2 run joy joy_node` (or `joy_linux` package)
- **Topics**: Subscribe to `/joy` (sensor_msgs/msg/Joy)
- **Modes**:
  - **Continuous** (`joystick_teleop_continuous`): Push stick = move, release = stop. Config: `joystick_continuous.yaml`
  - **Button-driven** (`joystick_teleop`): Buttons trigger increase/decrease; axis provides scale. Config: `joystick_controls.yaml`

#### Estop
- **Option A**: Teleop subscribes to estop state; when true, publish zeros and ignore input
- **Option B**: Estop button on controller calls `/emergency_stop_motors` service
- **Option C**: Both: controller button triggers estop; teleop also subscribes to estop state to stop publishing

---

## Implementation Plan

### Phase 1: Refactor & Fix Bugs (Current Console Teleop)
1. Remove `from py_compile import main` (bug)
2. Fix radians/degrees handling in `all_dof_teleop.py`
3. Fix `cleanup()` and call it on exit
4. Extract shared logic into `common.py` and `teleop_base.py`
5. Improve readability: type hints, docstrings, clear function names

### Phase 2: Joystick Teleop
1. Add `joystick_teleop.py` that subscribes to `/joy`
2. Add `params/joystick_mapping.yaml` for axis/button mapping
3. Map axes to surge, sway, heave, yaw (with deadzone and scale)
4. Add launch file for joy_node + joystick_teleop

### Phase 3: World-Relative & Estop
1. Add `world_relative` parameter; subscribe to `/pose/yaw` when enabled
2. Add estop button mapping; call `/emergency_stop_motors` or subscribe to estop state
3. Add soft stop awareness (optional: subscribe to state topic)

### Phase 4: Record/Replay (Optional)
1. Add recording mode: log all target_* publishes with timestamps
2. Add replay node: read log and republish at recorded rate

### Phase 5: GNC Simplification (Optional)
1. Remove roll/pitch from launch if not needed
2. Document which DOFs are actively controlled

---

## Controller Mapping Suggestion

| Controller | Axis/Button | DOF | Notes |
|------------|-------------|-----|-------|
| Left stick X | Axis 0 | Sway | |
| Left stick Y | Axis 1 | Surge | Invert Y |
| Right stick X | Axis 2 | Yaw | |
| Right stick Y | Axis 3 | Heave | |
| L1 / RB | Button | Estop | Or soft stop |
| Start | Button | Zero state | Optional |

Make all mappings configurable via YAML.

---

## Dependencies

- `joy` (ROS2 joy package) for `joy_node`: `sudo apt install ros-<distro>-joy`
- `sensor_msgs` (already in package.xml)

---

## Quick Start

**Console teleop** (keyboard):
```bash
ros2 run mrobosub_teleop all_dof_teleop
# or
ros2 run mrobosub_teleop console_teleop
```

**Joystick teleop** (physical controller):
```bash
# Terminal 1: Start joy driver (install: sudo apt install ros-<distro>-joy)
ros2 run joy joy_node

# Terminal 2: Start joystick teleop
ros2 launch mrobosub_teleop joystick_teleop_launch.xml
```

## Testing Checklist

- [ ] Console teleop: publish commands, verify with `ros2 topic echo`
- [ ] Joystick teleop: run joy_node + joystick_teleop, verify axes map correctly
- [ ] World-relative: enable, rotate sub, verify surge/sway direction
- [ ] Estop: trigger, verify motors stop
- [ ] Soft stop: trigger from quartermaster, verify state machine stops
- [ ] State machine: run captain with Submerge, verify teleop works alongside (or in teleop-only state)
