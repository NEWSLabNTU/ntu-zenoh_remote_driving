# CLAUDE.md

## Project Overview

This is a **remote driving demonstration system** that enables teleoperated control of an AutoSDV vehicle using Zenoh middleware. The project splits execution between a vehicle (NVIDIA AGX Orin) and an operator station, communicating over a network.

**Key Technologies:**
- **Zenoh** - Distributed middleware for real-time ROS 2 communication
- **ROS 2 Humble** - Robot Operating System
- **Autoware 2025.02** - Autonomous driving stack (NEWSLAB variant, sensing-only mode)
- **AutoSDV** - 1/8-scale vehicle platform

## Architecture

```
Operator Station                          Vehicle (AGX Orin)
┌─────────────────────┐                  ┌─────────────────────┐
│ Keyboard Control    │                  │ Sensors             │
│ (autoware_manual_   │                  │ - Camera            │
│  control)           │                  │ - LiDAR             │
│         │           │                  │         │           │
│         ▼           │                  │         ▼           │
│ zenoh-bridge-ros2dds│◄────Zenoh───────►│ zenoh-bridge-ros2dds│
│ (server mode)       │    TCP:8001      │ (client mode)       │
│         │           │                  │         │           │
│         ▼           │                  │         ▼           │
│ RViz2 Visualization │                  │ remote_control      │
│ (point cloud, video)│                  │ (PCA9685 → motors)  │
└─────────────────────┘                  └─────────────────────┘
```

**Data Flow:**
- Control commands: Operator → Zenoh → Vehicle → PCA9685 PWM → Motors
- Sensor data: Vehicle sensors → Zenoh → Operator RViz2

## Environment Setup

The environment is managed via direnv. The `script/AutoSDV` symlink points to the AutoSDV installation with pre-built binaries.

```bash
# First-time setup: create symlink and allow direnv
ln -sfn /path/to/AutoSDV script/AutoSDV
direnv allow
```

The `.envrc` automatically sources:
- `script/AutoSDV/install/setup.bash` (chains to ROS 2 Humble)
- `install/setup.bash` (this project's workspace, when built)

## Build Commands

```bash
just build      # Build the project
just prepare    # Install Python dependencies (first time only)
just clean      # Remove build artifacts
```

## Running the System

```bash
# Step 1: On vehicle
just run_vehicle operator_ip=192.168.225.71

# Step 2: On operator machine
just run_pilot

# Step 3: On vehicle (SSH from operator)
ssh jetson@192.168.225.73
just controller
```

## Key File Locations

### Launch Files
- `src/rdrive_launch/launch/vehicle.launch.yaml` - Vehicle-side launch (sensors + Zenoh bridge)
- `src/rdrive_launch/launch/pilot.launch.yaml` - Operator-side launch (RViz + Zenoh bridge)
- `src/rdrive_launch/rviz/pilot.rviz` - RViz visualization config

### ROS 2 Packages
- `src/remote_control/` - Receives control commands via Zenoh, drives PCA9685 PWM
- `src/g923_control/` - Logitech G923 racing wheel controller (optional)
- `src/remote_lidar/` - Open3D point cloud visualization (alternative to RViz)
- `src/remote_camera/` - Camera publisher (legacy)
- `src/autoware_manual_control/` - Keyboard control (git submodule)
- `src/ffmpeg/` - Video streaming proxy (C++)

### Configuration
- `.envrc` - direnv environment setup
- `script/AutoSDV` - Symlink to AutoSDV installation
- `config/ScreenCamera_position.json` - Open3D camera view preset
- `justfile` - Build and run targets

## Hardware Configuration

**PCA9685 PWM Controller** (I2C bus 7, address 0x40):
- Channel 0: DC motor (ESC) - PWM range 180-580, neutral 380
- Channel 1: Servo (steering) - PWM range 260-500, neutral 380

## Network Configuration

Default IP assignments:
- Operator: 192.168.225.71
- Vehicle: 192.168.225.73

Ports:
- 8001: Zenoh DDS bridge
- 8003: FFmpeg video stream input
- 8080: FFmpeg video stream client

ROS Domain IDs:
- Operator: ROS_DOMAIN_ID=6
- Vehicle: ROS_DOMAIN_ID=9

## Related Projects

- **AutoSDV** (`script/AutoSDV` → `~/repos/AutoSDV`) - Vehicle platform with Autoware integration
- **Autoware 2025.02** (`~/repos/autoware/2025.02`) - Underlying autonomous driving stack

This project uses Autoware in **sensing-only mode** - planning, perception, and control modules are disabled on the vehicle since commands come from the remote operator.

## Development Tips

1. **Modifying launch parameters**: Edit YAML files in `src/rdrive_launch/launch/`
2. **Testing control locally**: Use `ros2 topic pub` to send AckermannControlCommand
3. **Debugging Zenoh**: Check Zenoh bridge logs on both sides for connection issues
4. **Video streaming issues**: Ensure FFmpeg proxy is running if using video stream

## Common Issues

- **No point cloud in RViz**: Check Zenoh bridge connection, verify LiDAR driver running on vehicle
- **Control latency**: Monitor latency printed by remote_control node
- **PWM not responding**: Verify I2C bus 7 access permissions on AGX Orin
