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
│   rmw_zenoh_cpp     │                  │   rmw_zenoh_cpp     │
│   (peer mode)       │                  │   (client mode)     │
│         │           │                  │         │           │
│         ▼           │                  │         │           │
│   Zenoh Router      │◄────Zenoh───────►│   (connects to      │
│   (rmw_zenohd)      │    TCP:7447      │    operator router) │
│         │           │                  │         │           │
│         ▼           │                  │         ▼           │
│ RViz2 Visualization │                  │ remote_control      │
│ (point cloud, video)│                  │ (PCA9685 → motors)  │
└─────────────────────┘                  └─────────────────────┘
```

**Data Flow:**
- Control commands: Operator → rmw_zenoh → Zenoh Router → Vehicle → PCA9685 PWM → Motors
- Sensor data: Vehicle sensors → rmw_zenoh → Zenoh Router → Operator RViz2

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

And sets rmw_zenoh environment variables:
- `RMW_IMPLEMENTATION=rmw_zenoh_cpp`
- `ZENOH_ROUTER_CONFIG_URI` - Path to router config
- `ZENOH_SESSION_CONFIG_URI` - Path to session config (local by default)

## Build Commands

```bash
just build      # Build the project
just setup      # Install Python dependencies (first time only)
just clean      # Remove build artifacts
```

## Running the System

```bash
# Step 0 (optional): Reset ROS 2 daemon if switching RMW implementations
just reset-daemon

# Step 1: On operator machine - Start Zenoh router (keep running)
just router

# Step 2: On operator machine - Start pilot station (new terminal)
just run_pilot

# Step 3: On vehicle - Start vehicle nodes
just run_vehicle operator_ip=192.168.225.71

# Step 4: On operator machine - Run keyboard controller
just controller
```

## Key File Locations

### Launch Files
- `src/rdrive_launch/launch/vehicle.launch.yaml` - Vehicle-side launch (Autoware sensing)
- `src/rdrive_launch/launch/pilot.launch.yaml` - Operator-side launch (RViz visualization)
- `src/rdrive_launch/rviz/pilot.rviz` - RViz visualization config

### ROS 2 Packages
- `src/remote_control/` - Receives control commands via Zenoh, drives PCA9685 PWM
- `src/g923_control/` - Logitech G923 racing wheel controller (optional)
- `src/remote_lidar/` - Open3D point cloud visualization (alternative to RViz)
- `src/remote_camera/` - Camera publisher (legacy)
- `src/autoware_manual_control/` - Keyboard control (git submodule)
- `src/ffmpeg/` - Video streaming proxy (C++)

### Configuration
- `.envrc` - direnv environment setup (sets RMW_IMPLEMENTATION and Zenoh config paths)
- `script/AutoSDV` - Symlink to AutoSDV installation
- `justfile` - Build and run targets

### Zenoh Configuration (`config/zenoh/`)
- `router.json5` - Zenoh router config (operator station, listens on TCP:7447)
- `session_local.json5` - Session config for local nodes (peer mode, connects to local router)
- `session_vehicle.json5` - Session config for vehicle (client mode, connects to operator router)

### Other Config
- `config/ScreenCamera_position.json` - Open3D camera view preset

## Hardware Configuration

**PCA9685 PWM Controller** (I2C bus 7, address 0x40):
- Channel 0: DC motor (ESC) - PWM range 180-580, neutral 380
- Channel 1: Servo (steering) - PWM range 260-500, neutral 380

## Network Configuration

Default IP assignments:
- Operator: 192.168.225.71
- Vehicle: 192.168.225.73

Ports:
- 7447: Zenoh router (rmw_zenoh)
- 8003: FFmpeg video stream input
- 8080: FFmpeg video stream client

**Note:** With rmw_zenoh, ROS_DOMAIN_ID is not used. Communication isolation is handled via Zenoh router connectivity.

## Related Projects

- **AutoSDV** (`script/AutoSDV` → `~/repos/AutoSDV`) - Vehicle platform with Autoware integration
- **Autoware 2025.02** (`~/repos/autoware/2025.02`) - Underlying autonomous driving stack

This project uses Autoware in **sensing-only mode** - planning, perception, and control modules are disabled on the vehicle since commands come from the remote operator.

## Development Tips

1. **Modifying launch parameters**: Edit YAML files in `src/rdrive_launch/launch/`
2. **Testing control locally**: Use `ros2 topic pub` to send AckermannControlCommand
3. **Debugging Zenoh**: Use `RUST_LOG=zenoh=info` for router/session logs
4. **Video streaming issues**: Ensure FFmpeg proxy is running if using video stream
5. **Changing operator IP**: Override via `ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/NEW_IP:7447"]'`

## Common Issues

- **No point cloud in RViz**: Check Zenoh router is running on operator, verify vehicle can connect
- **ROS 2 CLI not working**: Run `just reset-daemon` to restart ROS 2 daemon with rmw_zenoh
- **Vehicle can't connect**: Verify operator IP in `just run_vehicle operator_ip=...` and firewall allows TCP:7447
- **Control latency**: Monitor latency printed by remote_control node
- **PWM not responding**: Verify I2C bus 7 access permissions on AGX Orin
- **Router won't start (IPv6 error)**: Our config uses IPv4 (`tcp/0.0.0.0:7447`) which should work; check config file path
