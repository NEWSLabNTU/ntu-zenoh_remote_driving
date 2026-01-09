# Zenoh Remote Driving

A remote driving demonstration system for the AutoSDV vehicle platform using Zenoh middleware. The system enables teleoperated control where an operator station sends commands and receives sensor data from a vehicle over the network.

## Architecture

The system splits execution between two machines:

- **Vehicle** (NVIDIA AGX Orin): Runs sensors (camera, LiDAR) and motor control
- **Operator**: Runs visualization (RViz2) and sends control commands via keyboard

Communication uses Zenoh-ROS2DDS bridge for transparent ROS 2 topic forwarding over TCP.

## Prerequisites

- **Hardware**: NVIDIA AGX Orin with JetPack 6.0
- **ROS 2 Humble**: [Installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- **Autoware 2025.02**: Install the [NEWSLAB Debian package](https://github.com/NEWSLabNTU/autoware/releases/tag/rosdebian%2F2025.02-1)
- **zenoh-bridge-ros2dds**: Install from [Zenoh releases](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds?tab=readme-ov-file#linux-debian)
- **AutoSDV**: Clone and build [AutoSDV](https://github.com/NEWSLabNTU/AutoSDV.git) at `~/repos/AutoSDV`

## Setup

Create the AutoSDV symlink (one-time):

```bash
ln -sfn ~/repos/AutoSDV script/AutoSDV
```

Use direnv for automatic environment setup:

```bash
direnv allow
```

Or source manually:

```bash
source ~/repos/AutoSDV/install/setup.bash
```

## Build

```bash
make build
```

To install Python dependencies (first time only):

```bash
make prepare
```

## Usage

### Network Configuration

Configure your network with these IP assignments:

| Machine  | IP Address      |
|----------|-----------------|
| Operator | 192.168.225.71  |
| Vehicle  | 192.168.225.73  |

### Running the System

If using direnv, the environment is set up automatically. Otherwise, source the workspace in every terminal.

**Step 1: Start the vehicle**

On the vehicle machine:

```bash
make run_vehicle
```

To specify a different operator IP:

```bash
make run_vehicle OPERATOR_IP=X.Y.Z.W
```

**Step 2: Start the operator**

On the operator machine:

```bash
make run_pilot
```

This opens RViz2 for visualization. Wait a few seconds for the point cloud and sensor data to appear.

**Step 3: Start the controller**

SSH into the vehicle from the operator machine and run the keyboard controller:

```bash
ssh jetson@192.168.225.73
make controller
```

Use the keyboard to control the vehicle:
- Arrow keys for steering and throttle
- Follow on-screen instructions for additional controls

## Project Structure

```
src/
  rdrive_launch/       # Launch files and RViz config
  remote_control/      # Vehicle motor control via PCA9685
  autoware_manual_control/  # Keyboard control (submodule)
  g923_control/        # Logitech G923 wheel controller (optional)
  remote_lidar/        # Open3D point cloud viewer (optional)
  ffmpeg/              # Video streaming proxy
config/                # Configuration files
```

## Make Targets

| Target             | Description                         |
|--------------------|-------------------------------------|
| `make build`       | Build the project                   |
| `make prepare`     | Install Python and ROS dependencies |
| `make run_vehicle` | Launch vehicle-side system          |
| `make run_pilot`   | Launch operator-side system         |
| `make controller`  | Run keyboard controller             |
| `make clean`       | Remove build artifacts              |

## Troubleshooting

- **No sensor data in RViz**: Check that Zenoh bridge is connected (verify network connectivity between machines)
- **Control not responding**: Ensure the controller is running on the vehicle and I2C bus 7 is accessible
- **Connection refused**: Verify IP addresses and that the operator is running before starting the vehicle

## Related Projects

- [AutoSDV](https://github.com/NEWSLabNTU/AutoSDV) - Autonomous driving platform (based on Autoware 2025.02)
- [Autoware](https://github.com/autowarefoundation/autoware) - Open-source autonomous driving stack
