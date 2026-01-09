OPERATOR_IP := "192.168.225.71"
PROJECT_DIR := justfile_directory()

# Default recipe to show available commands
[private]
default:
    @just --list

# ============================================================================
# Setup and Build
# ============================================================================

# Install Python dependencies (first time only)
setup:
    pip install Adafruit-PCA9685==1.0.1
    rosdep install -y --from-paths src --ignore-src --rosdistro humble

# Build the project
build:
    colcon build \
        --base-paths src \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# ============================================================================
# rmw_zenoh Operations
# ============================================================================

# Terminate old ROS 2 daemon (run before switching RMW implementations)
reset-daemon:
    #!/usr/bin/env bash
    echo "Terminating ROS 2 daemon..."
    pkill -9 -f ros2 || true
    ros2 daemon stop || true
    echo "Done. You may now start the Zenoh router."

# Start Zenoh router (run on operator station before run_pilot)
router:
    #!/usr/bin/env bash
    echo "Starting Zenoh router..."
    echo "Config: ${ZENOH_ROUTER_CONFIG_URI}"
    ros2 run rmw_zenoh_cpp rmw_zenohd

# ============================================================================
# Remote Driving Operations
# ============================================================================

# Run pilot station (operator side) - requires router to be running
run_pilot:
    #!/usr/bin/env bash
    . install/setup.sh
    ros2 launch rdrive_launch pilot.launch.yaml

# Run vehicle (on AGX Orin) - connects to operator's Zenoh router
run_vehicle operator_ip=OPERATOR_IP:
    #!/usr/bin/env bash
    . install/setup.sh
    # Use vehicle session config (client mode) connecting to operator
    export ZENOH_SESSION_CONFIG_URI="{{PROJECT_DIR}}/config/zenoh/session_vehicle.json5"
    # Override the connection endpoint with the specified operator IP
    export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/{{operator_ip}}:7447"]'
    # Skip local router check since we're connecting to remote router
    export ZENOH_ROUTER_CHECK_ATTEMPTS=-1
    ros2 launch rdrive_launch vehicle.launch.yaml

# Run keyboard controller
controller:
    #!/usr/bin/env bash
    . install/setup.sh
    ros2 run autoware_manual_control keyboard_control --ros-args --remap /external/selected/control_cmd:=/control/command/control_cmd

# ============================================================================
# Maintenance
# ============================================================================

# Remove build artifacts
clean:
    rm -rf build install log
