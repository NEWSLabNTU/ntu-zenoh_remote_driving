OPERATOR_IP := "192.168.225.71"

# Default recipe to show available commands
[private]
default:
    @just --list

# Install Python dependencies (first time only)
prepare:
    pip install Adafruit-PCA9685==1.0.1
    rosdep install -y --from-paths src --ignore-src --rosdistro humble

# Build the project
build:
    colcon build \
        --base-paths src \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Run pilot station (operator side)
run_pilot:
    #!/usr/bin/env bash
    export ROS_DOMAIN_ID=6
    . install/setup.sh
    ros2 launch rdrive_launch pilot.launch.yaml

# Run vehicle (on AGX Orin)
run_vehicle operator_ip=OPERATOR_IP:
    #!/usr/bin/env bash
    export ROS_DOMAIN_ID=9
    . install/setup.sh
    ros2 launch rdrive_launch vehicle.launch.yaml operator_ip:={{operator_ip}}

# Run keyboard controller
controller:
    #!/usr/bin/env bash
    . install/setup.sh
    ros2 run autoware_manual_control keyboard_control --ros-args --remap /external/selected/control_cmd:=/control/command/control_cmd

# Remove build artifacts
clean:
    rm -rf build install log
