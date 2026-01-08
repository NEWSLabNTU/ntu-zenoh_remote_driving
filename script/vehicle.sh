#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/AutoSDV/install/setup.bash"
source ./install/setup.bash
source /opt/ros/humble/setup.bash

# Init. tmux
tmux kill-server

# FFMPEG
tmux new -s bridge_ffmpeg -d
tmux send-keys -t bridge_ffmpeg "taskset --cpu-list 0-2 bash $HOME/zenoh_remote_driving/script/ffmpeg.sh" ENTER

# Autoware
tmux new -s bridge_autoware -d
tmux send-keys -t bridge_autoware "RUST_LOG=info zenoh-bridge-ros2dds -l tcp/${VEHICLE_IP}:8001" ENTER

tmux new -s autoware -d
tmux send-keys -t autoware "sleep 5 && taskset --cpu-list 3-11 ros2 launch autoware_launch autoware.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit" ENTER


# Control
export ROS_DOMAIN_ID=2
ros2 run remote_control remote_control -l tcp/${VEHICLE_IP}:8002 | tee log.txt
