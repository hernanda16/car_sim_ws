#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_PATH="$SCRIPT_DIR/src/towing/models"
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$MODEL_PATH
echo "GAZEBO_MODEL_PATH = $GAZEBO_MODEL_PATH"

source install/setup.bash
ros2 launch towing ignition.launch.py --debug