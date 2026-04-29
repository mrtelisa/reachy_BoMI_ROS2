#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source ~/reachy_ws/install/setup.bash

WORLD_FILE=~/reachy_ws/src/reachy_bomi/worlds/base.world

ros2 launch reachy_bomi bomi_control.launch.py \
  world:=${WORLD_FILE}
