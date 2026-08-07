#!/bin/bash
# Launches the full multi-camera rig app (cameras + GUI + RViz) as one unit.
# Closing the GUI or RViz window shuts everything else down.
set -e

source /opt/ros/humble/setup.bash
source /home/hayden/cmu/kantor_lab/ros2_ws/install/setup.bash

exec ros2 launch multi_camera_rig_bringup app.launch.py
