# Multi-Camera Rig Bringup

This package provides unified launch files for bringing up complete multi-camera rig systems with cameras, detection, and reconstruction capabilities.

## Overview

The bringup package orchestrates the entire multi-camera rig pipeline by launching:
1. **Camera drivers** - Raw image acquisition from hardware or simulation
2. **Detection** - Object detection using YOLOv8 with TensorRT
3. **Reconstruction** - Stereo matching and 3D point cloud generation

## Architecture

The package follows a modular architecture:
- **Camera-specific bringup packages** (e.g., `firefly-ros2-wrapper-bringup`) handle only camera image acquisition
- **`multi_camera_rig_detection`** provides generalizable detection nodes
- **`multi_camera_rig_reconstruction`** provides generalizable reconstruction nodes
- **`multi_camera_rig_bringup`** orchestrates the complete pipeline

## Launch Files

### firefly_bringup.launch.py

Main launch file for Firefly stereo camera system with detection and reconstruction.

**Key Parameters:**
- `use_gazebo` - Switch between simulation (true) and real hardware (false)
- `enable_detection` - Enable/disable YOLOv8 object detection
- `use_semantics` - Enable semantic point cloud generation

**Usage:**

Real hardware:
```bash
ros2 launch multi_camera_rig_bringup firefly_bringup.launch.py use_gazebo:=false
```

Simulation:
```bash
ros2 launch multi_camera_rig_bringup firefly_bringup.launch.py use_gazebo:=true
```

With RViz visualization:
```bash
ros2 launch multi_camera_rig_bringup firefly_bringup.launch.py use_rviz:=true
```

## Dependencies

- `firefly-ros2-wrapper-bringup` - Firefly camera drivers
- `multi_camera_rig_detection` - Detection nodes
- `multi_camera_rig_reconstruction` - Reconstruction nodes
- `multi_camera_rig_trigger` - Hardware trigger control

## Topics

### Published Topics
- `/firefly_left/image_raw` - Raw left camera image
- `/firefly_right/image_raw` - Raw right camera image
- `/firefly_left/image_rect` - Rectified left image
- `/firefly_left/image_rect_scaled` - Rectified and scaled left image
- `/firefly_left/detections` - YOLO detections
- `/firefly_left/disparity` - Disparity map
- `/firefly_left/points2` - Semantic point cloud

## Configuration

Model paths and parameters can be configured via launch arguments. See launch file for complete list of parameters.

## Utility Nodes

The package includes several utility nodes for common tasks:

### qos_republisher_node
Republishes messages with different QoS settings (e.g., reliable → best_effort).

**Example:**
```bash
ros2 run multi_camera_rig_bringup qos_republisher_node --ros-args \
  -p type:=sensor_msgs/msg/Image \
  -p in_topic:=/camera/image_raw \
  -p out_topic:=/camera/image_raw_be \
  -p sub_qos.reliability:=reliable \
  -p pub_qos.reliability:=best_effort
```

### image_saver_node
Saves images from a topic to disk with timestamps.

**Example:**
```bash
ros2 run multi_camera_rig_bringup image_saver_node --ros-args \
  -p image_topic:=/firefly_left/image_rect \
  -p save_directory:=/home/user/saved_images \
  -p image_format:=png
```

Ground-truth generation now lives outside this package; `multi_camera_rig_bringup` only handles sensing, detection, and reconstruction.
