# Multi Camera Rig V3

This repository contains the ROS2 packages for the multi-camera rig system.

## Repository Structure

- `multi_camera_rig_description/` - URDF and visualization configuration for the multi-camera rig
- `camera_ros2_wrappers/` - ROS2 wrapper packages for various cameras
  - `firefly-ros2-wrapper/` - FLIR Firefly camera wrapper
  - `ximea-ros2-wrapper/` - Ximea camera wrapper
  - `zed-ros2-wrapper/` - Stereolabs ZED camera wrapper (submodule)

## Git Submodules

This repository uses git submodules to manage external dependencies:
- **zed-ros2-wrapper**: [stereolabs/zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
- **flir_camera_driver**: [ros-drivers/flir_camera_driver](https://github.com/ros-drivers/flir_camera_driver)

### Cloning this Repository

When cloning this repository for the first time, use:

```bash
git clone --recurse-submodules <your-repo-url>
```

Or if you've already cloned without submodules:

```bash
git clone <your-repo-url>
cd multi_camera_rig_v3
git submodule update --init --recursive
```

### Updating Submodules

To update the submodules to their latest versions:

```bash
# Update all submodules to the latest commit on their tracked branch
git submodule update --remote

# Or update a specific submodule
git submodule update --remote camera_ros2_wrappers/zed-ros2-wrapper
git submodule update --remote camera_ros2_wrappers/firefly-ros2-wrapper/flir_camera_driver

# After updating, commit the changes
git add .
git commit -m "Update submodules"
git push
```

### Working with Submodules

To pull the latest changes including submodules:

```bash
git pull --recurse-submodules
```

## Building

Build the packages using colcon:

```bash
cd /path/to/ros2_ws
colcon build --packages-select multi_camera_rig_description firefly-ros2-wrapper-bringup firefly-ros2-wrapper-description ximea-ros2-wrapper-bringup ximea-ros2-wrapper-description
```

## Setup Instructions

1. Clone this repository into your ROS2 workspace
2. Initialize and update submodules (see above)
3. Install dependencies for each camera driver as per their respective documentation
4. Build the workspace
5. Source the setup file: `source install/setup.bash`

## License

See individual package LICENSE files for details.
