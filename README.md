# Multi Camera Rig V3

This repository contains the ROS2 packages for the multi-camera rig system.

## Repository Structure

- `multi_camera_rig_description/` - URDF and visualization configuration for the multi-camera rig
- `camera_ros2_wrappers/` - ROS2 wrapper packages for various cameras
  - `firefly-ros2-wrapper/` - FLIR Firefly camera wrapper
  - `ximea-ros2-wrapper/` - Ximea camera wrapper
  - `zed-ros2-wrapper/` - Stereolabs ZED camera wrapper (submodule)
- `external/` - External dependencies and tools
  - `FoundationStereo/` - Zero-shot stereo matching foundation model (submodule)

## Git Submodules

This repository uses git submodules to manage external dependencies:
- **zed-ros2-wrapper**: [stereolabs/zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
- **flir_camera_driver**: [ros-drivers/flir_camera_driver](https://github.com/ros-drivers/flir_camera_driver)
- **FoundationStereo**: [NVlabs/FoundationStereo](https://github.com/NVlabs/FoundationStereo) - Zero-shot stereo matching foundation model

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
git submodule update --remote external/FoundationStereo

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

## FoundationStereo Setup

FoundationStereo is a zero-shot stereo matching foundation model that provides robust depth estimation from stereo camera pairs without requiring per-domain fine-tuning.

### Conda Environment Setup

The FoundationStereo model requires its own conda environment with specific dependencies. There is a environment.yml file within the repo which came with the FoundationStereo (Python 11), but if you want functionality with ROS2 then you may want to use the environment_ros2.yml (Python 10) in the top-level of this repository.

### Download Model Weights

Download one of the pre-trained models and place it in `external/FoundationStereo/pretrained_models/`:

- **[23-51-11](https://drive.google.com/drive/folders/1VhPebc_mMxWKccrv7pdQLTvXYVcLYpsf?usp=sharing)** (Recommended) - Best performing model, based on ViT-large
- **[11-33-40](https://drive.google.com/drive/folders/1VhPebc_mMxWKccrv7pdQLTvXYVcLYpsf?usp=sharing)** - Faster inference with slightly lower accuracy, based on ViT-small

The ViT-small model is much faster for inferencing, so I might recommend that one if you plan to try ROS implementation.

### TensorRT Integration for ROS2

For production use with ROS2, you'll want to convert the FoundationStereo model to TensorRT for optimized inference. This is especially important for real-time stereo matching applications.

**See the detailed TensorRT setup guide:**
- [`camera_ros2_wrappers/firefly-ros2-wrapper/firefly-ros2-wrapper-bringup/tools/README.md`](camera_ros2_wrappers/firefly-ros2-wrapper/firefly-ros2-wrapper-bringup/tools/README.md)

This guide covers:
- Hardware requirements for TensorRT conversion
- ONNX export configuration
- TensorRT conversion for ROS2 integration
- Running the optimized model with the Firefly stereo camera node

## Building

Build the packages using colcon:

```bash
cd /path/to/ros2_ws
colcon build --packages-select multi_camera_rig_description firefly-ros2-wrapper-bringup firefly-ros2-wrapper-description ximea-ros2-wrapper-bringup ximea-ros2-wrapper-description
```

## Setup Instructions

### ROS2 Packages

1. Clone this repository into your ROS2 workspace
2. Initialize and update submodules (see Git Submodules section above)
3. Install dependencies for each camera driver as per their respective documentation
4. Build the workspace
5. Source the setup file: `source install/setup.bash`
6. If you intend to use the Foundation Stereo model for stereo matching you will need to follow the steps to set up the TensorRT model.

## License

See individual package LICENSE files for details.
