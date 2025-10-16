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

The FoundationStereo model requires its own conda environment with specific dependencies. Follow these steps to set it up:

1. **Create the conda environment** from the provided environment file:
   ```bash
   cd external/FoundationStereo
   conda env create -f environment.yml
   ```

2. **Install flash-attn separately** (required to avoid environment creation errors):
   ```bash
   conda run -n foundation_stereo pip install flash-attn
   ```

3. **Activate the environment**:
   ```bash
   conda activate foundation_stereo
   ```

### Download Model Weights

Download one of the pre-trained models and place it in `external/FoundationStereo/pretrained_models/`:

- **[23-51-11](https://drive.google.com/drive/folders/1VhPebc_mMxWKccrv7pdQLTvXYVcLYpsf?usp=sharing)** (Recommended) - Best performing model, based on ViT-large
- **[11-33-40](https://drive.google.com/drive/folders/1VhPebc_mMxWKccrv7pdQLTvXYVcLYpsf?usp=sharing)** - Faster inference with slightly lower accuracy, based on ViT-small

Example:
```bash
cd external/FoundationStereo
mkdir -p pretrained_models
# Download and extract the model folder (e.g., 23-51-11) into pretrained_models/
```

### Running FoundationStereo

To run stereo depth estimation on a pair of images:

```bash
conda activate foundation_stereo
cd external/FoundationStereo
python scripts/run_demo.py \
  --left_file ./assets/left.png \
  --right_file ./assets/right.png \
  --ckpt_dir ./pretrained_models/23-51-11/model_best_bp2.pth \
  --out_dir ./test_outputs/
```

**Important Notes:**
- Input stereo images must be **rectified and undistorted** (horizontal epipolar lines)
- Do not swap left and right images
- Use PNG files with no lossy compression for best results
- Works best with RGB images, but also supports monochrome/IR stereo
- For high-resolution images (>1000px), use `--hiera 1` for full resolution or `--scale 0.5` for faster inference
- Specify camera intrinsics for point cloud generation (see FoundationStereo documentation)

### GPU Requirements

Tested on NVIDIA GPUs: 3090, 4090, A100, V100, Jetson Orin. Ensure you have sufficient GPU memory for your chosen model.

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

### FoundationStereo (Optional)

If you plan to use the FoundationStereo depth estimation:

1. Set up the conda environment (see FoundationStereo Setup section)
2. Download the pre-trained model weights
3. Test the installation with the demo script

Note: The FoundationStereo environment is separate from ROS2 and should be activated only when running stereo depth estimation tasks.

## License

See individual package LICENSE files for details.
