# Multi Camera Rig V3

ROS2 packages for a modular multi-camera stereo vision system with AI-powered detection and 3D reconstruction.

## Repository Structure

```
multi_camera_rig_v3/
├── multi_camera_rig_bringup/          # Main launch orchestration
├── multi_camera_rig_common/           # Shared TensorRT runner & QoS utilities
├── multi_camera_rig_detection/        # Generalizable YOLO detection
├── multi_camera_rig_reconstruction/   # Generalizable stereo reconstruction
├── multi_camera_rig_trigger/          # Video trigger control
├── multi_camera_rig_description/      # URDF and visualization
├── multi_camera_rig_cameras/
│   ├── firefly-ros2-wrapper/          # FLIR Firefly stereo camera
│   │   └── flir_camera_driver/        # (submodule)
│   ├── ximea-ros2-wrapper/            # Ximea camera
│   └── zed-ros2-wrapper/              # (submodule) Stereolabs ZED
└── external/
    └── FoundationStereo/              # (submodule) Zero-shot stereo model
```

## Git Submodules

This repository uses three submodules:
- **flir_camera_driver**: [ros-drivers/flir_camera_driver](https://github.com/ros-drivers/flir_camera_driver)
- **zed-ros2-wrapper**: [stereolabs/zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
- **FoundationStereo**: [NVlabs/FoundationStereo](https://github.com/NVlabs/FoundationStereo)

### Cloning

```bash
# Clone with submodules
git clone --recurse-submodules https://github.com/Feddockh/multi_camera_rig_v3.git

# Or if already cloned
git submodule update --init --recursive
```

### Updating Submodules

```bash
# Update all submodules
git submodule update --remote

# Update specific submodule
git submodule update --remote external/FoundationStereo

# Commit changes
git add .
git commit -m "Update submodules"
```

## Dependencies

### Optional: CUDA & TensorRT

The `multi_camera_rig_detection` and `multi_camera_rig_reconstruction` packages require **CUDA** and **TensorRT** to build their inference nodes (`yolo_trt_node`, `foundation_stereo_matcher_node`). If CUDA or TensorRT is not found on your system, these targets are automatically skipped and a warning is printed during CMake configuration.

To enable them, install:
- [CUDA Toolkit](https://developer.nvidia.com/cuda-downloads) (tested with CUDA 11/12, and CUDA 13.3)
- [TensorRT](https://developer.nvidia.com/tensorrt) — ensure `libnvinfer` and `libnvonnxparser` are on the library path (typically under `/usr/lib/x86_64-linux-gnu/` or `/usr/local/cuda/`)

**Verified configurations:**
| TensorRT | CUDA | Driver | GPU |
|---|---|---|---|
| 10.9.0.34 (.deb) | 11/12 | — | NVIDIA GeForce RTX 4090 |
| 11.2.1.2 (.deb) | 13.3 | 610.43.02 | NVIDIA GeForce RTX 2060 |

> `nvidia-smi` failing or CUDA reporting no device even though the driver is installed? See
> [Troubleshooting](#troubleshooting).

### Optional: Spinnaker SDK (FLIR Firefly cameras)

The real hardware pipeline for the FLIR Firefly stereo cameras requires the **Spinnaker SDK** from Teledyne Vision Solutions. Without it, `spinnaker_synchronized_camera_driver` will not be available at runtime and the real hardware launch path will fail. Simulation (`use_gazebo:=true`) works without it.

See [multi_camera_rig_cameras/firefly-ros2-wrapper/README.md](multi_camera_rig_cameras/firefly-ros2-wrapper/README.md) for full installation instructions.

### Optional: Ximea SDK

The `ximea-ros2-wrapper-bringup` package requires the **Ximea API** to build `ximea_camera_node`. If the SDK is not found, the node is skipped and a warning is printed during CMake configuration.

To enable it, install the [Ximea Linux Software Package](https://www.ximea.com/support/wiki/apis/XIMEA_Linux_Software_Package) (installs to `/opt/XIMEA/`).

```bash
# After downloading and extracting the Ximea SDK:
cd package
sudo ./install
```

## Setup

### 1. Build ROS2 Packages

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Download & Compile Models

Model preparation has three stages: download pretrained weights, build the FoundationStereo
ONNX model from its checkpoint, then compile both ONNX models into TensorRT engines. Models
are listed in [`scripts/models_manifest.py`](scripts/models_manifest.py) — add or swap in
other models by editing that file.

**Step 1 — Download pretrained weights** (prompts with sizes before downloading anything):
```bash
cd ~/ros2_ws/src/multi_camera_rig_v3
python3 scripts/download_models.py --all
```
This downloads the FoundationStereo ViT-Small (11-33-40) checkpoint into
`external/FoundationStereo/pretrained_models/11-33-40/` and the YOLO segmentation ONNX model
into `multi_camera_rig_detection/models/`.

**Step 2 — Build the FoundationStereo ONNX model**

FoundationStereo is distributed as a PyTorch checkpoint, not ONNX, so it needs to be exported
first. This uses the same isolated `model_tools` conda environment as other model tooling in
this repo, so its dependencies (PyTorch, CUDA wheels, etc.) never touch the system Python that
`colcon`/ROS2 rely on.
```bash
conda env create -f scripts/environment_model_tools.yml   # one-time setup
conda activate model_tools

cd external/FoundationStereo
XFORMERS_DISABLED=1 python scripts/make_onnx.py \
  --save_path ../../multi_camera_rig_reconstruction/models/fs_224x448_vit-small_iters5.onnx \
  --ckpt_dir ./pretrained_models/11-33-40/model_best_bp2.pth \
  --height 224 --width 448 --valid_iters 5
cd ../..

conda deactivate
```
Mixed precision is off by default (no extra flag needed) — TensorRT 11+ builds strongly-typed
engines and has no low-precision kernel for some layers in this model, so a mixed-precision
export fails to compile. See `MODIFICATIONS.md` in the FoundationStereo submodule for details.

**Step 3 — Compile to TensorRT** (run from the repo root):
```bash
./multi_camera_rig_common/tools/build_tool.sh
./multi_camera_rig_common/tools/make_tensorrt \
  multi_camera_rig_reconstruction/models/fs_224x448_vit-small_iters5.onnx \
  multi_camera_rig_reconstruction/models/fs_224x448_vit-small_iters5.plan
./multi_camera_rig_common/tools/make_tensorrt \
  multi_camera_rig_detection/models/best_lab_seg_v2.onnx \
  multi_camera_rig_detection/models/best_lab_seg_v2.plan
```

## Running the System

### Quick Start

**Full Pipeline (Real Hardware):**
```bash
ros2 launch multi_camera_rig_bringup firefly_bringup.launch.py
```

**Full Pipeline (Simulation):**
```bash
ros2 launch multi_camera_rig_bringup firefly_bringup.launch.py use_gazebo:=true
```

**Full App (Cameras + Rig Model + GUI + RViz, single unit):**
```bash
ros2 launch multi_camera_rig_bringup app.launch.py
```
Brings up `firefly_bringup.launch.py`, the Ximea camera, the rig's robot model
(`robot_state_publisher`/`joint_state_publisher`), the control GUI, and an RViz
window showing the rig model and the semantic point cloud
(`/firefly_left/points2`) all together. Closing the GUI window or the RViz
window shuts everything else down.

To launch it from a single desktop icon instead, run this once:
```bash
./multi_camera_rig_bringup/scripts/install_desktop_app.sh
```
This installs a "Multi-Camera Demo App" icon into the GNOME app menu and onto
your Desktop (re-run it any time, e.g. after moving the repo, to refresh the
install).

### Camera Only

**Real Hardware:**
```bash
ros2 launch firefly-ros2-wrapper-bringup bringup.launch.py
```

**Simulation:**
```bash
ros2 launch firefly-ros2-wrapper-bringup bringup.launch.py use_gazebo:=true
```

### Configuration Options

```bash
# Disable detection (reconstruction only)
ros2 launch multi_camera_rig_bringup firefly_bringup.launch.py enable_detection:=false

# Non-semantic pointcloud
ros2 launch multi_camera_rig_bringup firefly_bringup.launch.py use_semantics:=false

# Custom resolution
ros2 launch multi_camera_rig_bringup firefly_bringup.launch.py \
  output_width:=896 output_height:=672

# Custom models
ros2 launch multi_camera_rig_bringup firefly_bringup.launch.py \
  detection_model_trt:=custom.plan \
  stereo_matcher_model_trt:=custom_stereo.plan

# ArUco ground truth generation
ros2 launch multi_camera_rig_bringup firefly_bringup.launch.py detect_markers:=true
```

## Custom / Advanced Model Setup

The steps below are only needed if you're training/exporting your own weights instead of
using the pretrained models from [Download & Compile Models](#2-download--compile-models).

### FoundationStereo Model Setup

**Download Pre-trained Weights:**
- [ViT-Large (23-51-11)](https://drive.google.com/drive/folders/1VhPebc_mMxWKccrv7pdQLTvXYVcLYpsf) - Best accuracy
- [ViT-Small (11-33-40)](https://drive.google.com/drive/folders/1VhPebc_mMxWKccrv7pdQLTvXYVcLYpsf) - Faster inference

Place in: `external/FoundationStereo/pretrained_models/`

**Setup Conda Environment:**
```bash
cd external/FoundationStereo
conda env create -f environment_ros2.yml  # Python 3.10 for ROS2 compatibility
conda activate foundation_stereo_ros2
```

### Model Conversion Pipeline

**Convert PyTorch → ONNX → TensorRT**

```bash
# Step 1: Export PyTorch to ONNX
cd external/FoundationStereo
python tools/export_onnx.py \
  --model pretrained_models/23-51-11/model.pth \
  --output foundation_stereo_1088x1440.onnx \
  --height 1088 --width 1440

# Step 2: Build TensorRT tool
cd ../../multi_camera_rig_common/tools
./build_tool.sh

# Step 3: Convert ONNX to TensorRT (auto mode for stereo models)
./make_tensorrt \
  foundation_stereo_1088x1440.onnx \
  foundation_stereo_1088x1440.plan

# For YOLO models (with explicit shape override):
yolo export model=/path/to/best.pt \
    format=onnx \
    imgsz=1088,1440 \
    opset=17 \
    simplify=True \
    dynamic=False \
    nms=False

./make_tensorrt yolov8n.onnx yolov8n_1088x1440.plan \
  --min=1x3x1088x1440 --opt=1x3x1088x1440 --max=1x3x1088x1440
```

**Place Models:**
- Detection: `multi_camera_rig_detection/models/*.plan`
- Stereo: `multi_camera_rig_reconstruction/models/*.plan`

## Key Topics

### Camera
- `/firefly_left/image_raw` - Raw left image
- `/firefly_right/image_raw` - Raw right image
- `/firefly_left/camera_info` - Camera calibration

### Detection
- `/firefly_left/detections` - YOLO detections

### Reconstruction
- `/firefly_left/disparity` - Stereo disparity map
- `/firefly_left/points2` - Semantic pointcloud
- `/firefly_left/depth` - Depth image

## Utility Nodes

### QoS Republisher
```bash
ros2 run multi_camera_rig_bringup qos_republisher_node --ros-args \
  -p in_topic:=/camera/image_raw -p out_topic:=/camera/image_be \
  -p sub_qos.reliability:=reliable -p pub_qos.reliability:=best_effort
```

### Image Saver
```bash
ros2 run multi_camera_rig_bringup image_saver_node --ros-args \
  -p image_topic:=/firefly_left/image_rect \
  -p save_directory:=~/saved_images
```

### Joystick Trigger
```bash
ros2 run joy joy_node  # Publish /joy
ros2 run multi_camera_rig_trigger joy_trigger_node  # Map buttons to triggers
```

## Troubleshooting

**Cannot find package:**
```bash
source ~/ros2_ws/install/setup.bash
```

**TensorRT engine not found:**
```bash
ls multi_camera_rig_detection/models/
ls multi_camera_rig_reconstruction/models/
```
Install TensorRT via the .deb install following this guide: https://docs.nvidia.com/deeplearning/tensorrt/latest/installing-tensorrt/installing.html
See [Verified configurations](#optional-cuda--tensorrt) for TensorRT/CUDA/driver combinations this repo has been tested with.

**`nvidia-smi` fails / CUDA reports "no CUDA-capable device is detected" despite the driver being installed:**

On systems with Secure Boot enabled, an out-of-tree DKMS driver (like `nvidia-dkms`) builds a
kernel module signed with a locally generated key that the kernel doesn't trust yet, so the
module silently fails to load. Check whether this is the cause:
```bash
mokutil --sb-state          # "SecureBoot enabled"?
dmesg | grep -i nvidia      # no output at all = module never loaded
```
Fix by enrolling the DKMS signing key into Secure Boot's Machine Owner Key (MOK) database:
```bash
sudo mokutil --import /var/lib/shim-signed/mok/MOK.der
sudo reboot
```
On reboot, a blue **MOK Manager** screen appears before the OS boots — select **Enroll MOK →
Continue → Yes**, enter the password you set above, then let it reboot again. After that,
`nvidia-smi` should show your GPU.

**`conda: command not found` (needed for Step 2 of [Download & Compile Models](#2-download--compile-models)):**

Install Miniconda first:
```bash
curl -O https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh -b -p "$HOME/miniconda3"
source "$HOME/miniconda3/etc/profile.d/conda.sh"   # or open a new shell after `conda init`
```
This repo's conda environments use the `conda-forge` channel, so no Anaconda Terms of Service
acceptance is needed.

**TensorRT compile fails with `No low-precision conv kernel available for this strongly-typed Conv/ConvTranspose` (TensorRT 11+):**

TensorRT 11+ networks are strongly typed: op precision comes from the ONNX graph itself rather
than a builder flag. If a model's graph specifies FP16 for a layer your GPU/TensorRT combo has
no low-precision kernel for, `make_tensorrt` fails outright instead of falling back to FP32 for
that op. FoundationStereo hits this exact issue if exported with mixed precision, which is why
[Step 2](#2-download--compile-models) exports it with mixed precision off by default.

## License

See individual package LICENSE files for details.
