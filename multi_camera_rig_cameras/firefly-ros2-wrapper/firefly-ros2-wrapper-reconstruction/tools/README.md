# Foundation Stereo TensorRT Setup

This guide describes how to set up a TensorRT version of Foundation Stereo for use on your machine.

## Prerequisites

### Hardware Requirements
- GPU: NVIDIA RTX 4090 or better recommended
- GPU Memory: 16GB minimum
- System RAM: 32GB minimum

**Note:** If your hardware is less capable, you may be limited to smaller model sizes (e.g., 896×896 resolution).

## Setup Steps

### 1. Download FoundationStereo Models

Download the FoundationStereo models from their official repository.

### 2. Set Up Conda Environment

Create a conda environment using the provided `environment_ros2.yml` file. This environment is specifically configured to work with ROS2 requirements:
- NumPy version < 2.0
- PyTorch with sufficient functionality to load models and create ONNX exports

### 3. Create ONNX Model

Navigate to `external/FoundationStereo/scripts` and use the `make_onnx` script.

**Important Parameters:**
- **Image dimensions:** Height and width must be divisible by 224 due to downsampling requirements
- **Environment variable:** Set `XFORMERS_DISABLED=1` in the Python call
- **Model size:** Limited to 896×896 for systems with 16GB GPU memory and 32GB RAM
- **Iterations:** Lower iteration counts reduce model quality slightly but provide faster inference times

**Note:** While compute restrictions may not be apparent at this step, the TensorRT conversion in the next step will likely fail if the model is too large.

### 4. Convert to TensorRT

There are two approaches for TensorRT conversion:

#### Option A: Using Conda TensorRT (Testing Only)
Use the conversion script provided in the FoundationStereo repository. This uses the TensorRT version installed via conda.

**Use this option if:** You only need to test the model within the FoundationStereo package.

#### Option B: Using System TensorRT (ROS2 Integration)
Build and run the `make_tensor.cpp` script located in:
```
camera_ros2_wrappers/firefly-ros2-wrapper/firefly-ros2-wrapper-bringup/tools
```

**Use this option if:** You intend to use the model as part of your ROS2 stereo matching pipeline.

**Rationale:** The conda TensorRT version may conflict with the system-installed TensorRT version. For ROS2 integration, using the system TensorRT ensures compatibility.
