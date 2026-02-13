# TensorRT Tools

## make_tensorrt

Unified tool for building TensorRT engines from ONNX models. Works with YOLO detection models, stereo reconstruction models, and other ONNX formats.

### Build

```bash
cd tools/
./build_tool.sh
```

Or manually:
```bash
g++ -O3 -std=c++17 make_tensorrt.cpp -o make_tensorrt \
  -I/usr/local/cuda/targets/x86_64-linux/include \
  -L/usr/local/cuda/targets/x86_64-linux/lib \
  -lnvinfer -lnvonnxparser -lnvinfer_plugin -lcudart
```

### Usage

#### Auto Mode (Default)
Automatically handles dynamic dimensions by replacing -1 with 1. Suitable for stereo models and models with simple input shapes.

```bash
./make_tensorrt model.onnx output.plan
```

#### Manual Shape Override
Explicitly specify min/opt/max shapes for optimization. Recommended for YOLO models with specific input sizes.

```bash
./make_tensorrt model.onnx output.plan \
  --min=1x3x640x640 \
  --opt=1x3x1088x1440 \
  --max=1x3x1088x1440
```

### Examples

**YOLO Detection Model (1088x1440)**
```bash
./make_tensorrt yolov8n.onnx yolov8n.plan \
  --min=1x3x1088x1440 \
  --opt=1x3x1088x1440 \
  --max=1x3x1088x1440
```

**Stereo Reconstruction Model**
```bash
./make_tensorrt foundation_stereo.onnx foundation_stereo.plan
```

### Features

- **FP16 Optimization**: Automatically enables FP16 mode if supported by GPU
- **Dynamic Batch Support**: Handles models with dynamic dimensions
- **Flexible Input Shapes**: Supports both auto-detection and manual override
- **Multi-Input Models**: Works with models having multiple input tensors
- **Optimization Profiles**: Creates proper min/opt/max profiles for dynamic shapes
- **Large Workspace**: Allocates 8GB workspace for complex model optimization
