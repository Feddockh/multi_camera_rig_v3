#!/bin/bash
# Build the unified TensorRT ONNX-to-engine converter

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "Building make_tensorrt tool..."

g++ -O3 -std=c++17 make_tensorrt.cpp -o make_tensorrt \
  -I/usr/local/cuda/targets/x86_64-linux/include \
  -L/usr/local/cuda/targets/x86_64-linux/lib \
  -lnvinfer -lnvonnxparser -lnvinfer_plugin -lcudart

if [ $? -eq 0 ]; then
  echo "✓ Build successful: ./make_tensorrt"
  echo ""
  echo "Usage examples:"
  echo "  Auto mode (for stereo/generic models):"
  echo "    ./make_tensorrt model.onnx output.plan"
  echo ""
  echo "  Manual shape override (for YOLO):"
  echo "    ./make_tensorrt yolo.onnx yolo.plan --min=1x3x1088x1440 --opt=1x3x1088x1440 --max=1x3x1088x1440"
else
  echo "✗ Build failed"
  exit 1
fi
