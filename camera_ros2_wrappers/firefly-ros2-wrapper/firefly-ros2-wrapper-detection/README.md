# Place best.pt in the models folder
Currently, best.pt is from yolov8_large_rivendale_v6_k_fold1/weights/best.pt

# Export YOLOv8 to ONNX (raw outputs; do NMS yourself in C++)
yolo export model=/path/to/best.pt format=onnx imgsz=1088,1440 opset=17 simplify=True dynamic=False nms=False

# Generate the TensorRT from ONNX model
Compile (adjust CUDA path if needed):
  g++ -O3 -std=c++17 tools/make_tensorrt_yolo.cpp -o tools/make_tensorrt_yolo \
    -I/usr/local/cuda/targets/x86_64-linux/include \
    -L/usr/local/cuda/targets/x86_64-linux/lib \
    -lnvinfer -lnvonnxparser -lnvinfer_plugin -lcudart

Run:
  ./tools/make_tensorrt_yolo /path/to/model.onnx /path/to/out.plan

Optional override shapes:
  ./tools/make_tensorrt_yolo model.onnx out.plan --min=1x3x1088x1440 --opt=1x3x1088x1440 --max=1x3x1088x1440