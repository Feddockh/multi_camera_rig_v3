# Place best.pt in the models folder
Currently, best.pt is from yolov8_large_rivendale_v6_k_fold1/weights/best.pt

# Export YOLOv8 to ONNX (raw outputs; do NMS yourself in C++)
yolo export model=/path/to/best.pt format=onnx imgsz=1088,1440 opset=17 simplify=True dynamic=False nms=False

# Generate the TensorRT from ONNX model
Use the unified TensorRT tool from the common package:

Build tool (if not already built):
  cd ../multi_camera_rig_common/tools && ./build_tool.sh

Run with shape override for YOLO model (1088x1440):
  ../multi_camera_rig_common/tools/make_tensorrt model.onnx out.plan \
    --min=1x3x1088x1440 --opt=1x3x1088x1440 --max=1x3x1088x1440

See ../multi_camera_rig_common/tools/README.md for more details.