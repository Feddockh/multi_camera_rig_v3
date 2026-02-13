#include "firefly_detection/yolov8_detector.hpp"
#include <chrono>
#include <stdexcept>

namespace firefly_detection
{

// Helper to convert nvinfer1::Dims to TensorDims
static TensorDims convertDims(const nvinfer1::Dims &in)
{
    TensorDims out;
    out.nbDims = in.nbDims;
    for (int i = 0; i < in.nbDims && i < 8; ++i)
        out.d[i] = in.d[i];
    return out;
}

Yolov8Detector::Yolov8Detector(const Yolov8DetectorConfig &config)
    : config_(config)
{
    // Initialize TensorRT runner from common package
    runner_ = std::make_unique<multi_camera_rig_common::TrtRunner>(config_.engine_path, true);

    // Get tensor shapes
    auto in_shape = runner_->shapeOf(config_.input_tensor);
    auto out_shape = runner_->shapeOf(config_.output_tensor);

    // Validate input shape
    if (in_shape.nbDims != 4 || in_shape.d[0] != 1 || in_shape.d[1] != 3)
    {
        throw std::runtime_error("Invalid input shape: expected (1,3,H,W)");
    }
    if (out_shape.nbDims != 3)
    {
        throw std::runtime_error("Invalid output shape: expected (1, 4+nc, N)");
    }

    // Allocate input/output buffers
    input_floats_ = static_cast<size_t>(in_shape.d[1] * in_shape.d[2] * in_shape.d[3]);
    out_floats_ = static_cast<size_t>(out_shape.d[1] * out_shape.d[2]);

    input_nchw_.resize(input_floats_);
    output_.resize(out_floats_);
}

void Yolov8Detector::detect(const cv::Mat &bgr, std::vector<Det> &dets)
{
    // Letterbox preprocessing
    cv::Mat lb_img = letterbox(bgr, config_.input_w, config_.input_h, 
                                config_.stride, config_.scaleup, 
                                cv::Scalar(114, 114, 114), &lb_info_);

    // Convert to RGB NCHW
    bgrToRGBNCHW01(lb_img, input_nchw_);

    // Run inference using new multi-tensor API
    multi_camera_rig_common::TensorSpec input_spec{config_.input_tensor, input_nchw_.data(), nullptr, input_floats_};
    multi_camera_rig_common::TensorSpec output_spec{config_.output_tensor, nullptr, output_.data(), out_floats_};
    runner_->run({input_spec}, {output_spec});

    // Parse detections - convert nvinfer1::Dims to TensorDims
    auto out_shape = runner_->shapeOf(config_.output_tensor);
    TensorDims tensor_dims = convertDims(out_shape);
    
    parseYoloCxCyWhClassScores(output_.data(), tensor_dims, lb_info_,
                               config_.conf_thresh, config_.iou_thresh,
                               config_.max_det, dets);
}

void Yolov8Detector::scaleDetections(const std::vector<Det> &dets,
                                      double scale_x, double scale_y,
                                      std::vector<Det> &dets_scaled)
{
    dets_scaled.clear();
    dets_scaled.reserve(dets.size());
    
    for (const auto &d : dets)
    {
        Det scaled;
        scaled.cls = d.cls;
        scaled.conf = d.conf;
        scaled.x1 = d.x1 * scale_x;
        scaled.y1 = d.y1 * scale_y;
        scaled.x2 = d.x2 * scale_x;
        scaled.y2 = d.y2 * scale_y;
        dets_scaled.push_back(scaled);
    }
}

} // namespace firefly_detection
