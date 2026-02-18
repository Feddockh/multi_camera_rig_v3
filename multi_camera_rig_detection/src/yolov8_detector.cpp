#include "multi_camera_rig_detection/yolov8_detector.hpp"

#include <stdexcept>
#include <string>
#include <vector>

namespace multi_camera_rig_detection
{

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
    runner_ = std::make_unique<multi_camera_rig_common::TrtRunner>(config_.engine_path, true);

    // Shapes
    auto in_shape = runner_->shapeOf(config_.input_tensor);
    auto out_shape = runner_->shapeOf(config_.output_tensor);
    out_shape_ = convertDims(out_shape);

    // Validate input shape
    if (in_shape.nbDims != 4 || in_shape.d[0] != 1 || in_shape.d[1] != 3)
        throw std::runtime_error("Invalid input shape: expected (1,3,H,W)");

    // Allocate input
    input_floats_ = static_cast<size_t>(in_shape.d[1] * in_shape.d[2] * in_shape.d[3]);
    input_nchw_.resize(input_floats_);

    // Allocate primary output
    if (out_shape.nbDims != 3 || out_shape.d[0] != 1)
        throw std::runtime_error("Invalid output shape: expected 3D with batch=1");

    out_floats_ = static_cast<size_t>(out_shape.d[1] * out_shape.d[2]);
    output_.resize(out_floats_);

    // Determine segmentation
    if (config_.task == "seg")
        is_segmentation_ = true;
    else if (config_.task == "det")
        is_segmentation_ = false;
    else
        is_segmentation_ = runner_->hasTensor(config_.proto_tensor);

    if (is_segmentation_)
    {
        if (!runner_->hasTensor(config_.proto_tensor))
            throw std::runtime_error("Segmentation requested but proto_tensor not found in engine: " + config_.proto_tensor);

        auto p = runner_->shapeOf(config_.proto_tensor);
        if (p.nbDims != 4 || p.d[0] != 1)
            throw std::runtime_error("Invalid proto shape: expected (1,mask_dim,Hp,Wp)");

        proto_shape_ = convertDims(p);
        proto_floats_ = (size_t)p.d[1] * (size_t)p.d[2] * (size_t)p.d[3];
        proto_output_.resize(proto_floats_);
    }
}

void Yolov8Detector::detect(const cv::Mat &bgr, std::vector<Det> &dets)
{
    // Letterbox
    cv::Mat lb_img = letterbox(bgr,
                               config_.input_w, config_.input_h,
                               config_.stride, config_.scaleup,
                               cv::Scalar(114, 114, 114),
                               &lb_info_);

    // BGR -> RGB NCHW [0,1]
    bgrToRGBNCHW01(lb_img, input_nchw_);

    // Inference
    multi_camera_rig_common::TensorSpec in_spec{config_.input_tensor, input_nchw_.data(), nullptr, input_floats_};
    multi_camera_rig_common::TensorSpec out0_spec{config_.output_tensor, nullptr, output_.data(), out_floats_};

    std::vector<multi_camera_rig_common::TensorSpec> outs;
    outs.push_back(out0_spec);

    if (is_segmentation_)
    {
        multi_camera_rig_common::TensorSpec proto_spec{config_.proto_tensor, nullptr, proto_output_.data(), proto_floats_};
        outs.push_back(proto_spec);
    }

    runner_->run({in_spec}, outs);

    // Parse based on output dims
    auto out_dims = runner_->shapeOf(config_.output_tensor);
    TensorDims s = convertDims(out_dims);

    // Case A: (1,C,N) typical YOLO det exports
    const bool is_c_n = (s.nbDims == 3 && s.d[0] == 1 && s.d[1] > 6 && s.d[2] > 1000);

    // Case B: (1,N,C) End2End / transposed, common for seg TRT exports
    const bool is_n_c = (s.nbDims == 3 && s.d[0] == 1 && s.d[1] <= 5000 && s.d[2] <= 256);

    if (is_n_c)
    {
        // End2End layout: [x1,y1,x2,y2,score,cls,(mask...)]
        const int C = s.d[2];
        const int num_mask_coeffs = (is_segmentation_ ? config_.mask_dim : 0);

        // If mask requested but engine uses different mask_dim, adapt safely
        int use_mask = 0;
        if (is_segmentation_)
        {
            const int available = C - 6;
            use_mask = std::max(0, std::min(num_mask_coeffs, available));
        }

        parseYoloEnd2EndTransposed(output_.data(), s, lb_info_,
                                   (float)config_.conf_thresh, config_.max_det,
                                   use_mask, dets);
        return;
    }

    if (is_c_n)
    {
        parseYoloCxCyWhClassScores(output_.data(), s, lb_info_,
                                   (float)config_.conf_thresh, (float)config_.iou_thresh,
                                   config_.max_det, dets);
        return;
    }

    // Fallback: try to interpret as (1,C,N)
    parseYoloCxCyWhClassScores(output_.data(), s, lb_info_,
                               (float)config_.conf_thresh, (float)config_.iou_thresh,
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
        scaled.x1 = (float)(d.x1 * scale_x);
        scaled.y1 = (float)(d.y1 * scale_y);
        scaled.x2 = (float)(d.x2 * scale_x);
        scaled.y2 = (float)(d.y2 * scale_y);
        scaled.mask_coeffs = d.mask_coeffs; // unchanged
        dets_scaled.push_back(std::move(scaled));
    }
}

} // namespace multi_camera_rig_detection
