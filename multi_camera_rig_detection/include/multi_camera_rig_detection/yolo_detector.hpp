#ifndef MULTI_CAMERA_RIG_DETECTION_YOLO_DETECTOR_HPP
#define MULTI_CAMERA_RIG_DETECTION_YOLO_DETECTOR_HPP

#include "multi_camera_rig_common/trt_runner.hpp"
#include "multi_camera_rig_detection/detection_utils.hpp"

#include <opencv2/core.hpp>
#include <memory>
#include <string>
#include <vector>

namespace multi_camera_rig_detection
{

/**
 * @brief YOLO detector configuration (detection + optional segmentation)
 */
struct YoloDetectorConfig
{
    // Engine + tensors
    std::string engine_path;
    std::string input_tensor{"images"};
    std::string output_tensor{"output0"};

    // Optional segmentation proto tensor
    std::string proto_tensor{"output1"};

    // Task control: "auto" | "det" | "seg"
    std::string task{"auto"};

    // Input dimensions
    int input_w{1440};
    int input_h{1088};
    int stride{32};
    bool scaleup{true};

    // Detection parameters
    double conf_thresh{0.25};
    double iou_thresh{0.45};
    int max_det{300};

    // Segmentation parameters
    int mask_dim{32};          // typical
};

/**
 * @brief YOLO (v8+ style) detector using TensorRT. Supports:
 *   - detection engines with output (1,C,N)
 *   - segmentation engines with output0 (1,N,C) and proto output1 (1,mask_dim,Hp,Wp)
 */
class YoloDetector
{
public:
    explicit YoloDetector(const YoloDetectorConfig &config);
    ~YoloDetector() = default;

    void detect(const cv::Mat &bgr, std::vector<Det> &dets);

    static void scaleDetections(const std::vector<Det> &dets,
                                double scale_x, double scale_y,
                                std::vector<Det> &dets_scaled);

    const LetterboxInfo &getLetterboxInfo() const { return lb_info_; }

    bool isSegmentation() const { return is_segmentation_; }
    const std::vector<float> &proto() const { return proto_output_; }
    const TensorDims &protoShape() const { return proto_shape_; }

private:
    YoloDetectorConfig config_;
    std::unique_ptr<multi_camera_rig_common::TrtRunner> runner_;

    size_t input_floats_{0};
    size_t out_floats_{0};
    size_t proto_floats_{0};

    std::vector<float> input_nchw_;
    std::vector<float> output_;
    std::vector<float> proto_output_;

    LetterboxInfo lb_info_;

    bool is_segmentation_{false};
    TensorDims out_shape_{};
    TensorDims proto_shape_{};
};

} // namespace multi_camera_rig_detection

#endif // MULTI_CAMERA_RIG_DETECTION_YOLO_DETECTOR_HPP
