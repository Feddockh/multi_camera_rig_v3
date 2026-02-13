#ifndef MULTI_CAMERA_RIG_DETECTION_YOLOV8_DETECTOR_HPP
#define MULTI_CAMERA_RIG_DETECTION_YOLOV8_DETECTOR_HPP

#include "multi_camera_rig_common/trt_runner.hpp"
#include "multi_camera_rig_detection/detection_utils.hpp"
#include <opencv2/core.hpp>
#include <memory>
#include <string>
#include <vector>

namespace multi_camera_rig_detection
{

/**
 * @brief YOLOv8 detector configuration
 */
struct Yolov8DetectorConfig
{
    // Model parameters
    std::string engine_path;
    std::string input_tensor{"images"};
    std::string output_tensor{"output0"};

    // Input dimensions
    int input_w{1440};
    int input_h{1088};
    int stride{32};
    bool scaleup{true};

    // Detection parameters
    double conf_thresh{0.25};
    double iou_thresh{0.45};
    int max_det{300};
};

/**
 * @brief YOLOv8 object detector using TensorRT
 */
class Yolov8Detector
{
public:
    /**
     * @brief Construct detector with configuration
     * @param config Detector configuration
     */
    explicit Yolov8Detector(const Yolov8DetectorConfig &config);
    ~Yolov8Detector() = default;

    /**
     * @brief Run detection on BGR image
     * @param bgr Input BGR image
     * @param dets Output detections
     */
    void detect(const cv::Mat &bgr, std::vector<Det> &dets);

    /**
     * @brief Scale detections to a different resolution
     * @param dets Input detections (in original resolution)
     * @param scale_x X scale factor (output_width / input_width)
     * @param scale_y Y scale factor (output_height / input_height)
     * @param dets_scaled Output scaled detections
     */
    static void scaleDetections(const std::vector<Det> &dets,
                                  double scale_x, double scale_y,
                                  std::vector<Det> &dets_scaled);

    /**
     * @brief Get letterbox info from last detection
     * @return Letterbox transformation info
     */
    const LetterboxInfo &getLetterboxInfo() const { return lb_info_; }

private:
    Yolov8DetectorConfig config_;
    std::unique_ptr<multi_camera_rig_common::TrtRunner> runner_;
    
    size_t input_floats_{0};
    size_t out_floats_{0};
    std::vector<float> input_nchw_;
    std::vector<float> output_;
    
    LetterboxInfo lb_info_;
};

} // namespace multi_camera_rig_detection

#endif // MULTI_CAMERA_RIG_DETECTION_YOLOV8_DETECTOR_HPP
