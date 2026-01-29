#ifndef FIREFLY_DETECTION_YOLOV8_DETECTOR_HPP
#define FIREFLY_DETECTION_YOLOV8_DETECTOR_HPP

#include "firefly_detection/trt_runner.hpp"
#include "firefly_detection/detection_utils.hpp"
#include <opencv2/core.hpp>
#include <memory>
#include <string>
#include <vector>

namespace firefly_detection
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
     * @brief Get letterbox info from last detection
     * @return Letterbox transformation info
     */
    const LetterboxInfo &getLetterboxInfo() const { return lb_info_; }

private:
    Yolov8DetectorConfig config_;
    std::unique_ptr<TrtRunner> runner_;
    
    size_t input_floats_{0};
    size_t out_floats_{0};
    std::vector<float> input_nchw_;
    std::vector<float> output_;
    
    LetterboxInfo lb_info_;
};

} // namespace firefly_detection

#endif // FIREFLY_DETECTION_YOLOV8_DETECTOR_HPP
