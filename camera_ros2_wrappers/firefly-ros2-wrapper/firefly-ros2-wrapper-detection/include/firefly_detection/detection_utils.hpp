#ifndef FIREFLY_DETECTION_DETECTION_UTILS_HPP
#define FIREFLY_DETECTION_DETECTION_UTILS_HPP

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

namespace firefly_detection
{

/**
 * @brief Detection result
 */
struct Det
{
    int cls{-1};
    float conf{0.f};
    float x1{0.f}, y1{0.f}, x2{0.f}, y2{0.f};
};

/**
 * @brief Letterbox transformation information
 */
struct LetterboxInfo
{
    float scale{1.f};
    int pad_x{0}; // left pad
    int pad_y{0}; // top pad
    int in_w{0}, in_h{0};
    int out_w{0}, out_h{0};
};

/**
 * @brief Simple tensor dimensions (batch, channels, spatial)
 */
struct TensorDims
{
    int nbDims{0};
    int d[8]{0};
};

/**
 * @brief Apply letterbox padding to image (Ultralytics-like)
 * @param bgr Input BGR image
 * @param out_w Target width
 * @param out_h Target height
 * @param stride Stride for padding alignment (default: 32)
 * @param scaleup Whether to scale up small images (default: true)
 * @param pad_color Padding color (default: gray 114)
 * @param info Optional output letterbox info
 * @return Letterboxed image
 */
cv::Mat letterbox(const cv::Mat &bgr,
                  int out_w, int out_h,
                  int stride = 32,
                  bool scaleup = true,
                  const cv::Scalar &pad_color = cv::Scalar(114, 114, 114),
                  LetterboxInfo *info = nullptr);

/**
 * @brief Convert BGR image to RGB NCHW float format [0,1]
 * @param bgr Input BGR image
 * @param out_nchw Output NCHW float array
 */
void bgrToRGBNCHW01(const cv::Mat &bgr, std::vector<float> &out_nchw);

/**
 * @brief Parse YOLO output tensor (1, 4+nc, N) format
 * @param out Output tensor data
 * @param out_dims Output tensor dimensions
 * @param lb Letterbox transformation info
 * @param conf_thresh Confidence threshold
 * @param iou_thresh IoU threshold for NMS
 * @param max_det Maximum detections to return
 * @param dets Output detections
 */
void parseYoloCxCyWhClassScores(const float *out,
                                const TensorDims &out_dims,
                                const LetterboxInfo &lb,
                                float conf_thresh,
                                float iou_thresh,
                                int max_det,
                                std::vector<Det> &dets);

} // namespace firefly_detection

#endif // FIREFLY_DETECTION_DETECTION_UTILS_HPP
