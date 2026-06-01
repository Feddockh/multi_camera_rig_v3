#ifndef MULTI_CAMERA_RIG_RECONSTRUCTION_FOUNDATION_STEREO_MATCHER_HPP
#define MULTI_CAMERA_RIG_RECONSTRUCTION_FOUNDATION_STEREO_MATCHER_HPP

#include "multi_camera_rig_common/trt_runner.hpp"
#include <opencv2/core.hpp>
#include <memory>
#include <string>
#include <vector>

namespace multi_camera_rig_reconstruction
{

/**
 * @brief Configuration for FoundationStereoMatcher
 */
struct FoundationStereoMatcherConfig
{
    std::string engine_path;
    
    // Disparity filter options
    std::string disp_filter_mode{"none"}; // "none" or "speckle"
    int speckle_max_size{120};
    double speckle_range{1.0};   // in disparity pixels
    double speckle_scale{16.0};  // float->fixed conversion scale
    float min_disparity{0.0f};   // pixels below this are set to 0 (depth > max_range)
};

/**
 * @brief Foundation stereo matching processor
 * Uses TensorRT for GPU-accelerated stereo matching
 */
class FoundationStereoMatcher
{
public:
    /**
     * @brief Construct matcher with configuration
     * @param config Matcher configuration
     */
    explicit FoundationStereoMatcher(const FoundationStereoMatcherConfig &config);

    /**
     * @brief Get input width expected by engine
     */
    int inputWidth() const { return in_w_; }

    /**
     * @brief Get input height expected by engine
     */
    int inputHeight() const { return in_h_; }

    /**
     * @brief Get output width produced by engine
     */
    int outputWidth() const { return out_w_; }

    /**
     * @brief Get output height produced by engine
     */
    int outputHeight() const { return out_h_; }

    /**
     * @brief Process stereo pair to generate disparity map
     * @param left_bgr Left image in BGR format (CV_8UC3)
     * @param right_bgr Right image in BGR format (CV_8UC3)
     * @param disparity Output disparity map (CV_32FC1), allocated by this method
     */
    void process(const cv::Mat &left_bgr, const cv::Mat &right_bgr, cv::Mat &disparity);

private:
    void bgrToNCHWFloat(const cv::Mat &bgr, int H, int W, std::vector<float> &out);
    void applyDisparityFilter(cv::Mat &disparity);

    FoundationStereoMatcherConfig config_;
    std::unique_ptr<multi_camera_rig_common::TrtRunner> runner_;
    
    int in_w_{0}, in_h_{0};
    int out_w_{0}, out_h_{0};
    
    // Buffers
    std::vector<float> left_nchw_;
    std::vector<float> right_nchw_;
    std::vector<float> disp_buffer_;
};

} // namespace multi_camera_rig_reconstruction

#endif // MULTI_CAMERA_RIG_RECONSTRUCTION_FOUNDATION_STEREO_MATCHER_HPP
