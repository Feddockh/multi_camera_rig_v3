#ifndef FIREFLY_RECONSTRUCTION_STEREO_RECTIFY_SCALE_HPP
#define FIREFLY_RECONSTRUCTION_STEREO_RECTIFY_SCALE_HPP

#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

namespace firefly_reconstruction
{

/**
 * @brief Configuration for StereoRectifyScale
 */
struct StereoRectifyScaleConfig
{
    int output_width{896};
    int output_height{672};
    int interpolation{cv::INTER_LINEAR}; // cv::InterpolationFlags
};

/**
 * @brief Stereo rectification and scaling processor
 * Combines undistortion, rectification, and resolution scaling in one pass
 */
class StereoRectifyScale
{
public:
    /**
     * @brief Construct processor with configuration
     * @param config Processor configuration
     */
    explicit StereoRectifyScale(const StereoRectifyScaleConfig &config);

    /**
     * @brief Update camera calibration
     * @param camera_info ROS CameraInfo message
     */
    void updateCameraInfo(const sensor_msgs::msg::CameraInfo &camera_info);

    /**
     * @brief Rectify image (undistort and rectify)
     * @param input Input image (any encoding)
     * @param output Output rectified image (same encoding)
     * @param output_camera_info Output rectified camera info
     * @return true if processing succeeded, false if calibration not ready
     */
    bool rectify(const cv::Mat &input, cv::Mat &output,
                 sensor_msgs::msg::CameraInfo &output_camera_info);

    /**
     * @brief Scale image and camera info
     * @param input Input image (typically rectified)
     * @param input_camera_info Input camera info
     * @param output Output scaled image (same encoding)
     * @param output_camera_info Output scaled camera info
     * @return true if processing succeeded
     */
    bool scale(const cv::Mat &input,
               const sensor_msgs::msg::CameraInfo &input_camera_info,
               cv::Mat &output,
               sensor_msgs::msg::CameraInfo &output_camera_info);

    /**
     * @brief Process image: rectify and scale (convenience method)
     * @param input Input image (any encoding)
     * @param output Output rectified and scaled image (same encoding)
     * @param output_camera_info Output scaled camera info
     * @return true if processing succeeded, false if calibration not ready
     */
    bool process(const cv::Mat &input, cv::Mat &output,
                 sensor_msgs::msg::CameraInfo &output_camera_info);

    /**
     * @brief Check if calibration is ready
     */
    bool isReady() const { return have_info_ && maps_valid_; }

private:
    void ensureMaps(int in_w, int in_h);
    static void scaleIntrinsics(sensor_msgs::msg::CameraInfo &ci, double sx, double sy);

    StereoRectifyScaleConfig config_;
    
    bool have_info_{false};
    bool maps_valid_{false};
    int last_in_w_{-1}, last_in_h_{-1};
    sensor_msgs::msg::CameraInfo last_info_;

    cv::Mat map1_, map2_;
};

} // namespace firefly_reconstruction

#endif // FIREFLY_RECONSTRUCTION_STEREO_RECTIFY_SCALE_HPP
