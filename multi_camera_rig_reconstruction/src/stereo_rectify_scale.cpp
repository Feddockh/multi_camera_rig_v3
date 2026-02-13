#include "multi_camera_rig_reconstruction/stereo_rectify_scale.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

namespace multi_camera_rig_reconstruction
{

StereoRectifyScale::StereoRectifyScale(int output_width, int output_height, int interpolation)
    : output_width_(output_width), output_height_(output_height), interpolation_(interpolation)
{
}

void StereoRectifyScale::updateCameraInfo(const sensor_msgs::msg::CameraInfo &camera_info)
{
    last_info_ = camera_info;
    have_info_ = true;
    maps_valid_ = false; // new calibration => recompute maps
}

void StereoRectifyScale::ensureMaps(int in_w, int in_h)
{
    if (!have_info_)
        return;
    if (maps_valid_ && in_w == last_in_w_ && in_h == last_in_h_)
        return;

    // Build OpenCV matrices from CameraInfo
    cv::Mat K(3, 3, CV_64F, (void *)last_info_.k.data());
    cv::Mat D((int)last_info_.d.size(), 1, CV_64F, (void *)last_info_.d.data());
    cv::Mat R(3, 3, CV_64F, (void *)last_info_.r.data());
    cv::Mat P(3, 4, CV_64F, (void *)last_info_.p.data());

    cv::Size in_size(in_w, in_h);

    // For rectified images, use the left 3x3 of P as the new camera matrix
    cv::Mat P3 = P(cv::Rect(0, 0, 3, 3)).clone();

    cv::initUndistortRectifyMap(
        K, D, R, P3, in_size, CV_32FC1,
        map1_, map2_);

    last_in_w_ = in_w;
    last_in_h_ = in_h;
    maps_valid_ = true;
}

void StereoRectifyScale::scaleIntrinsics(sensor_msgs::msg::CameraInfo &ci, double sx, double sy)
{
    // K (row-major)
    ci.k[0] *= sx; // fx
    ci.k[2] *= sx; // cx
    ci.k[4] *= sy; // fy
    ci.k[5] *= sy; // cy

    // P (row-major 3x4): scale fx, fy, cx, cy, and also Tx if present
    ci.p[0] *= sx; // fx
    ci.p[2] *= sx; // cx
    ci.p[3] *= sx; // Tx (baseline term in pixels)
    ci.p[5] *= sy; // fy
    ci.p[6] *= sy; // cy
}

bool StereoRectifyScale::rectify(const cv::Mat &input, cv::Mat &output,
                                  sensor_msgs::msg::CameraInfo &output_camera_info)
{
    if (!have_info_)
        return false;

    // Recompute maps if needed
    ensureMaps(input.cols, input.rows);
    if (!maps_valid_)
        return false;

    // Rectify
    cv::remap(input, output, map1_, map2_, interpolation_);

    // Create rectified CameraInfo
    output_camera_info = last_info_;
    output_camera_info.width = input.cols;
    output_camera_info.height = input.rows;

    // For rectified images, D is usually zeroed and R is identity
    output_camera_info.d.assign(output_camera_info.d.size(), 0.0);
    output_camera_info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    return true;
}

bool StereoRectifyScale::scale(const cv::Mat &input,
                                const sensor_msgs::msg::CameraInfo &input_camera_info,
                                cv::Mat &output,
                                sensor_msgs::msg::CameraInfo &output_camera_info)
{
    // Scale image
    cv::resize(input, output, cv::Size(output_width_, output_height_),
               0, 0, interpolation_);

    // Create scaled CameraInfo
    output_camera_info = input_camera_info;
    output_camera_info.width = output_width_;
    output_camera_info.height = output_height_;

    // Scale intrinsics from input image size -> output size
    const double sx = static_cast<double>(output_width_) / static_cast<double>(input.cols);
    const double sy = static_cast<double>(output_height_) / static_cast<double>(input.rows);
    scaleIntrinsics(output_camera_info, sx, sy);

    return true;
}

bool StereoRectifyScale::process(const cv::Mat &input, cv::Mat &output,
                                  sensor_msgs::msg::CameraInfo &output_camera_info)
{
    // Rectify first
    cv::Mat rectified;
    sensor_msgs::msg::CameraInfo rect_info;
    if (!rectify(input, rectified, rect_info))
        return false;

    // Then scale
    return scale(rectified, rect_info, output, output_camera_info);
}

} // namespace multi_camera_rig_reconstruction
