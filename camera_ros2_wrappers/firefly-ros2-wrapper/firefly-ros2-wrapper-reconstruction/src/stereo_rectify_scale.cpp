#include "firefly_reconstruction/stereo_rectify_scale.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

namespace firefly_reconstruction
{

StereoRectifyScale::StereoRectifyScale(const StereoRectifyScaleConfig &config)
    : config_(config)
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

bool StereoRectifyScale::process(const cv::Mat &input, cv::Mat &output,
                                  sensor_msgs::msg::CameraInfo &output_camera_info)
{
    if (!have_info_)
        return false;

    // Recompute maps if needed
    ensureMaps(input.cols, input.rows);
    if (!maps_valid_)
        return false;

    // Rectify
    cv::Mat rectified;
    cv::remap(input, rectified, map1_, map2_, config_.interpolation);

    // Scale
    cv::resize(rectified, output, cv::Size(config_.output_width, config_.output_height),
               0, 0, config_.interpolation);

    // Create scaled CameraInfo
    output_camera_info = last_info_;
    output_camera_info.width = config_.output_width;
    output_camera_info.height = config_.output_height;

    // For rectified images, D is usually zeroed and R is identity
    output_camera_info.d.assign(output_camera_info.d.size(), 0.0);
    output_camera_info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    // Scale intrinsics from input image size -> output size
    const double sx = static_cast<double>(config_.output_width) / static_cast<double>(input.cols);
    const double sy = static_cast<double>(config_.output_height) / static_cast<double>(input.rows);
    scaleIntrinsics(output_camera_info, sx, sy);

    return true;
}

} // namespace firefly_reconstruction
