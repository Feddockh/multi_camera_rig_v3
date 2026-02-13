#include "multi_camera_rig_reconstruction/foundation_stereo_matcher.hpp"
#include "multi_camera_rig_common/qos_utils.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <stdexcept>
#include <algorithm>

namespace multi_camera_rig_reconstruction
{

FoundationStereoMatcher::FoundationStereoMatcher(const FoundationStereoMatcherConfig &config)
    : config_(config)
{
    if (config_.engine_path.empty())
        throw std::runtime_error("engine_path is empty");

    runner_ = std::make_unique<multi_camera_rig_common::TrtRunner>(config_.engine_path, true);

    const auto in = runner_->shapeOf("left");  // e.g. (1,3,672,896)
    in_h_ = in.d[2];
    in_w_ = in.d[3];

    const auto out = runner_->shapeOf("disp"); // e.g. (1,1,672,896)
    out_h_ = out.d[2];
    out_w_ = out.d[3];

    // Pre-allocate host buffers
    left_nchw_.resize(3 * in_h_ * in_w_);
    right_nchw_.resize(3 * in_h_ * in_w_);
    disp_buffer_.resize(static_cast<size_t>(out_h_ * out_w_));
}

void FoundationStereoMatcher::bgrToNCHWFloat(const cv::Mat &bgr, int H, int W, std::vector<float> &out)
{
    // out: C*H*W contiguous floats
    const int HW = H * W;
    out.resize(3 * HW);

    const uint8_t *p = bgr.ptr<uint8_t>(0);
    float *outB = out.data() + 0 * HW;
    float *outG = out.data() + 1 * HW;
    float *outR = out.data() + 2 * HW;

    for (int i = 0; i < HW; ++i)
    {
        outB[i] = static_cast<float>(p[3 * i + 0]);
        outG[i] = static_cast<float>(p[3 * i + 1]);
        outR[i] = static_cast<float>(p[3 * i + 2]);
    }
}

void FoundationStereoMatcher::applyDisparityFilter(cv::Mat &disparity)
{
    auto mode = multi_camera_rig_common::toLower(config_.disp_filter_mode);
    
    if (mode == "none")
        return;

    // Normalize invalids: treat <=0 as invalid
    for (int v = 0; v < disparity.rows; ++v)
    {
        float *row = disparity.ptr<float>(v);
        for (int u = 0; u < disparity.cols; ++u)
        {
            if (!(row[u] > 0.0f))
                row[u] = 0.0f;
        }
    }

    if (mode == "speckle")
    {
        // Convert float disparity -> fixed-point, filter speckles, convert back
        const double S = std::max(1.0, config_.speckle_scale);
        cv::Mat disp_16s(disparity.size(), CV_16S);

        for (int v = 0; v < disparity.rows; ++v)
        {
            const float *src = disparity.ptr<float>(v);
            int16_t *dst = disp_16s.ptr<int16_t>(v);
            for (int u = 0; u < disparity.cols; ++u)
            {
                const float d = src[u];
                dst[u] = (d > 0.0f) ? (int16_t)std::lround(d * S) : (int16_t)0;
            }
        }

        // Remove small connected components with similar disparity values
        cv::filterSpeckles(
            disp_16s,
            0,                                                // newVal for removed speckles
            config_.speckle_max_size,                        // max speckle size
            (int)std::lround(config_.speckle_range * S)     // max disparity variation within speckle
        );

        // Convert back to float disparity
        for (int v = 0; v < disparity.rows; ++v)
        {
            float *dst = disparity.ptr<float>(v);
            const int16_t *src = disp_16s.ptr<int16_t>(v);
            for (int u = 0; u < disparity.cols; ++u)
            {
                const int16_t q = src[u];
                dst[u] = (q > 0) ? (float)((double)q / S) : 0.0f;
            }
        }
    }
}

void FoundationStereoMatcher::process(const cv::Mat &left_bgr, const cv::Mat &right_bgr, cv::Mat &disparity)
{
    if (left_bgr.cols != in_w_ || left_bgr.rows != in_h_ ||
        right_bgr.cols != in_w_ || right_bgr.rows != in_h_)
    {
        throw std::runtime_error("Input image size does not match engine input");
    }

    // Convert to NCHW format
    bgrToNCHWFloat(left_bgr, in_h_, in_w_, left_nchw_);
    bgrToNCHWFloat(right_bgr, in_h_, in_w_, right_nchw_);

    // Run inference using new multi-tensor API
    multi_camera_rig_common::TensorSpec left_spec{"left", left_nchw_.data(), nullptr, left_nchw_.size()};
    multi_camera_rig_common::TensorSpec right_spec{"right", right_nchw_.data(), nullptr, right_nchw_.size()};
    multi_camera_rig_common::TensorSpec disp_spec{"disp", nullptr, disp_buffer_.data(), disp_buffer_.size()};
    runner_->run({left_spec, right_spec}, {disp_spec});

    // Copy to output mat

    disparity = cv::Mat(out_h_, out_w_, CV_32FC1, disp_buffer_.data()).clone();

    // Apply filtering
    applyDisparityFilter(disparity);
}

} // namespace multi_camera_rig_reconstruction
