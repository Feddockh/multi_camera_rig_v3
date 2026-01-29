#include "firefly_reconstruction/semantic_pointcloud.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cstring>
#include <limits>

namespace firefly_reconstruction
{

SemanticPointCloud::SemanticPointCloud(const SemanticPointCloudConfig &config)
    : config_(config)
{
}

void SemanticPointCloud::updateCameraInfo(const sensor_msgs::msg::CameraInfo &info)
{
    fx_ = info.k[0];
    fy_ = info.k[4];
    cx_ = info.k[2];
    cy_ = info.k[5];
    img_w_ = info.width;
    img_h_ = info.height;
    have_info_ = true;
}

float SemanticPointCloud::packRGBFloat(uint8_t r, uint8_t g, uint8_t b)
{
    const uint32_t rgb = (static_cast<uint32_t>(r) << 16) |
                         (static_cast<uint32_t>(g) << 8) |
                         (static_cast<uint32_t>(b));
    float f;
    std::memcpy(&f, &rgb, sizeof(float));
    return f;
}

bool SemanticPointCloud::processDepthImage(
    const sensor_msgs::msg::Image &disp_msg,
    const std_msgs::msg::Header &header,
    sensor_msgs::msg::Image &depth_msg)
{
    if (!have_info_)
        return false;

    // Convert disparity image
    cv_bridge::CvImageConstPtr disp_cv;
    try
    {
        disp_cv = cv_bridge::toCvShare(std::make_shared<sensor_msgs::msg::Image>(disp_msg), "32FC1");
    }
    catch (const std::exception &)
    {
        return false;
    }

    const int out_h = disp_cv->image.rows;
    const int out_w = disp_cv->image.cols;
    const double fx = fx_;
    const double B = config_.baseline;
    const double maxR = config_.max_range_m;

    std::vector<float> depth((size_t)out_h * out_w, std::numeric_limits<float>::quiet_NaN());

    for (int v = 0; v < out_h; ++v)
    {
        const float *row = disp_cv->image.ptr<float>(v);
        float *drow = depth.data() + v * out_w;
        for (int u = 0; u < out_w; ++u)
        {
            const float disp = row[u];
            if (disp <= 0.0f)
                continue;
            const double Z = fx * B / (double)disp;
            if (Z <= 0.0 || Z > maxR)
                continue;
            drow[u] = (float)Z;
        }
    }

    // Apply depth filtering
    applyDepthFiltering(depth, out_h, out_w);

    depth_msg.header = header;
    depth_msg.height = out_h;
    depth_msg.width = out_w;
    depth_msg.encoding = "32FC1";
    depth_msg.is_bigendian = false;
    depth_msg.step = out_w * sizeof(float);
    depth_msg.data.resize(depth.size() * sizeof(float));
    std::memcpy(depth_msg.data.data(), depth.data(), depth.size() * sizeof(float));

    return true;
}

void SemanticPointCloud::applyDepthFiltering(std::vector<float> &depth, int out_h, int out_w)
{
    std::string dmode = config_.depth_filter_mode;
    std::transform(dmode.begin(), dmode.end(), dmode.begin(),
                   [](unsigned char c)
                   { return static_cast<char>(std::tolower(c)); });

    if (dmode == "median")
    {
        cv::Mat dmat(out_h, out_w, CV_32FC1, depth.data());
        cv::Mat valid = (dmat == dmat) & (dmat > 0.0f);
        cv::Mat tmp = dmat.clone();
        tmp.setTo(0.0f, ~valid);

        int k = std::max(3, config_.depth_median_ksize | 1);
        cv::medianBlur(tmp, tmp, k);

        tmp.setTo(std::numeric_limits<float>::quiet_NaN(), ~valid);
        std::memcpy(depth.data(), tmp.ptr<float>(0), depth.size() * sizeof(float));
    }
    else if (dmode == "flying_pixel")
    {
        cv::Mat dmat(out_h, out_w, CV_32FC1, depth.data());

        int k = std::max(3, config_.depth_flying_ksize | 1);
        int r = k / 2;
        const double tau = std::max(0.0, config_.depth_flying_tau);

        cv::Mat tmp = dmat.clone();
        for (int v = 0; v < tmp.rows; ++v)
        {
            float *row = tmp.ptr<float>(v);
            for (int u = 0; u < tmp.cols; ++u)
            {
                const float z = row[u];
                if (!(z == z) || z <= 0.0f)
                    row[u] = 0.0f;
            }
        }

        cv::Mat med;
        cv::medianBlur(tmp, med, k);

        for (int v = r; v < dmat.rows - r; ++v)
        {
            float *zrow = dmat.ptr<float>(v);
            const float *mrow = med.ptr<float>(v);

            for (int u = r; u < dmat.cols - r; ++u)
            {
                const float z = zrow[u];
                if (!(z == z) || z <= 0.0f)
                    continue;

                const float m = mrow[u];
                if (!(m > 0.0f))
                    continue;

                int valid_n = 0;
                for (int yy = v - r; yy <= v + r; ++yy)
                {
                    const float *rr = dmat.ptr<float>(yy);
                    for (int xx = u - r; xx <= u + r; ++xx)
                    {
                        const float zz = rr[xx];
                        if ((zz == zz) && zz > 0.0f)
                            valid_n++;
                    }
                }
                if (valid_n < config_.depth_flying_min_neighbors)
                    continue;

                const double rel = std::abs((double)z - (double)m) / (double)m;
                if (rel > tau)
                {
                    zrow[u] = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }
    }
}

bool SemanticPointCloud::processNormalPointCloud(
    const sensor_msgs::msg::Image &disp_msg,
    const sensor_msgs::msg::Image &image_msg,
    const std_msgs::msg::Header &header,
    sensor_msgs::msg::PointCloud2 &cloud_msg)
{
    if (!have_info_)
        return false;

    // Convert disparity image
    cv_bridge::CvImageConstPtr disp_cv;
    try
    {
        disp_cv = cv_bridge::toCvShare(std::make_shared<sensor_msgs::msg::Image>(disp_msg), "32FC1");
    }
    catch (const std::exception &)
    {
        return false;
    }

    // Convert RGB image
    cv_bridge::CvImageConstPtr image_cv;
    try
    {
        auto img_ptr = std::make_shared<sensor_msgs::msg::Image>(image_msg);
        image_cv = cv_bridge::toCvShare(img_ptr, image_msg.encoding);
    }
    catch (const std::exception &)
    {
        return false;
    }

    const int out_h = disp_cv->image.rows;
    const int out_w = disp_cv->image.cols;

    // Verify image and disparity sizes match
    if (image_cv->image.rows != out_h || image_cv->image.cols != out_w)
        return false;

    const double fx = fx_;
    const double fy = fy_;
    const double cx = cx_;
    const double cy = cy_;
    const double B = config_.baseline;
    const double maxR = config_.max_range_m;
    const int s = std::max(1, config_.stride);

    // Apply pointcloud filtering to get keep mask
    std::vector<uint8_t> keep_mask((size_t)out_h * out_w, 1);
    applyPointCloudFiltering(disp_cv->image, fx, fy, cx, cy, B, maxR, s, out_h, out_w, keep_mask);

    // Count valid points
    size_t n_valid = 0;
    for (int v = 0; v < out_h; v += s)
    {
        const float *row = disp_cv->image.ptr<float>(v);
        for (int u = 0; u < out_w; u += s)
        {
            if (keep_mask[(size_t)v * out_w + u] == 0)
                continue;

            const float d = row[u];
            if (d <= 0.0f)
            {
                if (config_.use_background)
                    n_valid++;
                continue;
            }

            const double Z = fx * B / static_cast<double>(d);
            if (Z <= 0.0 || Z > maxR)
            {
                if (config_.use_background)
                    n_valid++;
                continue;
            }

            n_valid++;
        }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(n_valid);
    cloud.is_bigendian = false;
    cloud.is_dense = false;

    // Define xyz + rgb fields
    cloud.fields.resize(4);
    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0;
    cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[0].count = 1;

    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4;
    cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[1].count = 1;

    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8;
    cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[2].count = 1;

    cloud.fields[3].name = "rgb";
    cloud.fields[3].offset = 12;
    cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[3].count = 1;

    cloud.point_step = 16; // 4 floats: x,y,z,rgb
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step);

    // Fill packed data
    size_t idx = 0;
    for (int v = 0; v < out_h; v += s)
    {
        const float *row = disp_cv->image.ptr<float>(v);
        for (int u = 0; u < out_w; u += s)
        {
            if (keep_mask[(size_t)v * out_w + u] == 0)
                continue;

            const float d = row[u];
            bool use_max_range = false;

            if (d <= 0.0f)
            {
                if (config_.use_background)
                    use_max_range = true;
                else
                    continue;
            }

            double Z = 0.0;
            if (!use_max_range)
            {
                Z = fx * B / static_cast<double>(d);
                if (Z <= 0.0)
                {
                    if (config_.use_background)
                        use_max_range = true;
                    else
                        continue;
                }
                else if (Z > maxR)
                {
                    if (config_.use_background)
                        Z = maxR;
                    else
                        continue;
                }
            }
            else
            {
                Z = maxR;
            }

            const float X = static_cast<float>((static_cast<double>(u) - cx) * Z / fx);
            const float Y = static_cast<float>((static_cast<double>(v) - cy) * Z / fy);
            const float Zf = static_cast<float>(Z);

            // Sample color from image
            const cv::Vec3b bgr = image_cv->image.at<cv::Vec3b>(v, u);
            const float rgb_f = packRGBFloat(bgr[2], bgr[1], bgr[0]);

            uint8_t *ptr = cloud.data.data() + idx * cloud.point_step;
            std::memcpy(ptr + 0, &X, sizeof(float));
            std::memcpy(ptr + 4, &Y, sizeof(float));
            std::memcpy(ptr + 8, &Zf, sizeof(float));
            std::memcpy(ptr + 12, &rgb_f, sizeof(float));

            idx++;
        }
    }

    cloud_msg = cloud;
    return true;
}

bool SemanticPointCloud::processSemanticPointCloud(
    const sensor_msgs::msg::Image &disp_msg,
    const sensor_msgs::msg::Image &image_msg,
    const std_msgs::msg::Header &header,
    sensor_msgs::msg::PointCloud2 &cloud_msg)
{
    if (!have_info_)
        return false;

    // Convert disparity image
    cv_bridge::CvImageConstPtr disp_cv;
    try
    {
        disp_cv = cv_bridge::toCvShare(std::make_shared<sensor_msgs::msg::Image>(disp_msg), "32FC1");
    }
    catch (const std::exception &)
    {
        return false;
    }

    // Convert RGB image
    cv_bridge::CvImageConstPtr image_cv;
    try
    {
        auto img_ptr = std::make_shared<sensor_msgs::msg::Image>(image_msg);
        image_cv = cv_bridge::toCvShare(img_ptr, image_msg.encoding);
    }
    catch (const std::exception &)
    {
        return false;
    }

    const int out_h = disp_cv->image.rows;
    const int out_w = disp_cv->image.cols;

    // Verify image and disparity sizes match
    if (image_cv->image.rows != out_h || image_cv->image.cols != out_w)
        return false;

    const double fx = fx_;
    const double fy = fy_;
    const double cx = cx_;
    const double cy = cy_;
    const double B = config_.baseline;
    const double maxR = config_.max_range_m;
    const int s = std::max(1, config_.stride);

    // TODO: Implement detection-based semantic labeling
    // For now, create semantic labels and confidences arrays filled with background values
    const size_t N = (size_t)out_h * out_w;
    std::vector<int32_t> labels(N, config_.background_class_id);
    std::vector<float> confidences(N, static_cast<float>(config_.background_confidence));

    // TODO: When detection message is available:
    // 1. Scale detections from original resolution to current resolution
    // 2. For each detection bounding box, assign class_id and confidence
    //    to pixels within the box (highest confidence wins for overlaps)

    // Apply pointcloud filtering
    std::vector<uint8_t> keep_mask((size_t)out_h * out_w, 1);
    applyPointCloudFiltering(disp_cv->image, fx, fy, cx, cy, B, maxR, s, out_h, out_w, keep_mask);

    // Count valid points
    size_t n_valid = 0;
    for (int v = 0; v < out_h; v += s)
    {
        const float *row = disp_cv->image.ptr<float>(v);
        for (int u = 0; u < out_w; u += s)
        {
            if (keep_mask[(size_t)v * out_w + u] == 0)
                continue;

            const float d = row[u];
            if (d <= 0.0f)
            {
                if (config_.use_background)
                    n_valid++;
                continue;
            }

            const double Z = fx * B / static_cast<double>(d);
            if (Z <= 0.0 || Z > maxR)
            {
                if (config_.use_background)
                    n_valid++;
                continue;
            }

            n_valid++;
        }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(n_valid);
    cloud.is_bigendian = false;
    cloud.is_dense = false;

    // Define fields for semantic pointcloud: x, y, z, rgb, class_id, confidence
    cloud.fields.resize(6);

    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0;
    cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[0].count = 1;

    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4;
    cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[1].count = 1;

    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8;
    cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[2].count = 1;

    cloud.fields[3].name = "rgb";
    cloud.fields[3].offset = 12;
    cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[3].count = 1;

    cloud.fields[4].name = "class_id";
    cloud.fields[4].offset = 16;
    cloud.fields[4].datatype = sensor_msgs::msg::PointField::INT32;
    cloud.fields[4].count = 1;

    cloud.fields[5].name = "confidence";
    cloud.fields[5].offset = 20;
    cloud.fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[5].count = 1;

    cloud.point_step = 24; // 6 fields: x,y,z,rgb(float),class_id(int32),confidence(float)
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step);

    // Fill packed data
    size_t idx = 0;
    for (int v = 0; v < out_h; v += s)
    {
        const float *row = disp_cv->image.ptr<float>(v);
        for (int u = 0; u < out_w; u += s)
        {
            if (keep_mask[(size_t)v * out_w + u] == 0)
                continue;

            const float d = row[u];
            bool use_max_range = false;

            if (d <= 0.0f)
            {
                if (config_.use_background)
                    use_max_range = true;
                else
                    continue;
            }

            double Z = 0.0;
            if (!use_max_range)
            {
                Z = fx * B / static_cast<double>(d);
                if (Z <= 0.0)
                {
                    if (config_.use_background)
                        use_max_range = true;
                    else
                        continue;
                }
                else if (Z > maxR)
                {
                    if (config_.use_background)
                        Z = maxR;
                    else
                        continue;
                }
            }
            else
            {
                Z = maxR;
            }

            const float X = static_cast<float>((static_cast<double>(u) - cx) * Z / fx);
            const float Y = static_cast<float>((static_cast<double>(v) - cy) * Z / fy);
            const float Zf = static_cast<float>(Z);

            // Sample color from image
            const cv::Vec3b bgr = image_cv->image.at<cv::Vec3b>(v, u);
            const float rgb_f = packRGBFloat(bgr[2], bgr[1], bgr[0]);

            // Get semantic info for this pixel
            const size_t pixel_idx = (size_t)v * out_w + u;
            const int32_t class_id = labels[pixel_idx];
            const float conf = confidences[pixel_idx];

            uint8_t *ptr = cloud.data.data() + idx * cloud.point_step;
            std::memcpy(ptr + 0, &X, sizeof(float));
            std::memcpy(ptr + 4, &Y, sizeof(float));
            std::memcpy(ptr + 8, &Zf, sizeof(float));
            std::memcpy(ptr + 12, &rgb_f, sizeof(float));
            std::memcpy(ptr + 16, &class_id, sizeof(int32_t));
            std::memcpy(ptr + 20, &conf, sizeof(float));

            idx++;
        }
    }

    cloud_msg = cloud;
    return true;
}

void SemanticPointCloud::applyPointCloudFiltering(
    const cv::Mat &disp_img,
    double fx, double fy, double cx, double cy,
    double B, double maxR, int s,
    int out_h, int out_w,
    std::vector<uint8_t> &keep_mask)
{
    std::string pcmode = config_.pc_filter_mode;
    std::transform(pcmode.begin(), pcmode.end(), pcmode.begin(),
                   [](unsigned char c)
                   { return static_cast<char>(std::tolower(c)); });

    if (pcmode == "grid_outlier")
    {
        int k = std::max(3, config_.pc_grid_ksize | 1);
        int r = k / 2;
        const double tau = std::max(0.0, config_.pc_grid_tau);

        // Build depth image from disparity
        std::vector<float> zimg((size_t)out_h * out_w, std::numeric_limits<float>::quiet_NaN());
        for (int v = 0; v < out_h; ++v)
        {
            const float *row = disp_img.ptr<float>(v);
            float *zrow = zimg.data() + v * out_w;
            for (int u = 0; u < out_w; ++u)
            {
                const float d = row[u];
                if (d <= 0.0f)
                    continue;
                const double Z = fx * B / (double)d;
                if (Z <= 0.0 || Z > maxR)
                    continue;
                zrow[u] = (float)Z;
            }
        }

        cv::Mat zmat(out_h, out_w, CV_32FC1, zimg.data());
        cv::Mat tmp = zmat.clone();
        for (int v = 0; v < out_h; ++v)
        {
            float *row = tmp.ptr<float>(v);
            for (int u = 0; u < out_w; ++u)
            {
                const float z = row[u];
                if (!(z == z) || z <= 0.0f)
                    row[u] = 0.0f;
            }
        }
        cv::Mat med;
        cv::medianBlur(tmp, med, k);

        for (int v = r; v < out_h - r; ++v)
        {
            for (int u = r; u < out_w - r; ++u)
            {
                const float z = zimg[(size_t)v * out_w + u];
                if (!(z == z) || z <= 0.0f)
                    continue;

                const float m = med.at<float>(v, u);
                if (!(m > 0.0f))
                    continue;

                int valid_n = 0;
                for (int yy = v - r; yy <= v + r; ++yy)
                {
                    for (int xx = u - r; xx <= u + r; ++xx)
                    {
                        const float zz = zimg[(size_t)yy * out_w + xx];
                        if ((zz == zz) && zz > 0.0f)
                            valid_n++;
                    }
                }
                if (valid_n < config_.pc_grid_min_neighbors)
                    continue;

                const double rel = std::abs((double)z - (double)m) / (double)m;
                if (rel > tau)
                {
                    keep_mask[(size_t)v * out_w + u] = 0;
                }
            }
        }
    }
    // NOTE: knn_outlier filter removed for simplicity - can be added if needed
}

} // namespace firefly_reconstruction
