#include "multi_camera_rig_reconstruction/semantic_pointcloud.hpp"
#include "multi_camera_rig_msgs/msg/instance_segmentation2_d_array.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cstring>
#include <limits>
#include <iostream>

namespace multi_camera_rig_reconstruction
{

static inline int clampi(int v, int lo, int hi)
{
    return std::max(lo, std::min(v, hi));
}

SemanticPointCloud::SemanticPointCloud(const SemanticPointCloudConfig &config, bool debug, rclcpp::Logger logger)
    : config_(config), debug_(debug), logger_(logger)
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

float SemanticPointCloud::classIdToRGBFloat(int32_t class_id)
{
    // Generate distinct colors for different class IDs using a color palette
    // Use a simple hashing scheme to generate colors
    if (class_id < 0)
    {
        // Background: gray
        return packRGBFloat(128, 128, 128);
    }
    
    // Use a predefined color palette for common classes (0-19)
    // Based on a distinct color scheme
    const uint8_t palette[][3] = {
        {0, 150, 255},    // Blue - class 0
        {230, 25, 75},    // Red - class 1
        {60, 180, 75},    // Green - class 2
        {255, 225, 25},   // Yellow - class 3
        {245, 130, 48},   // Orange - class 4
        {145, 30, 180},   // Purple - class 5
        {70, 240, 240},   // Cyan - class 6
        {240, 50, 230},   // Magenta - class 7
        {210, 245, 60},   // Lime - class 8
        {250, 190, 212},  // Pink - class 9
        {0, 128, 128},    // Teal - class 10
        {220, 190, 255},  // Lavender - class 11
        {170, 110, 40},   // Brown - class 12
        {255, 250, 200},  // Beige - class 13
        {128, 0, 0},      // Maroon - class 14
        {170, 255, 195},  // Mint - class 15
        {128, 128, 0},    // Olive - class 16
        {255, 215, 180},  // Coral - class 17
        {0, 0, 128},      // Navy - class 18
        {128, 128, 128}   // Grey - class 19
    };
    
    if (class_id < 20)
    {
        return packRGBFloat(palette[class_id][0], palette[class_id][1], palette[class_id][2]);
    }
    
    // For class IDs >= 20, use a hash function to generate colors
    uint32_t hash = (uint32_t)class_id * 2654435761u;  // Knuth's multiplicative hash
    uint8_t r = (hash >> 16) & 0xFF;
    uint8_t g = (hash >> 8) & 0xFF;
    uint8_t b = hash & 0xFF;
    
    // Ensure colors are not too dark
    r = (r < 50) ? r + 100 : r;
    g = (g < 50) ? g + 100 : g;
    b = (b < 50) ? b + 100 : b;
    
    return packRGBFloat(r, g, b);
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

bool SemanticPointCloud::processNormalPointCloud(
    const sensor_msgs::msg::Image &disp_msg,
    const sensor_msgs::msg::Image &image_msg,
    const std_msgs::msg::Header &header,
    sensor_msgs::msg::PointCloud2 &cloud_msg)
{
    if (debug_)
        RCLCPP_INFO(logger_, "[SemanticPointCloud] processNormalPointCloud called");

    if (!have_info_)
    {
        if (debug_)
            RCLCPP_WARN(logger_, "[SemanticPointCloud] No camera info available");
        return false;
    }

    // Convert disparity image
    cv_bridge::CvImageConstPtr disp_cv;
    try
    {
        if (debug_)
            RCLCPP_INFO(logger_, "[SemanticPointCloud] Converting disparity (encoding: %s)", disp_msg.encoding.c_str());
        disp_cv = cv_bridge::toCvShare(std::make_shared<sensor_msgs::msg::Image>(disp_msg), "32FC1");
        if (debug_)
            RCLCPP_INFO(logger_, "[SemanticPointCloud] Disparity converted successfully");
    }
    catch (const std::exception &e)
    {
        if (debug_)
            RCLCPP_ERROR(logger_, "[SemanticPointCloud] Failed to convert disparity: %s", e.what());
        return false;
    }

    // Convert RGB image
    cv_bridge::CvImageConstPtr image_cv;
    try
    {
        if (debug_)
            RCLCPP_INFO(logger_, "[SemanticPointCloud] Converting RGB image (encoding: %s)", image_msg.encoding.c_str());
        auto img_ptr = std::make_shared<sensor_msgs::msg::Image>(image_msg);
        image_cv = cv_bridge::toCvShare(img_ptr, image_msg.encoding);
        if (debug_)
            RCLCPP_INFO(logger_, "[SemanticPointCloud] RGB image converted successfully");
    }
    catch (const std::exception &e)
    {
        if (debug_)
            RCLCPP_ERROR(logger_, "[SemanticPointCloud] Failed to convert RGB image: %s", e.what());
        return false;
    }

    const int out_h = disp_cv->image.rows;
    const int out_w = disp_cv->image.cols;

    // Verify image and disparity sizes match
    if (image_cv->image.rows != out_h || image_cv->image.cols != out_w)
    {
        if (debug_)
            RCLCPP_ERROR(logger_, "[SemanticPointCloud] Size mismatch: image %dx%d vs disparity %dx%d",
                        image_cv->image.rows, image_cv->image.cols, out_h, out_w);
        return false;
    }

    const double fx = fx_;
    const double fy = fy_;
    const double cx = cx_;
    const double cy = cy_;
    const double B = config_.baseline;
    const double maxR = config_.max_range_m;
    const int s = std::max(1, config_.stride);

    if (debug_)
        RCLCPP_INFO(logger_, "[SemanticPointCloud] Starting point counting (stride=%d, maxR=%.2f)", s, maxR);

    // Count valid points
    size_t n_valid = 0;
    for (int v = 0; v < out_h; v += s)
    {
        const float *row = disp_cv->image.ptr<float>(v);
        for (int u = 0; u < out_w; u += s)
        {
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
    const std::vector<vision_msgs::msg::Detection2D> &detections,
    double scale_x,
    double scale_y,
    const std_msgs::msg::Header &header,
    sensor_msgs::msg::PointCloud2 &cloud_msg)
{
    if (debug_)
        RCLCPP_INFO(logger_, "[SemanticPointCloud] processSemanticPointCloud called with %zu detections", detections.size());

    if (!have_info_)
    {
        if (debug_)
            RCLCPP_WARN(logger_, "[SemanticPointCloud] No camera info available");
        return false;
    }

    // Convert disparity image
    cv_bridge::CvImageConstPtr disp_cv;
    try
    {
        if (debug_)
            RCLCPP_INFO(logger_, "[SemanticPointCloud] Converting disparity (encoding: %s)", disp_msg.encoding.c_str());
        disp_cv = cv_bridge::toCvShare(std::make_shared<sensor_msgs::msg::Image>(disp_msg), "32FC1");
        if (debug_)
            RCLCPP_INFO(logger_, "[SemanticPointCloud] Disparity converted successfully");
    }
    catch (const std::exception &e)
    {
        if (debug_)
            RCLCPP_ERROR(logger_, "[SemanticPointCloud] Failed to convert disparity: %s", e.what());
        return false;
    }

    // Convert RGB image
    cv_bridge::CvImageConstPtr image_cv;
    try
    {
        if (debug_)
            RCLCPP_INFO(logger_, "[SemanticPointCloud] Converting RGB image (encoding: %s)", image_msg.encoding.c_str());
        auto img_ptr = std::make_shared<sensor_msgs::msg::Image>(image_msg);
        image_cv = cv_bridge::toCvShare(img_ptr, image_msg.encoding);
        if (debug_)
            RCLCPP_INFO(logger_, "[SemanticPointCloud] RGB image converted successfully");
    }
    catch (const std::exception &e)
    {
        if (debug_)
            RCLCPP_ERROR(logger_, "[SemanticPointCloud] Failed to convert RGB image: %s", e.what());
        return false;
    }

    const int out_h = disp_cv->image.rows;
    const int out_w = disp_cv->image.cols;

    // Verify image and disparity sizes match
    if (image_cv->image.rows != out_h || image_cv->image.cols != out_w)
    {
        if (debug_)
            RCLCPP_ERROR(logger_, "[SemanticPointCloud] Size mismatch: image %dx%d vs disparity %dx%d",
                        image_cv->image.rows, image_cv->image.cols, out_h, out_w);
        return false;
    }

    // Create semantic labels and confidences arrays (initialized to background)
    const size_t N = (size_t)out_h * out_w;
    std::vector<int32_t> labels(N, config_.background_class_id);
    std::vector<float> confidences(N, static_cast<float>(config_.background_confidence));

    if (debug_)
        RCLCPP_INFO(logger_, "[SemanticPointCloud] Initialized labels/confidences array with %zu elements", N);

    // Sort detections by confidence (ascending) so higher confidence overwrites lower
    std::vector<vision_msgs::msg::Detection2D> sorted_dets = detections;
    std::sort(sorted_dets.begin(), sorted_dets.end(),
              [](const auto &a, const auto &b) {
                  return a.results[0].hypothesis.score < b.results[0].hypothesis.score;
              });

    // Scale detections and assign labels
    for (const auto &det : sorted_dets)
    {
        // Extract bounding box in original resolution
        const float orig_cx = det.bbox.center.position.x;
        const float orig_cy = det.bbox.center.position.y;
        const float orig_w = det.bbox.size_x;
        const float orig_h = det.bbox.size_y;

        // Scale to rectified/scaled resolution
        const float cx = orig_cx * scale_x;
        const float cy = orig_cy * scale_y;
        const float w = orig_w * scale_x;
        const float h = orig_h * scale_y;

        // Convert center+size to corners
        int x1 = static_cast<int>(cx - w * 0.5f);
        int y1 = static_cast<int>(cy - h * 0.5f);
        int x2 = static_cast<int>(cx + w * 0.5f);
        int y2 = static_cast<int>(cy + h * 0.5f);

        // Clip to image bounds
        x1 = std::max(0, x1);
        y1 = std::max(0, y1);
        x2 = std::min(out_w - 1, x2);
        y2 = std::min(out_h - 1, y2);

        const int32_t class_id = std::stoi(det.results[0].hypothesis.class_id);
        const float conf = det.results[0].hypothesis.score;

        if (debug_)
            RCLCPP_INFO(logger_, "[SemanticPointCloud] Detection: class=%d conf=%.3f bbox=[%d,%d,%d,%d] (scaled from [%.1f,%.1f,%.1f,%.1f])",
                       class_id, conf, x1, y1, x2, y2,
                       orig_cx - orig_w*0.5f, orig_cy - orig_h*0.5f,
                       orig_cx + orig_w*0.5f, orig_cy + orig_h*0.5f);

        // Assign class_id and confidence to all pixels in bounding box
        for (int v = y1; v <= y2; ++v)
        {
            for (int u = x1; u <= x2; ++u)
            {
                const size_t idx = (size_t)v * out_w + u;
                labels[idx] = class_id;
                confidences[idx] = conf;
            }
        }
    }

    if (debug_)
        RCLCPP_INFO(logger_, "[SemanticPointCloud] Finished assigning labels from %zu detections", detections.size());

    // Now generate point cloud with semantic labels
    const double fx = fx_;
    const double fy = fy_;
    const double cx = cx_;
    const double cy = cy_;
    const double B = config_.baseline;
    const double maxR = config_.max_range_m;
    const int s = std::max(1, config_.stride);

    if (debug_)
        RCLCPP_INFO(logger_, "[SemanticPointCloud] Starting point counting (stride=%d, maxR=%.2f)", s, maxR);

    // Count valid points
    size_t n_valid = 0;
    for (int v = 0; v < out_h; v += s)
    {
        const float *row = disp_cv->image.ptr<float>(v);
        for (int u = 0; u < out_w; u += s)
        {
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

    if (debug_)
        RCLCPP_INFO(logger_, "[SemanticPointCloud] Found %zu valid points", n_valid);

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

            // Get semantic info for this pixel
            const size_t pixel_idx = (size_t)v * out_w + u;
            const int32_t class_id = labels[pixel_idx];
            const float conf = confidences[pixel_idx];

            // Determine color: either from image RGB or from class ID
            float rgb_f;
            if (config_.color_by_class)
            {
                // Color by class ID
                rgb_f = classIdToRGBFloat(class_id);
            }
            else
            {
                // Color by image RGB
                const cv::Vec3b rgb = image_cv->image.at<cv::Vec3b>(v, u);
                rgb_f = packRGBFloat(rgb[0], rgb[1], rgb[2]);
            }

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

    if (debug_)
        RCLCPP_INFO(logger_, "[SemanticPointCloud] Created semantic point cloud with %zu points", idx);

    cloud_msg = cloud;
    return true;
}

bool SemanticPointCloud::processSemanticPointCloudInstances(
    const sensor_msgs::msg::Image &disp_msg,
    const sensor_msgs::msg::Image &image_msg,
    const multi_camera_rig_msgs::msg::InstanceSegmentation2DArray &instances,
    double scale_x,
    double scale_y,
    const std_msgs::msg::Header &header,
    sensor_msgs::msg::PointCloud2 &cloud_msg)
{
    if (debug_)
        RCLCPP_INFO(logger_, "[SemanticPointCloud] processSemanticPointCloudInstances called with %zu instances",
                    instances.detections.size());

    if (!have_info_)
    {
        if (debug_)
            RCLCPP_WARN(logger_, "[SemanticPointCloud] No camera info available");
        return false;
    }

    // Convert disparity image
    cv_bridge::CvImageConstPtr disp_cv;
    try
    {
        disp_cv = cv_bridge::toCvShare(std::make_shared<sensor_msgs::msg::Image>(disp_msg), "32FC1");
    }
    catch (const std::exception &e)
    {
        if (debug_)
            RCLCPP_ERROR(logger_, "[SemanticPointCloud] Failed to convert disparity: %s", e.what());
        return false;
    }

    // Convert RGB image
    cv_bridge::CvImageConstPtr image_cv;
    try
    {
        auto img_ptr = std::make_shared<sensor_msgs::msg::Image>(image_msg);
        image_cv = cv_bridge::toCvShare(img_ptr, image_msg.encoding);
    }
    catch (const std::exception &e)
    {
        if (debug_)
            RCLCPP_ERROR(logger_, "[SemanticPointCloud] Failed to convert RGB image: %s", e.what());
        return false;
    }

    const int out_h = disp_cv->image.rows;
    const int out_w = disp_cv->image.cols;

    if (image_cv->image.rows != out_h || image_cv->image.cols != out_w)
    {
        if (debug_)
            RCLCPP_ERROR(logger_, "[SemanticPointCloud] Size mismatch: image %dx%d vs disparity %dx%d",
                         image_cv->image.rows, image_cv->image.cols, out_h, out_w);
        return false;
    }

    // labels/conf initialized to background
    const size_t N = (size_t)out_h * (size_t)out_w;
    std::vector<int32_t> labels(N, config_.background_class_id);
    std::vector<float> confidences(N, (float)config_.background_confidence);

    // Sort by score ascending so higher score overwrites later
    std::vector<multi_camera_rig_msgs::msg::InstanceSegmentation2D> sorted = instances.detections;
    std::sort(sorted.begin(), sorted.end(),
              [](const auto &a, const auto &b) { return a.score < b.score; });

    for (const auto &inst : sorted)
    {
        int32_t class_id = -1;
        try
        {
            class_id = static_cast<int32_t>(std::stoi(inst.class_id));
        }
        catch (...)
        {
            continue;
        }
        const float conf = (float)inst.score;

        // bbox in original resolution
        const float orig_cx = (float)inst.bbox.center.position.x;
        const float orig_cy = (float)inst.bbox.center.position.y;
        const float orig_w  = (float)inst.bbox.size_x;
        const float orig_h  = (float)inst.bbox.size_y;

        // scale bbox into target image
        const float cx = orig_cx * (float)scale_x;
        const float cy = orig_cy * (float)scale_y;
        const float w  = orig_w  * (float)scale_x;
        const float h  = orig_h  * (float)scale_y;

        int x1 = (int)std::floor(cx - 0.5f * w);
        int y1 = (int)std::floor(cy - 0.5f * h);
        int x2 = (int)std::ceil (cx + 0.5f * w);
        int y2 = (int)std::ceil (cy + 0.5f * h);

        x1 = clampi(x1, 0, out_w - 1);
        y1 = clampi(y1, 0, out_h - 1);
        x2 = clampi(x2, 0, out_w);
        y2 = clampi(y2, 0, out_h);

        if (!(x2 > x1 && y2 > y1))
            continue;

        const bool has_mask =
            inst.mask_width > 0 && inst.mask_height > 0 &&
            inst.mask_data.size() == (size_t)inst.mask_width * (size_t)inst.mask_height;

        if (has_mask)
        {
            // Mask is ROI-aligned to bbox (as published by detector)
            // It should match bbox size. If it doesn't, safely map with scaling.
            const int mw = (int)inst.mask_width;
            const int mh = (int)inst.mask_height;
            const uint8_t *m = inst.mask_data.data();

            // Map pixels in bbox to mask indices:
            // u in [x1,x2) -> mx in [0,mw)
            // v in [y1,y2) -> my in [0,mh)
            const float sxm = (mw > 0) ? (float)mw / (float)(x2 - x1) : 0.f;
            const float sym = (mh > 0) ? (float)mh / (float)(y2 - y1) : 0.f;

            for (int v = y1; v < y2; ++v)
            {
                const int my = clampi((int)std::floor((v - y1) * sym), 0, mh - 1);
                const uint8_t *mrow = m + (size_t)my * (size_t)mw;

                for (int u = x1; u < x2; ++u)
                {
                    const int mx = clampi((int)std::floor((u - x1) * sxm), 0, mw - 1);
                    if (mrow[mx] == 0)
                        continue;

                    const size_t idx = (size_t)v * (size_t)out_w + (size_t)u;
                    labels[idx] = class_id;
                    confidences[idx] = conf;
                }
            }
        }
        else
        {
            // Fallback: bbox fill
            for (int v = y1; v < y2; ++v)
            {
                for (int u = x1; u < x2; ++u)
                {
                    const size_t idx = (size_t)v * (size_t)out_w + (size_t)u;
                    labels[idx] = class_id;
                    confidences[idx] = conf;
                }
            }
        }
    }

    // Generate pointcloud (same as existing semantic path)
    const double fx = fx_;
    const double fy = fy_;
    const double cx0 = cx_;
    const double cy0 = cy_;
    const double B = config_.baseline;
    const double maxR = config_.max_range_m;
    const int s = std::max(1, config_.stride);

    // Count valid points
    size_t n_valid = 0;
    for (int v = 0; v < out_h; v += s)
    {
        const float *row = disp_cv->image.ptr<float>(v);
        for (int u = 0; u < out_w; u += s)
        {
            const float d = row[u];
            if (d <= 0.0f)
            {
                if (config_.use_background) n_valid++;
                continue;
            }
            const double Z = fx * B / (double)d;
            if (Z <= 0.0 || Z > maxR)
            {
                if (config_.use_background) n_valid++;
                continue;
            }
            n_valid++;
        }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.width = (uint32_t)n_valid;
    cloud.is_bigendian = false;
    cloud.is_dense = false;

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

    cloud.point_step = 24;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step);

    size_t out_i = 0;
    for (int v = 0; v < out_h; v += s)
    {
        const float *row = disp_cv->image.ptr<float>(v);
        for (int u = 0; u < out_w; u += s)
        {
            const float d = row[u];
            bool use_max_range = false;

            if (d <= 0.0f)
            {
                if (config_.use_background) use_max_range = true;
                else continue;
            }

            double Z = 0.0;
            if (!use_max_range)
            {
                Z = fx * B / (double)d;
                if (Z <= 0.0)
                {
                    if (config_.use_background) use_max_range = true;
                    else continue;
                }
                else if (Z > maxR)
                {
                    if (config_.use_background) Z = maxR;
                    else continue;
                }
            }
            else
            {
                Z = maxR;
            }

            const float X = (float)(((double)u - cx0) * Z / fx);
            const float Y = (float)(((double)v - cy0) * Z / fy);
            const float Zf = (float)Z;

            const size_t pix = (size_t)v * (size_t)out_w + (size_t)u;
            const int32_t class_id = labels[pix];
            const float conf = confidences[pix];

            float rgb_f;
            // RCLCPP_INFO(logger_, "Point: X=%.2f Y=%.2f Z=%.2f class_id=%d conf=%.2f", X, Y, Zf, class_id, conf);
            // RCLCPP_INFO(logger_, "Color by class: %s", config_.color_by_class ? "true" : "false");
            if (config_.color_by_class)
            {
                rgb_f = classIdToRGBFloat(class_id);
            }
            else
            {
                const cv::Vec3b rgb = image_cv->image.at<cv::Vec3b>(v, u);
                rgb_f = packRGBFloat(rgb[0], rgb[1], rgb[2]);
            }

            uint8_t *ptr = cloud.data.data() + out_i * cloud.point_step;
            std::memcpy(ptr + 0,  &X, sizeof(float));
            std::memcpy(ptr + 4,  &Y, sizeof(float));
            std::memcpy(ptr + 8,  &Zf, sizeof(float));
            std::memcpy(ptr + 12, &rgb_f, sizeof(float));
            std::memcpy(ptr + 16, &class_id, sizeof(int32_t));
            std::memcpy(ptr + 20, &conf, sizeof(float));
            out_i++;
        }
    }

    cloud_msg = std::move(cloud);
    return true;
}

} // namespace multi_camera_rig_reconstruction
