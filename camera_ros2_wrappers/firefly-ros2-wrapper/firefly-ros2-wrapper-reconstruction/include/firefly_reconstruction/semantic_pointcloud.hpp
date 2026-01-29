#pragma once

#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <memory>
#include <string>
#include <vector>

namespace firefly_reconstruction
{

// Custom point type for semantic pointcloud
struct PointXYZRGBSemanticConfidence
{
    float x;
    float y;
    float z;
    float rgb;        // packed RGB (PCL/RViz compatible)
    int32_t class_id; // semantic class
    float confidence; // detection confidence

    PointXYZRGBSemanticConfidence()
        : x(0), y(0), z(0), rgb(0), class_id(-1), confidence(0.0f) {}
} __attribute__((packed));

struct SemanticPointCloudConfig
{
    // Mode selection
    bool use_semantics = false;

    // Stereo parameters
    double baseline = 0.06;

    // Point cloud generation parameters
    int stride = 2;
    double max_range_m = 10.0;
    bool use_background = false;

    // Output control
    bool publish_cloud = true;
    bool publish_depth = true;

    // Semantic parameters
    int background_class_id = -1;
    double background_confidence = 0.5;

    // Depth filtering options
    std::string depth_filter_mode = "none";
    int depth_flying_ksize = 5;
    double depth_flying_tau = 0.25;
    int depth_flying_min_neighbors = 6;
    int depth_median_ksize = 5;

    // Point cloud filtering options
    std::string pc_filter_mode = "none";
    int pc_grid_ksize = 5;
    double pc_grid_tau = 0.25;
    int pc_grid_min_neighbors = 4;
    int pc_knn_k = 20;
    double pc_knn_stddev_multiplier = 2.0;
};

class SemanticPointCloud
{
public:
    explicit SemanticPointCloud(const SemanticPointCloudConfig &config);
    ~SemanticPointCloud() = default;

    // Update camera intrinsics
    void updateCameraInfo(const sensor_msgs::msg::CameraInfo &info);

    // Check if camera info has been received
    bool hasCameraInfo() const { return have_info_; }

    // Process disparity and image to generate depth image
    // Returns true if successful
    bool processDepthImage(
        const sensor_msgs::msg::Image &disp_msg,
        const std_msgs::msg::Header &header,
        sensor_msgs::msg::Image &depth_msg);

    // Process disparity and image to generate point cloud (normal mode)
    // Returns true if successful
    bool processNormalPointCloud(
        const sensor_msgs::msg::Image &disp_msg,
        const sensor_msgs::msg::Image &image_msg,
        const std_msgs::msg::Header &header,
        sensor_msgs::msg::PointCloud2 &cloud_msg);

    // Process disparity and image to generate semantic point cloud
    // Returns true if successful
    bool processSemanticPointCloud(
        const sensor_msgs::msg::Image &disp_msg,
        const sensor_msgs::msg::Image &image_msg,
        const std_msgs::msg::Header &header,
        sensor_msgs::msg::PointCloud2 &cloud_msg);

private:
    // Apply depth filtering to depth image
    void applyDepthFiltering(std::vector<float> &depth, int out_h, int out_w);

    // Apply point cloud filtering and generate keep mask
    void applyPointCloudFiltering(
        const cv::Mat &disp_img,
        double fx, double fy, double cx, double cy,
        double B, double maxR, int s,
        int out_h, int out_w,
        std::vector<uint8_t> &keep_mask);

    // Helper to pack RGB into a float (PCL convention)
    static float packRGBFloat(uint8_t r, uint8_t g, uint8_t b);

    SemanticPointCloudConfig config_;
    
    // Camera intrinsics
    bool have_info_{false};
    double fx_{0}, fy_{0}, cx_{0}, cy_{0};
    uint32_t img_w_{0}, img_h_{0};
};

} // namespace firefly_reconstruction
