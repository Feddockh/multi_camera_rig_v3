/*
Semantic Point Cloud Node (Refactored)
Thin ROS2 wrapper around SemanticPointCloud processor class.

Subscribes to disparity, camera info, and RGB image to create point clouds.
Two modes:
  - normal: Creates RGB pointcloud
  - semantic: Creates semantic pointcloud with class/confidence from detections

Run with:
    ros2 run firefly-ros2-wrapper-reconstruction semantic_pointcloud_node --ros-args \
    -p use_semantics:=false \
    -p baseline:=0.06 \
    -p stride:=1 \
    -p max_range_m:=10.0
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include "firefly_reconstruction/semantic_pointcloud.hpp"
#include "firefly_reconstruction/qos_utils.hpp"

#include <memory>
#include <string>

class SemanticPointCloudNode : public rclcpp::Node
{
public:
    SemanticPointCloudNode() : Node("semantic_pointcloud_node")
    {
        // Configure processor
        firefly_reconstruction::SemanticPointCloudConfig config;
        config.use_semantics = declare_parameter<bool>("use_semantics", false);
        config.baseline = declare_parameter<double>("baseline", 0.06);
        config.stride = declare_parameter<int>("stride", 2);
        config.max_range_m = declare_parameter<double>("max_range_m", 10.0);
        config.use_background = declare_parameter<bool>("use_background", false);
        config.background_class_id = declare_parameter<int>("background_class_id", -1);
        config.background_confidence = declare_parameter<double>("background_confidence", 0.5);
        config.publish_cloud = declare_parameter<bool>("publish_cloud", true);
        config.publish_depth = declare_parameter<bool>("publish_depth", true);

        // Depth filtering options
        config.depth_filter_mode = declare_parameter<std::string>("depth_filter.mode", "none");
        config.depth_flying_ksize = declare_parameter<int>("depth_filter.flying_ksize", 5);
        config.depth_flying_tau = declare_parameter<double>("depth_filter.flying_tau", 0.25);
        config.depth_flying_min_neighbors = declare_parameter<int>("depth_filter.flying_min_neighbors", 6);
        config.depth_median_ksize = declare_parameter<int>("depth_filter.median_ksize", 5);

        // Point cloud filtering options
        config.pc_filter_mode = declare_parameter<std::string>("pc_filter.mode", "none");
        config.pc_grid_ksize = declare_parameter<int>("pc_filter.grid_ksize", 5);
        config.pc_grid_tau = declare_parameter<double>("pc_filter.grid_tau", 0.25);
        config.pc_grid_min_neighbors = declare_parameter<int>("pc_filter.grid_min_neighbors", 4);
        config.pc_knn_k = declare_parameter<int>("pc_filter.knn_k", 20);
        config.pc_knn_stddev_multiplier = declare_parameter<double>("pc_filter.knn_stddev_multiplier", 2.0);

        processor_ = std::make_unique<firefly_reconstruction::SemanticPointCloud>(config);
        config_ = config;

        // Topics
        std::string disparity_topic = declare_parameter<std::string>("disparity_topic", "/stereo/disparity");
        std::string info_topic = declare_parameter<std::string>("camera_info_topic", "/firefly_left/camera_info_rect_scaled");
        std::string image_topic = declare_parameter<std::string>("image_topic", "/firefly_left/image_rect_scaled");
        std::string cloud_topic = declare_parameter<std::string>("cloud_topic", "/stereo/points");
        std::string depth_topic = declare_parameter<std::string>("depth_topic", "/stereo/depth");

        // QoS parameters using shared utility
        std::string sub_rel = declare_parameter<std::string>("sub_qos.reliability", "best_effort");
        std::string sub_dur = declare_parameter<std::string>("sub_qos.durability", "volatile");
        std::string sub_hist = declare_parameter<std::string>("sub_qos.history", "keep_last");
        int sub_depth = declare_parameter<int>("sub_qos.depth", 5);
        
        std::string pub_rel = declare_parameter<std::string>("pub_qos.reliability", "best_effort");
        std::string pub_dur = declare_parameter<std::string>("pub_qos.durability", "volatile");
        std::string pub_hist = declare_parameter<std::string>("pub_qos.history", "keep_last");
        int pub_depth = declare_parameter<int>("pub_qos.depth", 5);

        auto sub_qos = firefly_reconstruction::makeQos(sub_rel, sub_dur, sub_hist, sub_depth);
        auto pub_qos = firefly_reconstruction::makeQos(pub_rel, pub_dur, pub_hist, pub_depth);

        // Subscribers
        info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            info_topic, sub_qos,
            std::bind(&SemanticPointCloudNode::onInfo, this, std::placeholders::_1));

        disp_sub_.subscribe(this, disparity_topic, sub_qos.get_rmw_qos_profile());
        image_sub_.subscribe(this, image_topic, sub_qos.get_rmw_qos_profile());

        // Publishers
        if (config.publish_cloud)
            cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, pub_qos);
        if (config.publish_depth)
            depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_topic, pub_qos);

        // Synchronizer for disparity + image
        using Policy = message_filters::sync_policies::ExactTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
        sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(
            Policy(5), disp_sub_, image_sub_);
        sync_->registerCallback(std::bind(&SemanticPointCloudNode::onSyncCallback, this,
                                          std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "Semantic PointCloud Node initialized");
        RCLCPP_INFO(get_logger(), "  Mode: %s", config.use_semantics ? "SEMANTIC" : "NORMAL");
        RCLCPP_INFO(get_logger(), "  Disparity: %s", disparity_topic.c_str());
        RCLCPP_INFO(get_logger(), "  Image: %s", image_topic.c_str());
        RCLCPP_INFO(get_logger(), "  Camera Info: %s", info_topic.c_str());
    }

private:
    void onInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        processor_->updateCameraInfo(*msg);
    }

    void onSyncCallback(const sensor_msgs::msg::Image::ConstSharedPtr &disp_msg,
                        const sensor_msgs::msg::Image::ConstSharedPtr &image_msg)
    {
        if (!processor_->hasCameraInfo())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for camera info...");
            return;
        }

        const auto t0 = now();

        // Publish depth image if requested
        if (config_.publish_depth && depth_pub_)
        {
            sensor_msgs::msg::Image depth_msg;
            if (processor_->processDepthImage(*disp_msg, disp_msg->header, depth_msg))
            {
                depth_pub_->publish(depth_msg);
            }
            else
            {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to process depth image");
            }
        }

        // Publish point cloud if requested
        if (config_.publish_cloud && cloud_pub_)
        {
            sensor_msgs::msg::PointCloud2 cloud_msg;
            bool success = false;

            if (config_.use_semantics)
            {
                success = processor_->processSemanticPointCloud(*disp_msg, *image_msg, 
                                                                 image_msg->header, cloud_msg);
            }
            else
            {
                success = processor_->processNormalPointCloud(*disp_msg, *image_msg,
                                                               image_msg->header, cloud_msg);
            }

            if (success)
            {
                cloud_pub_->publish(cloud_msg);
            }
            else
            {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to process point cloud");
            }
        }

        const auto t2 = now();
        const double total_ms = (t2 - t0).seconds() * 1e3;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "Processing time: %.1f ms", total_ms);
    }

    std::unique_ptr<firefly_reconstruction::SemanticPointCloud> processor_;
    firefly_reconstruction::SemanticPointCloudConfig config_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> disp_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    
    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>
        sync_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SemanticPointCloudNode>());
    rclcpp::shutdown();
    return 0;
}
