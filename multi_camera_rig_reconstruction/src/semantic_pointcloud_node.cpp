#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include "multi_camera_rig_reconstruction/semantic_pointcloud.hpp"
#include "multi_camera_rig_common/qos_utils.hpp"
#include "multi_camera_rig_msgs/msg/instance_segmentation2_d_array.hpp"

#include <memory>
#include <string>

class SemanticPointCloudNode : public rclcpp::Node
{
public:
    SemanticPointCloudNode() : Node("semantic_pointcloud_node")
    {
        // ---- Params: algorithm/config ----
        multi_camera_rig_reconstruction::SemanticPointCloudConfig config;
        config.use_semantics = declare_parameter<bool>("use_semantics", false);
        config.baseline = declare_parameter<double>("baseline", 0.06);
        config.stride = declare_parameter<int>("stride", 2);
        config.max_range_m = declare_parameter<double>("max_range_m", 10.0);
        config.use_background = declare_parameter<bool>("use_background", false);
        config.background_class_id = declare_parameter<int>("background_class_id", -1);
        config.background_confidence = declare_parameter<double>("background_confidence", 0.5);
        config.color_by_class = declare_parameter<bool>("color_by_class", false);
        config.publish_cloud = declare_parameter<bool>("publish_cloud", true);
        config.publish_depth = declare_parameter<bool>("publish_depth", false);

        debug_ = declare_parameter<bool>("debug", false);

        // ---- Params: topics ----
        disparity_topic_ = declare_parameter<std::string>("disparity_topic", "/stereo/disparity");
        info_topic_ = declare_parameter<std::string>("camera_info_topic", "/firefly_left/camera_info_rect_scaled");
        image_topic_ = declare_parameter<std::string>("image_topic", "/firefly_left/image_rect_scaled");
        cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/stereo/points");
        depth_topic_ = declare_parameter<std::string>("depth_topic", "/stereo/depth");

        detection_topic_ = declare_parameter<std::string>("detection_topic", "/yolo/detections");
        seg_detection_topic_ = declare_parameter<std::string>("seg_detection_topic", "/yolo/instance_segmentation");

        // ---- Params: semantic input selection ----
        use_seg_detection_ = declare_parameter<bool>("use_seg_detection", false);

        // ---- QoS ----
        const std::string sub_rel = declare_parameter<std::string>("sub_qos.reliability", "best_effort");
        const std::string sub_dur = declare_parameter<std::string>("sub_qos.durability", "volatile");
        const std::string sub_hist = declare_parameter<std::string>("sub_qos.history", "keep_last");
        const int sub_depth = declare_parameter<int>("sub_qos.depth", 5);

        const std::string pub_rel = declare_parameter<std::string>("pub_qos.reliability", "best_effort");
        const std::string pub_dur = declare_parameter<std::string>("pub_qos.durability", "volatile");
        const std::string pub_hist = declare_parameter<std::string>("pub_qos.history", "keep_last");
        const int pub_depth = declare_parameter<int>("pub_qos.depth", 5);

        sub_qos_ = multi_camera_rig_common::makeQos(sub_rel, sub_dur, sub_hist, sub_depth);
        pub_qos_ = multi_camera_rig_common::makeQos(pub_rel, pub_dur, pub_hist, pub_depth);

        // ---- Processor ----
        processor_ = std::make_unique<multi_camera_rig_reconstruction::SemanticPointCloud>(config, debug_, get_logger());
        config_ = config;

        // ---- ROS pub/sub ----
        info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            info_topic_, sub_qos_,
            std::bind(&SemanticPointCloudNode::onInfo, this, std::placeholders::_1));

        if (config_.publish_cloud)
            cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, pub_qos_);
        if (config_.publish_depth)
            depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_topic_, pub_qos_);

        // ---- Synchronizers ----
        setupSync();

        // ---- Log summary ----
        RCLCPP_INFO(get_logger(), "Semantic PointCloud Node initialized");
        RCLCPP_INFO(get_logger(), "  Mode: %s", config_.use_semantics ? "SEMANTIC" : "NORMAL");
        if (config_.use_semantics)
            RCLCPP_INFO(get_logger(), "  Semantic source: %s",
                        use_seg_detection_ ? "InstanceSegmentation2DArray" : "Detection2DArray");
        RCLCPP_INFO(get_logger(), "  Disparity: %s", disparity_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  Image: %s", image_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  Camera Info: %s", info_topic_.c_str());
        if (config_.use_semantics)
        {
            RCLCPP_INFO(get_logger(), "  Detections: %s", detection_topic_.c_str());
            RCLCPP_INFO(get_logger(), "  Instances: %s", seg_detection_topic_.c_str());
        }
    }

private:
    // ------------------------
    // Setup / wiring
    // ------------------------
    void setupSync()
    {
        disp_sub_.subscribe(this, disparity_topic_, sub_qos_.get_rmw_qos_profile());
        image_sub_.subscribe(this, image_topic_, sub_qos_.get_rmw_qos_profile());

        if (!config_.use_semantics)
        {
            using NormalPolicy =
                message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
            sync_normal_ = std::make_shared<message_filters::Synchronizer<NormalPolicy>>(
                NormalPolicy(5), disp_sub_, image_sub_);
            sync_normal_->registerCallback(std::bind(&SemanticPointCloudNode::onNormal, this,
                                                     std::placeholders::_1, std::placeholders::_2));
            return;
        }

        if (use_seg_detection_)
        {
            seg_detection_sub_.subscribe(this, seg_detection_topic_, sub_qos_.get_rmw_qos_profile());

            using SegPolicy = message_filters::sync_policies::ExactTime<
                sensor_msgs::msg::Image, sensor_msgs::msg::Image,
                multi_camera_rig_msgs::msg::InstanceSegmentation2DArray>;

            sync_semantic_seg_ = std::make_shared<message_filters::Synchronizer<SegPolicy>>(
                SegPolicy(5), disp_sub_, image_sub_, seg_detection_sub_);

            // ✅ FIX: bind to instance callback
            sync_semantic_seg_->registerCallback(std::bind(&SemanticPointCloudNode::onSemanticInstances, this,
                                                           std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        }
        else
        {
            detection_sub_.subscribe(this, detection_topic_, sub_qos_.get_rmw_qos_profile());

            using DetPolicy = message_filters::sync_policies::ExactTime<
                sensor_msgs::msg::Image, sensor_msgs::msg::Image, vision_msgs::msg::Detection2DArray>;

            sync_semantic_ = std::make_shared<message_filters::Synchronizer<DetPolicy>>(
                DetPolicy(5), disp_sub_, image_sub_, detection_sub_);

            sync_semantic_->registerCallback(std::bind(&SemanticPointCloudNode::onSemanticDetections, this,
                                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        }
    }

    // ------------------------
    // Shared helpers
    // ------------------------
    void onInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        processor_->updateCameraInfo(*msg);
    }

    bool readyOrWarn()
    {
        if (processor_->hasCameraInfo())
            return true;

        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for camera info...");
        return false;
    }

    void maybePublishDepth(const sensor_msgs::msg::Image &disp, const std_msgs::msg::Header &header)
    {
        if (!config_.publish_depth || !depth_pub_)
            return;

        sensor_msgs::msg::Image depth_msg;
        if (processor_->processDepthImage(disp, header, depth_msg))
        {
            depth_pub_->publish(depth_msg);
        }
        else
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to process depth image");
        }
    }

    template <typename GenerateCloudFn>
    void maybePublishCloud(GenerateCloudFn &&fn)
    {
        if (!config_.publish_cloud || !cloud_pub_)
            return;

        sensor_msgs::msg::PointCloud2 cloud_msg;
        const bool success = fn(cloud_msg);

        if (success)
            cloud_pub_->publish(cloud_msg);
        else
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to process point cloud");
    }

    // ------------------------
    // Callbacks
    // ------------------------
    void onNormal(const sensor_msgs::msg::Image::ConstSharedPtr &disp_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr &image_msg)
    {
        if (debug_)
        {
            RCLCPP_INFO(get_logger(), "onNormal: disp=%dx%d image=%dx%d",
                        disp_msg->width, disp_msg->height, image_msg->width, image_msg->height);
        }

        if (!readyOrWarn())
            return;

        const auto t0 = now();

        maybePublishDepth(*disp_msg, disp_msg->header);

        maybePublishCloud([&](sensor_msgs::msg::PointCloud2 &out) {
            return processor_->processNormalPointCloud(*disp_msg, *image_msg, disp_msg->header, out);
        });

        const auto t1 = now();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "Processing time (normal): %.1f ms", (t1 - t0).seconds() * 1e3);
    }

    void onSemanticDetections(const sensor_msgs::msg::Image::ConstSharedPtr &disp_msg,
                              const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
                              const vision_msgs::msg::Detection2DArray::ConstSharedPtr &det_msg)
    {
        if (debug_)
        {
            RCLCPP_INFO(get_logger(), "onSemanticDetections: disp=%dx%d image=%dx%d dets=%zu",
                        disp_msg->width, disp_msg->height,
                        image_msg->width, image_msg->height,
                        det_msg->detections.size());
        }

        if (!readyOrWarn())
            return;

        const auto t0 = now();

        maybePublishCloud([&](sensor_msgs::msg::PointCloud2 &out) {
            // Detections already scaled to image -> scale factors 1.0
            return processor_->processSemanticPointCloud(*disp_msg, *image_msg, det_msg->detections,
                                                         1.0, 1.0, disp_msg->header, out);
        });

        maybePublishDepth(*disp_msg, disp_msg->header);

        const auto t1 = now();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "Processing time (semantic det): %.1f ms", (t1 - t0).seconds() * 1e3);
    }

    void onSemanticInstances(const sensor_msgs::msg::Image::ConstSharedPtr &disp_msg,
                             const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
                             const multi_camera_rig_msgs::msg::InstanceSegmentation2DArray::ConstSharedPtr &inst_msg)
    {
        if (debug_)
        {
            RCLCPP_INFO(get_logger(), "onSemanticInstances: disp=%dx%d image=%dx%d inst=%zu",
                        disp_msg->width, disp_msg->height,
                        image_msg->width, image_msg->height,
                        inst_msg->detections.size());
        }

        if (!readyOrWarn())
            return;

        const auto t0 = now();

        maybePublishCloud([&](sensor_msgs::msg::PointCloud2 &out) {
            return processor_->processSemanticPointCloudInstances(*disp_msg, *image_msg, *inst_msg,
                                                                  1.0, 1.0, disp_msg->header, out);
        });

        maybePublishDepth(*disp_msg, disp_msg->header);

        const auto t1 = now();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "Processing time (semantic inst): %.1f ms", (t1 - t0).seconds() * 1e3);
    }

private:
    // ---- Processor ----
    std::unique_ptr<multi_camera_rig_reconstruction::SemanticPointCloud> processor_;
    multi_camera_rig_reconstruction::SemanticPointCloudConfig config_;
    bool debug_{false};

    // ---- Topics / wiring ----
    bool use_seg_detection_{false};

    std::string disparity_topic_;
    std::string info_topic_;
    std::string image_topic_;
    std::string cloud_topic_;
    std::string depth_topic_;
    std::string detection_topic_;
    std::string seg_detection_topic_;

    // ---- QoS ----
    rclcpp::QoS sub_qos_{rclcpp::SystemDefaultsQoS()};
    rclcpp::QoS pub_qos_{rclcpp::SystemDefaultsQoS()};

    // ---- ROS ----
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;

    message_filters::Subscriber<sensor_msgs::msg::Image> disp_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray> detection_sub_;
    message_filters::Subscriber<multi_camera_rig_msgs::msg::InstanceSegmentation2DArray> seg_detection_sub_;

    // Synchronizers
    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync_normal_;

    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ExactTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image, vision_msgs::msg::Detection2DArray>>> sync_semantic_;

    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ExactTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image,
            multi_camera_rig_msgs::msg::InstanceSegmentation2DArray>>> sync_semantic_seg_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SemanticPointCloudNode>());
    rclcpp::shutdown();
    return 0;
}
