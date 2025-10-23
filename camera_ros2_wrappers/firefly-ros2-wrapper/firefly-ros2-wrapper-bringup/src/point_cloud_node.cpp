#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/stereo_camera_model.h>
#include <chrono>
#include <memory>

class PointCloudNode : public rclcpp::Node
{
public:
    PointCloudNode() : Node("point_cloud_node")
    {
        // Declare parameters
        this->declare_parameter("use_color", true);
        this->declare_parameter("queue_size", 10);
        this->declare_parameter("min_depth", 0.1);  // meters
        this->declare_parameter("max_depth", 10.0);  // meters
        this->declare_parameter("organized", false);  // true = organized point cloud (with NaN for invalid points)
        
        use_color_ = this->get_parameter("use_color").as_bool();
        int queue_size = this->get_parameter("queue_size").as_int();
        min_depth_ = this->get_parameter("min_depth").as_double();
        max_depth_ = this->get_parameter("max_depth").as_double();
        organized_ = this->get_parameter("organized").as_bool();

        // Initialize subscribers using message_filters for synchronization
        disparity_sub_.subscribe(this, "disparity");
        left_info_sub_.subscribe(this, "left/camera_info");
        right_info_sub_.subscribe(this, "right/camera_info");
        
        if (use_color_) {
            left_image_sub_.subscribe(this, "left/image_rect_color");
            
            // 4-way sync with color
            sync_with_color_ = std::make_shared<message_filters::Synchronizer<SyncPolicyWithColor>>(
                SyncPolicyWithColor(queue_size), 
                disparity_sub_, left_info_sub_, right_info_sub_, left_image_sub_);
            sync_with_color_->registerCallback(&PointCloudNode::cloudCallbackWithColor, this);
            
            RCLCPP_INFO(this->get_logger(), "Using 4-way sync with color image");
        } else {
            // 3-way sync without color
            sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
                SyncPolicy(queue_size), 
                disparity_sub_, left_info_sub_, right_info_sub_);
            sync_->registerCallback(&PointCloudNode::cloudCallback, this);
            
            RCLCPP_INFO(this->get_logger(), "Using 3-way sync without color");
        }

        // Create publisher
        points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "points2", 10);

        RCLCPP_INFO(this->get_logger(), 
            "Point Cloud Node initialized (color=%s, organized=%s, depth range: %.2f-%.2f m)",
            use_color_ ? "true" : "false",
            organized_ ? "true" : "false",
            min_depth_, max_depth_);
    }

private:
    void cloudCallback(
        const stereo_msgs::msg::DisparityImage::ConstSharedPtr& disparity_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& left_info,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& right_info)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Received synchronized messages (3-way)");
        processPointCloud(disparity_msg, left_info, right_info, nullptr);
    }

    void cloudCallbackWithColor(
        const stereo_msgs::msg::DisparityImage::ConstSharedPtr& disparity_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& left_info,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& right_info,
        const sensor_msgs::msg::Image::ConstSharedPtr& left_image)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Received synchronized messages (4-way with color)");
        processPointCloud(disparity_msg, left_info, right_info, left_image);
    }

    void processPointCloud(
        const stereo_msgs::msg::DisparityImage::ConstSharedPtr& disparity_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& left_info,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& right_info,
        const sensor_msgs::msg::Image::ConstSharedPtr& left_image)
    {
        try {
            auto start_time = std::chrono::high_resolution_clock::now();

            // Setup stereo camera model
            image_geometry::StereoCameraModel model;
            model.fromCameraInfo(*left_info, *right_info);

            // Convert disparity image to OpenCV Mat
            cv::Mat disparity_float;
            if (disparity_msg->image.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                cv_bridge::CvImagePtr disparity_ptr = 
                    cv_bridge::toCvCopy(disparity_msg->image, sensor_msgs::image_encodings::TYPE_32FC1);
                disparity_float = disparity_ptr->image;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Unsupported disparity encoding: %s", 
                    disparity_msg->image.encoding.c_str());
                return;
            }

            // Get color image if available
            cv::Mat color_image;
            if (left_image) {
                cv_bridge::CvImagePtr color_ptr;
                if (left_image->encoding == sensor_msgs::image_encodings::MONO8) {
                    color_ptr = cv_bridge::toCvCopy(left_image, sensor_msgs::image_encodings::MONO8);
                    // Convert mono to BGR for consistency
                    cv::cvtColor(color_ptr->image, color_image, cv::COLOR_GRAY2BGR);
                } else if (left_image->encoding == sensor_msgs::image_encodings::BGR8 ||
                           left_image->encoding == sensor_msgs::image_encodings::RGB8) {
                    color_ptr = cv_bridge::toCvCopy(left_image, sensor_msgs::image_encodings::BGR8);
                    color_image = color_ptr->image;
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Unsupported color encoding: %s, using white points", 
                        left_image->encoding.c_str());
                }
            }

            // Count valid points
            int height = disparity_float.rows;
            int width = disparity_float.cols;
            size_t valid_points = 0;

            if (!organized_) {
                // Count valid points for dense cloud
                for (int v = 0; v < height; ++v) {
                    for (int u = 0; u < width; ++u) {
                        float d = disparity_float.at<float>(v, u);
                        if (d > 0.0f) {
                            // Compute depth to check validity
                            float Z = (disparity_msg->t * disparity_msg->f) / d;
                            if (Z >= min_depth_ && Z <= max_depth_) {
                                valid_points++;
                            }
                        }
                    }
                }
            } else {
                // Organized cloud - keep all points
                valid_points = height * width;
            }

            // Create PointCloud2 message
            auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            cloud_msg->header = disparity_msg->header;
            cloud_msg->height = organized_ ? height : 1;
            cloud_msg->width = organized_ ? width : valid_points;
            cloud_msg->is_dense = !organized_;  // organized clouds have NaN values
            cloud_msg->is_bigendian = false;

            // Setup fields
            sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
            if (use_color_ && !color_image.empty()) {
                modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
            } else {
                modifier.setPointCloud2FieldsByString(1, "xyz");
            }
            modifier.resize(valid_points);

            // Determine if we have color
            bool has_color = use_color_ && !color_image.empty();

            // Create iterators
            sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
            
            // Only create color iterators if we have color data
            std::unique_ptr<sensor_msgs::PointCloud2Iterator<uint8_t>> iter_r;
            std::unique_ptr<sensor_msgs::PointCloud2Iterator<uint8_t>> iter_g;
            std::unique_ptr<sensor_msgs::PointCloud2Iterator<uint8_t>> iter_b;
            
            if (has_color) {
                iter_r = std::make_unique<sensor_msgs::PointCloud2Iterator<uint8_t>>(*cloud_msg, "r");
                iter_g = std::make_unique<sensor_msgs::PointCloud2Iterator<uint8_t>>(*cloud_msg, "g");
                iter_b = std::make_unique<sensor_msgs::PointCloud2Iterator<uint8_t>>(*cloud_msg, "b");
            }

            // Get camera parameters
            float fx = disparity_msg->f;
            float fy = disparity_msg->f;  // Assuming square pixels
            float cx = left_info->k[2];   // Principal point x
            float cy = left_info->k[5];   // Principal point y
            float baseline = disparity_msg->t;

            // Fill point cloud
            size_t point_count = 0;
            for (int v = 0; v < height; ++v) {
                for (int u = 0; u < width; ++u) {
                    float d = disparity_float.at<float>(v, u);
                    
                    if (organized_) {
                        // For organized clouds, include invalid points as NaN
                        if (d > 0.0f) {
                            // Compute 3D point using pinhole camera model
                            float Z = (baseline * fx) / d;
                            
                            if (Z >= min_depth_ && Z <= max_depth_) {
                                float X = (u - cx) * Z / fx;
                                float Y = (v - cy) * Z / fy;
                                
                                *iter_x = X;
                                *iter_y = Y;
                                *iter_z = Z;
                            } else {
                                *iter_x = std::numeric_limits<float>::quiet_NaN();
                                *iter_y = std::numeric_limits<float>::quiet_NaN();
                                *iter_z = std::numeric_limits<float>::quiet_NaN();
                            }
                        } else {
                            *iter_x = std::numeric_limits<float>::quiet_NaN();
                            *iter_y = std::numeric_limits<float>::quiet_NaN();
                            *iter_z = std::numeric_limits<float>::quiet_NaN();
                        }

                        // Add color
                        if (has_color) {
                            cv::Vec3b color = color_image.at<cv::Vec3b>(v, u);
                            **iter_b = color[0];
                            **iter_g = color[1];
                            **iter_r = color[2];
                        }

                        ++iter_x; ++iter_y; ++iter_z;
                        if (has_color) { ++(*iter_r); ++(*iter_g); ++(*iter_b); }
                        point_count++;
                        
                    } else {
                        // For dense clouds, only include valid points
                        if (d > 0.0f) {
                            float Z = (baseline * fx) / d;
                            
                            if (Z >= min_depth_ && Z <= max_depth_) {
                                float X = (u - cx) * Z / fx;
                                float Y = (v - cy) * Z / fy;
                                
                                *iter_x = X;
                                *iter_y = Y;
                                *iter_z = Z;

                                // Add color
                                if (has_color) {
                                    cv::Vec3b color = color_image.at<cv::Vec3b>(v, u);
                                    **iter_b = color[0];
                                    **iter_g = color[1];
                                    **iter_r = color[2];
                                }

                                ++iter_x; ++iter_y; ++iter_z;
                                if (has_color) { ++(*iter_r); ++(*iter_g); ++(*iter_b); }
                                point_count++;
                            }
                        }
                    }
                }
            }

            // Resize to actual number of points (in case we overestimated)
            if (!organized_ && point_count != valid_points) {
                cloud_msg->width = point_count;
                cloud_msg->data.resize(point_count * cloud_msg->point_step);
            }

            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Generated point cloud with %zu points in %ld ms (%.2f Hz)", 
                point_count, duration.count(), 1000.0 / duration.count());

            // Publish
            points_pub_->publish(*cloud_msg);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
        }
    }

    // Sync policies
    typedef message_filters::sync_policies::ApproximateTime<
        stereo_msgs::msg::DisparityImage,
        sensor_msgs::msg::CameraInfo,
        sensor_msgs::msg::CameraInfo> SyncPolicy;

    typedef message_filters::sync_policies::ApproximateTime<
        stereo_msgs::msg::DisparityImage,
        sensor_msgs::msg::CameraInfo,
        sensor_msgs::msg::CameraInfo,
        sensor_msgs::msg::Image> SyncPolicyWithColor;

    // Subscribers
    message_filters::Subscriber<stereo_msgs::msg::DisparityImage> disparity_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> left_info_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> right_info_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> left_image_sub_;

    // Synchronizers
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicyWithColor>> sync_with_color_;

    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;

    // Parameters
    bool use_color_;
    bool organized_;
    double min_depth_;
    double max_depth_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
