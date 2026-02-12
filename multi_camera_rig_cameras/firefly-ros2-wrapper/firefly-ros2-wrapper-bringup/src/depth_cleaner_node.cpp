/**
 * @file depth_cleaner_node.cpp
 * @brief Assigns far-away depth values to invalid pixels for proper free space mapping
 * 
 * Subscribes to raw depth images and republishes cleaned versions where:
 * - Valid depth values (min_depth < depth <= max_depth) are preserved
 * - Invalid values (NaN, Inf, 0, < min_depth, > max_depth) are handled:
 *   - Sampled invalid pixels (stride pattern) → set to invalid_depth_value for free space marking
 *   - Non-sampled invalid pixels → set to NaN (ignored by pointcloud generation)
 * 
 * This ensures pointcloud generation includes distant points for areas with no returns,
 * allowing the octomap to properly mark free space throughout the camera's field of view,
 * while minimizing computational overhead from redundant rays and preventing sensor origin artifacts.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class DepthCleanerNode : public rclcpp::Node
{
public:
    DepthCleanerNode() : Node("depth_cleaner_node")
    {
        // Declare parameters
        this->declare_parameter("input_depth_topic", "/firefly_left/depth/image");
        this->declare_parameter("output_depth_topic", "/firefly_left/depth/image_cleaned");
        this->declare_parameter("min_depth", 0.1);
        this->declare_parameter("max_depth", 5.0);
        this->declare_parameter("invalid_depth_value", 5.0);
        this->declare_parameter("queue_size", 10);
        this->declare_parameter("invalid_pixel_stride", 1);  // New parameter!
        this->declare_parameter("preserve_valid_pixels", true);

        // Get parameters
        std::string input_topic = this->get_parameter("input_depth_topic").as_string();
        std::string output_topic = this->get_parameter("output_depth_topic").as_string();
        min_depth_ = this->get_parameter("min_depth").as_double();
        max_depth_ = this->get_parameter("max_depth").as_double();
        invalid_depth_value_ = this->get_parameter("invalid_depth_value").as_double();
        int queue_size = this->get_parameter("queue_size").as_int();
        invalid_pixel_stride_ = this->get_parameter("invalid_pixel_stride").as_int();
        preserve_valid_pixels_ = this->get_parameter("preserve_valid_pixels").as_bool();

        // Validate parameters
        if (invalid_pixel_stride_ < 1)
        {
            RCLCPP_WARN(this->get_logger(), 
                       "invalid_pixel_stride must be >= 1, setting to 1");
            invalid_pixel_stride_ = 1;
        }

        // Create subscriber and publisher
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic, queue_size,
            std::bind(&DepthCleanerNode::depthCallback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic, queue_size);

        RCLCPP_INFO(this->get_logger(), "Depth cleaner node started");
        RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Min valid depth: %.2f m", min_depth_);
        RCLCPP_INFO(this->get_logger(), "  Max valid depth: %.2f m", max_depth_);
        RCLCPP_INFO(this->get_logger(), "  Invalid depth replacement: %.2f m", invalid_depth_value_);
        RCLCPP_INFO(this->get_logger(), "  Invalid pixel stride: %d (1=all, 2=every 2nd, 4=every 4th, etc.)", 
                   invalid_pixel_stride_);
        RCLCPP_INFO(this->get_logger(), "  Preserve valid pixels: %s", 
                   preserve_valid_pixels_ ? "true" : "false");
        
        if (invalid_pixel_stride_ > 1)
        {
            double reduction = 100.0 * (1.0 - 1.0 / (invalid_pixel_stride_ * invalid_pixel_stride_));
            RCLCPP_INFO(this->get_logger(), "  → Reducing invalid pixels by ~%.1f%%", reduction);
        }
    }

private:
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS image to OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat& depth = cv_ptr->image;
        
        // Create masks using vectorized operations (much faster than loops)
        // Mask for NaN/Inf values
        cv::Mat finite_mask;
        cv::patchNaNs(depth, 0.0f);  // Replace NaN with 0 first for comparison operations
        
        // Mask for valid depth: min_depth < depth <= max_depth
        // This filters out sensor origin artifacts (depth=0) and too-close measurements
        cv::Mat valid_mask = (depth > static_cast<float>(min_depth_)) & (depth <= static_cast<float>(max_depth_));
        cv::Mat invalid_mask = ~valid_mask;
        
        // Count valid/invalid pixels efficiently
        int valid_count = cv::countNonZero(valid_mask);
        int invalid_count = depth.rows * depth.cols - valid_count;
        
        // Create stride sampling mask (grid pattern)
        cv::Mat stride_mask = cv::Mat::zeros(depth.size(), CV_8U);
        if (invalid_pixel_stride_ == 1)
        {
            // No downsampling - keep all invalid pixels
            stride_mask = invalid_mask.clone();
        }
        else
        {
            // Create grid pattern mask efficiently using parallel processing
            cv::parallel_for_(cv::Range(0, depth.rows), [&](const cv::Range& range) {
                for (int y = range.start; y < range.end; ++y)
                {
                    if (y % invalid_pixel_stride_ == 0)
                    {
                        uint8_t* stride_row = stride_mask.ptr<uint8_t>(y);
                        const uint8_t* invalid_row = invalid_mask.ptr<uint8_t>(y);
                        
                        for (int x = 0; x < depth.cols; ++x)
                        {
                            if (x % invalid_pixel_stride_ == 0 && invalid_row[x])
                            {
                                stride_row[x] = 255;
                            }
                        }
                    }
                }
            });
        }
        
        int invalid_kept = cv::countNonZero(stride_mask);
        int invalid_zeroed = invalid_count - invalid_kept;
        
        // Apply transformations using vectorized operations
        // 1. Set sampled invalid pixels to invalid_depth_value
        depth.setTo(static_cast<float>(invalid_depth_value_), stride_mask);
        
        // 2. Set non-sampled invalid pixels to NaN (so they're properly ignored, not treated as depth=0 at camera origin)
        cv::Mat non_sampled_invalid = invalid_mask & ~stride_mask;
        depth.setTo(std::numeric_limits<float>::quiet_NaN(), non_sampled_invalid);
        
        // 3. Optionally downsample valid pixels (rarely used)
        if (!preserve_valid_pixels_ && invalid_pixel_stride_ > 1)
        {
            cv::Mat valid_stride_mask = cv::Mat::zeros(depth.size(), CV_8U);
            cv::parallel_for_(cv::Range(0, depth.rows), [&](const cv::Range& range) {
                for (int y = range.start; y < range.end; ++y)
                {
                    if (y % invalid_pixel_stride_ == 0)
                    {
                        uint8_t* stride_row = valid_stride_mask.ptr<uint8_t>(y);
                        const uint8_t* valid_row = valid_mask.ptr<uint8_t>(y);
                        
                        for (int x = 0; x < depth.cols; ++x)
                        {
                            if (x % invalid_pixel_stride_ == 0 && valid_row[x])
                            {
                                stride_row[x] = 255;
                            }
                        }
                    }
                }
            });
            
            cv::Mat non_sampled_valid = valid_mask & ~valid_stride_mask;
            depth.setTo(std::numeric_limits<float>::quiet_NaN(), non_sampled_valid);
        }
        
        // Log statistics periodically (every 100 frames)
        static int frame_count = 0;
        if (++frame_count % 100 == 0)
        {
            int total_pixels = depth.rows * depth.cols;
            double invalid_percent = 100.0 * invalid_count / total_pixels;
            double kept_percent = invalid_count > 0 ? 100.0 * invalid_kept / invalid_count : 0.0;
            
            RCLCPP_INFO(this->get_logger(), 
                        "Frame %d: %dx%d pixels | Valid: %d (%.1f%%) | Invalid: %d (%.1f%%) | Kept: %d (%.1f%% of invalid) | Zeroed: %d",
                        frame_count, depth.cols, depth.rows,
                        valid_count, 100.0 * valid_count / total_pixels,
                        invalid_count, invalid_percent,
                        invalid_kept, kept_percent,
                        invalid_zeroed);
        }

        // Publish cleaned depth image
        pub_->publish(*cv_ptr->toImageMsg());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    double min_depth_;
    double max_depth_;
    double invalid_depth_value_;
    int invalid_pixel_stride_;
    bool preserve_valid_pixels_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthCleanerNode>());
    rclcpp::shutdown();
    return 0;
}
