#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <chrono>
#include <chrono>
#include <memory>

class StereoDisparityNode : public rclcpp::Node
{
public:
    StereoDisparityNode() : Node("stereo_disparity_node")
    {
        // Declare parameters with descriptors for rqt_reconfigure
        rcl_interfaces::msg::ParameterDescriptor desc;
        rcl_interfaces::msg::IntegerRange int_range;
        rcl_interfaces::msg::FloatingPointRange float_range;
        
        // stereo_algorithm: 0=StereoBM, 1=StereoSGBM
        desc = rcl_interfaces::msg::ParameterDescriptor();
        desc.description = "Stereo algorithm: 0=StereoBM, 1=StereoSGBM";
        int_range.from_value = 0;
        int_range.to_value = 1;
        int_range.step = 1;
        desc.integer_range = {int_range};
        this->declare_parameter("stereo_algorithm", 1, desc);
        
        // min_disparity
        desc = rcl_interfaces::msg::ParameterDescriptor();
        desc.description = "Minimum disparity value";
        int_range.from_value = -128;
        int_range.to_value = 128;
        int_range.step = 1;
        desc.integer_range = {int_range};
        this->declare_parameter("min_disparity", 0, desc);
        
        // num_disparities (must be multiple of 16)
        desc = rcl_interfaces::msg::ParameterDescriptor();
        desc.description = "Number of disparities (multiple of 16)";
        int_range.from_value = 16;
        int_range.to_value = 256;
        int_range.step = 16;
        desc.integer_range = {int_range};
        this->declare_parameter("num_disparities", 96, desc);
        
        // block_size (must be odd)
        desc = rcl_interfaces::msg::ParameterDescriptor();
        desc.description = "Block size (odd number 3-21)";
        int_range.from_value = 3;
        int_range.to_value = 21;
        int_range.step = 2;
        desc.integer_range = {int_range};
        this->declare_parameter("block_size", 11, desc);
        
        // P1
        desc = rcl_interfaces::msg::ParameterDescriptor();
        desc.description = "P1 smoothness penalty (0=auto)";
        int_range.from_value = 0;
        int_range.to_value = 5000;
        int_range.step = 10;
        desc.integer_range = {int_range};
        this->declare_parameter("P1", 0, desc);
        
        // P2
        desc = rcl_interfaces::msg::ParameterDescriptor();
        desc.description = "P2 smoothness penalty (0=auto)";
        int_range.from_value = 0;
        int_range.to_value = 10000;
        int_range.step = 50;
        desc.integer_range = {int_range};
        this->declare_parameter("P2", 0, desc);
        
        // disp12_max_diff
        desc = rcl_interfaces::msg::ParameterDescriptor();
        desc.description = "Max disparity diff in left-right check (-1=disable)";
        int_range.from_value = -1;
        int_range.to_value = 10;
        int_range.step = 1;
        desc.integer_range = {int_range};
        this->declare_parameter("disp12_max_diff", 1, desc);
        
        // pre_filter_cap
        desc = rcl_interfaces::msg::ParameterDescriptor();
        desc.description = "Prefilter cap";
        int_range.from_value = 1;
        int_range.to_value = 63;
        int_range.step = 1;
        desc.integer_range = {int_range};
        this->declare_parameter("pre_filter_cap", 31, desc);
        
        // uniqueness_ratio
        desc = rcl_interfaces::msg::ParameterDescriptor();
        desc.description = "Uniqueness ratio (5-15 typical)";
        int_range.from_value = 0;
        int_range.to_value = 100;
        int_range.step = 1;
        desc.integer_range = {int_range};
        this->declare_parameter("uniqueness_ratio", 10, desc);
        
        // speckle_window_size
        desc = rcl_interfaces::msg::ParameterDescriptor();
        desc.description = "Speckle window size (0=disable)";
        int_range.from_value = 0;
        int_range.to_value = 300;
        int_range.step = 10;
        desc.integer_range = {int_range};
        this->declare_parameter("speckle_window_size", 200, desc);
        
        // speckle_range
        desc = rcl_interfaces::msg::ParameterDescriptor();
        desc.description = "Speckle range (0=disable)";
        int_range.from_value = 0;
        int_range.to_value = 10;
        int_range.step = 1;
        desc.integer_range = {int_range};
        this->declare_parameter("speckle_range", 2, desc);
        
        // mode (SGBM mode)
        desc = rcl_interfaces::msg::ParameterDescriptor();
        desc.description = "SGBM mode: 0=SGBM, 1=HH, 2=3WAY";
        int_range.from_value = 0;
        int_range.to_value = 2;
        int_range.step = 1;
        desc.integer_range = {int_range};
        this->declare_parameter("mode", 2, desc);
        
        // focal_length
        desc = rcl_interfaces::msg::ParameterDescriptor();
        desc.description = "Focal length in pixels";
        float_range.from_value = 100.0;
        float_range.to_value = 2000.0;
        float_range.step = 1.0;
        desc.floating_point_range = {float_range};
        this->declare_parameter("focal_length", 858.0, desc);
        
        // baseline
        desc = rcl_interfaces::msg::ParameterDescriptor();
        desc.description = "Baseline in meters";
        float_range.from_value = 0.01;
        float_range.to_value = 1.0;
        float_range.step = 0.01;
        desc.floating_point_range = {float_range};
        this->declare_parameter("baseline", 0.06, desc);


        // Initialize subscribers using message_filters for synchronization
        left_image_sub_.subscribe(this, "/firefly_left/image_rect_mono");
        right_image_sub_.subscribe(this, "/firefly_right/image_rect_mono");
        left_info_sub_.subscribe(this, "/firefly_left/camera_info");
        right_info_sub_.subscribe(this, "/firefly_right/camera_info");

        // Create synchronizer for approximate time sync
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), left_image_sub_, right_image_sub_, left_info_sub_, right_info_sub_);
        
        sync_->registerCallback(&StereoDisparityNode::stereoCallback, this);

        // Create publisher
        disparity_pub_ = this->create_publisher<stereo_msgs::msg::DisparityImage>(
            "/firefly/disparity_custom", 10);

        // Initialize stereo matcher
        initializeStereoMatcher();
        
        // Apply params once to ensure consistency
        applyParams();

        // Live parameter updates
        param_cb_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params)
            {
                bool reinit = false;
                for (auto &p : params)
                {
                    if (p.get_name() == "stereo_algorithm" || p.get_name() == "mode")
                        reinit = true;
                }
                if (reinit) {
                    initializeStereoMatcher();
                }
                // Always re-apply to push values to OpenCV object
                applyParams();

                rcl_interfaces::msg::SetParametersResult res;
                res.successful = true;
                res.reason = "updated";
                return res;
            });

        RCLCPP_INFO(this->get_logger(), "Stereo Disparity Node initialized");
    }

private:
    void applyParams()
    {
        // Enforce valid ranges and apply to OpenCV matcher
        int min_disp = this->get_parameter("min_disparity").as_int();
        int num_disp = this->get_parameter("num_disparities").as_int();
        int block_size = this->get_parameter("block_size").as_int();
        int uniqueness = this->get_parameter("uniqueness_ratio").as_int();
        int speckle_win = this->get_parameter("speckle_window_size").as_int();
        int speckle_rng = this->get_parameter("speckle_range").as_int();
        int pre_cap = this->get_parameter("pre_filter_cap").as_int();
        int disp12diff = this->get_parameter("disp12_max_diff").as_int();
        int P1v = this->get_parameter("P1").as_int();
        int P2v = this->get_parameter("P2").as_int();
        int mode = this->get_parameter("mode").as_int();
        int algo = this->get_parameter("stereo_algorithm").as_int();

        // Constraints
        if (block_size % 2 == 0) block_size += 1;
        block_size = std::max(3, std::min(21, block_size));
        if (num_disp % 16 != 0) num_disp = ((num_disp + 15) / 16) * 16; // round to multiple of 16
        num_disp = std::max(16, num_disp);

        if (algo == 0 && stereo_bm_)
        {
            stereo_bm_->setMinDisparity(min_disp);
            stereo_bm_->setNumDisparities(num_disp);
            stereo_bm_->setBlockSize(block_size);
            stereo_bm_->setPreFilterCap(pre_cap);
            stereo_bm_->setUniquenessRatio(uniqueness);
            stereo_bm_->setSpeckleWindowSize(speckle_win);
            stereo_bm_->setSpeckleRange(speckle_rng);
            stereo_bm_->setDisp12MaxDiff(disp12diff);
        }
        else if (algo == 1 && stereo_sgbm_)
        {
            stereo_sgbm_->setMinDisparity(min_disp);
            stereo_sgbm_->setNumDisparities(num_disp);
            stereo_sgbm_->setBlockSize(block_size);
            stereo_sgbm_->setPreFilterCap(pre_cap);
            stereo_sgbm_->setUniquenessRatio(uniqueness);
            stereo_sgbm_->setSpeckleWindowSize(speckle_win);
            stereo_sgbm_->setSpeckleRange(speckle_rng);
            stereo_sgbm_->setDisp12MaxDiff(disp12diff);
            stereo_sgbm_->setMode(mode);

            // Auto-compute P1/P2 if too low (reasonable defaults)
            if (P1v <= 0 || P2v <= 0) {
                int cn = 1; // MONO8
                P1v = 8 * cn * block_size * block_size;
                P2v = 32 * cn * block_size * block_size;
            }
            stereo_sgbm_->setP1(P1v);
            stereo_sgbm_->setP2(P2v);
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Applied params: algo=%d, min_disp=%d, num_disp=%d, block=%d, uniq=%d, speckle(%d,%d), P1=%d, P2=%d, mode=%d",
            algo, min_disp, num_disp, block_size, uniqueness, speckle_win, speckle_rng, P1v, P2v, mode);
    }

    void initializeStereoMatcher()
    {
        try {
            int algorithm = this->get_parameter("stereo_algorithm").as_int();
            
            if (algorithm == 0) {
                // StereoBM
                stereo_bm_ = cv::StereoBM::create(
                    this->get_parameter("num_disparities").as_int(),
                    this->get_parameter("block_size").as_int()
                );
                
                if (stereo_bm_) {
                    stereo_bm_->setMinDisparity(this->get_parameter("min_disparity").as_int());
                    stereo_bm_->setPreFilterCap(this->get_parameter("pre_filter_cap").as_int());
                    stereo_bm_->setUniquenessRatio(this->get_parameter("uniqueness_ratio").as_int());
                    stereo_bm_->setSpeckleWindowSize(this->get_parameter("speckle_window_size").as_int());
                    stereo_bm_->setSpeckleRange(this->get_parameter("speckle_range").as_int());
                    stereo_bm_->setDisp12MaxDiff(this->get_parameter("disp12_max_diff").as_int());
                    RCLCPP_INFO(this->get_logger(), "StereoBM initialized successfully");
                }
            } else {
                // StereoSGBM  
                stereo_sgbm_ = cv::StereoSGBM::create(
                    this->get_parameter("min_disparity").as_int(),
                    this->get_parameter("num_disparities").as_int(),
                    this->get_parameter("block_size").as_int()
                );
                
                if (stereo_sgbm_) {
                    stereo_sgbm_->setP1(this->get_parameter("P1").as_int());
                    stereo_sgbm_->setP2(this->get_parameter("P2").as_int());
                    stereo_sgbm_->setDisp12MaxDiff(this->get_parameter("disp12_max_diff").as_int());
                    stereo_sgbm_->setPreFilterCap(this->get_parameter("pre_filter_cap").as_int());
                    stereo_sgbm_->setUniquenessRatio(this->get_parameter("uniqueness_ratio").as_int());
                    stereo_sgbm_->setSpeckleWindowSize(this->get_parameter("speckle_window_size").as_int());
                    stereo_sgbm_->setSpeckleRange(this->get_parameter("speckle_range").as_int());
                    stereo_sgbm_->setMode(this->get_parameter("mode").as_int());
                    RCLCPP_INFO(this->get_logger(), "StereoSGBM initialized successfully");
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize stereo matcher: %s", e.what());
        }
    }

    void stereoCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& left_image,
        const sensor_msgs::msg::Image::ConstSharedPtr& right_image,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& left_info,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& right_info)
    {
        // Basic safety checks
        if (!left_image || !right_image) {
            RCLCPP_WARN(this->get_logger(), "Received null image pointers");
            return;
        }
        
        if (left_image->data.empty() || right_image->data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty image data");
            return;
        }
        
        try {
            RCLCPP_DEBUG(this->get_logger(), "Processing stereo pair");
            
            // Convert ROS images to OpenCV
            cv_bridge::CvImagePtr left_ptr = cv_bridge::toCvCopy(left_image, sensor_msgs::image_encodings::MONO8);
            cv_bridge::CvImagePtr right_ptr = cv_bridge::toCvCopy(right_image, sensor_msgs::image_encodings::MONO8);
            
            if (!left_ptr || !right_ptr) {
                RCLCPP_ERROR(this->get_logger(), "Failed to convert images");
                return;
            }
            
            cv::Mat left_gray = left_ptr->image;
            cv::Mat right_gray = right_ptr->image;
            
            if (left_gray.empty() || right_gray.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Converted images are empty");
                return;
            }
            
            // Compute disparity
            cv::Mat disparity;
            
            int algorithm = this->get_parameter("stereo_algorithm").as_int();
            if (algorithm == 0 && stereo_bm_) {
                stereo_bm_->compute(left_gray, right_gray, disparity);
            } else if (algorithm == 1 && stereo_sgbm_) {
                stereo_sgbm_->compute(left_gray, right_gray, disparity);
            } else {
                RCLCPP_ERROR(this->get_logger(), "No valid stereo algorithm configured");
                return;
            }
            
            if (disparity.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Disparity computation failed");
                return;
            }
            
            // Convert disparity to ROS message
            auto disparity_msg = std::make_shared<stereo_msgs::msg::DisparityImage>();
            disparity_msg->header = left_image->header;
            
            // Set disparity image info
            disparity_msg->image.header = left_image->header;
            disparity_msg->image.height = disparity.rows;
            disparity_msg->image.width = disparity.cols;
            disparity_msg->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            disparity_msg->image.is_bigendian = 0;
            disparity_msg->image.step = disparity.cols * sizeof(float);
            
            // Convert CV_16S to float
            cv::Mat disparity_float;
            disparity.convertTo(disparity_float, CV_32FC1, 1.0/16.0);
            
            // Copy data safely
            size_t data_size = disparity_float.rows * disparity_float.cols * sizeof(float);
            disparity_msg->image.data.resize(data_size);
            std::memcpy(disparity_msg->image.data.data(), disparity_float.data, data_size);
            
            // Set disparity parameters
            disparity_msg->f = this->get_parameter("focal_length").as_double();  
            disparity_msg->t = this->get_parameter("baseline").as_double();   
            disparity_msg->valid_window.x_offset = 0;
            disparity_msg->valid_window.y_offset = 0;
            disparity_msg->valid_window.width = disparity.cols;
            disparity_msg->valid_window.height = disparity.rows;
            disparity_msg->min_disparity = this->get_parameter("min_disparity").as_int();
            disparity_msg->max_disparity = this->get_parameter("min_disparity").as_int() + 
                                         this->get_parameter("num_disparities").as_int();
            disparity_msg->delta_d = 1.0/16.0;
            
            // Publish
            disparity_pub_->publish(*disparity_msg);
            RCLCPP_DEBUG(this->get_logger(), "Published disparity image");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing stereo images: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Unknown error processing stereo images");
        }
    }

    // Message filter typedefs
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image,
        sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo> SyncPolicy;

    // Subscribers
    message_filters::Subscriber<sensor_msgs::msg::Image> left_image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> left_info_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> right_info_sub_;
    
    // Synchronizer
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    // Publisher
    rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr disparity_pub_;
    
    // Stereo matchers
    cv::Ptr<cv::StereoBM> stereo_bm_;
    cv::Ptr<cv::StereoSGBM> stereo_sgbm_;
    
    // Parameter callback handle
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoDisparityNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}