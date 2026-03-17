#include "ximea-ros2-wrapper-bringup/ximea_camera_node.hpp"

using namespace std::chrono_literals;

XimeaCameraNode::XimeaCameraNode(const rclcpp::NodeOptions &options)
    : Node("ximea_camera_node", options),
      running_(false),
      xi_handle_(nullptr),
      camera_initialized_(false),
      mid_dark_mean_(0.0f)
{
    // Declare parameters with defaults
    declare_parameter("camera_name", "ximea");
    declare_parameter("frame_id", "ximea_optical_frame");
    declare_parameter("gain", 0.0);
    declare_parameter("exposure_time", 10000);
    declare_parameter("enable_ffc", true);
    declare_parameter("device_id", 0);
    declare_parameter("trigger_timeout_ms", 5000);
    declare_parameter("ffc_dir", "");
    declare_parameter("camera_info_url", "");
    declare_parameter("pub_qos_reliability", "best_effort");

    camera_name_ = get_parameter("camera_name").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    gain_ = static_cast<float>(get_parameter("gain").as_double());
    exposure_time_ = get_parameter("exposure_time").as_int();
    enable_ffc_ = get_parameter("enable_ffc").as_bool();
    device_id_ = get_parameter("device_id").as_int();
    trigger_timeout_ms_ = get_parameter("trigger_timeout_ms").as_int();

    std::string ffc_dir = get_parameter("ffc_dir").as_string();
    if (!ffc_dir.empty() && ffc_dir[0] == '~')
    {
        const char *home = std::getenv("HOME");
        if (home != nullptr)
        {
            ffc_dir.replace(0, 1, home);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "HOME environment variable not set.");
        }
    }
    ffc_dir_ = ffc_dir;

    // Create the FFC directory inside the data directory
    if (enable_ffc_)
    {
        if (!std::filesystem::exists(ffc_dir_))
        {
            std::filesystem::create_directories(ffc_dir_);
        }
    }

    // Initialize camera info manager
    std::string camera_info_url = get_parameter("camera_info_url").as_string();
    camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
        this, camera_name_, camera_info_url);

    // Initialize Publishers (no "~/" in ROS 2; namespace from launch instead)
    std::string pub_qos_str = get_parameter("pub_qos_reliability").as_string();
    rclcpp::QoS pub_qos(10);
    if (pub_qos_str == "reliable") {
        pub_qos.reliable();
    } else {
        pub_qos.best_effort();
    }
    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
        camera_name_ + std::string("/image_raw"), pub_qos);
    camera_info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        camera_name_ + std::string("/camera_info"), pub_qos);

    RCLCPP_INFO(this->get_logger(), "XimeaCameraNode initialized for camera: %s",
                camera_name_.c_str());

    // Initialize the camera
    camera_initialized_ = initialize_camera();
    if (!camera_initialized_)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize XIMEA camera.");
        return;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "XIMEA camera initialized successfully.");
    }

    // Register dynamic parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&XimeaCameraNode::on_parameters_set, this, std::placeholders::_1));

    // Register reload_ffc service so callers can reload calibration without restarting
    srv_reload_ffc_ = this->create_service<std_srvs::srv::Trigger>(
        camera_name_ + "/reload_ffc",
        std::bind(&XimeaCameraNode::reload_ffc_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Start capture thread for hardware-triggered images
    running_ = true;
    capture_thread_ = std::thread(&XimeaCameraNode::capture_thread_func, this);
}

XimeaCameraNode::~XimeaCameraNode()
{
    // Stop capture thread
    running_ = false;
    if (capture_thread_.joinable())
    {
        capture_thread_.join();
    }

    if (camera_initialized_ && xi_handle_ != nullptr)
    {
        RCLCPP_INFO(this->get_logger(), "Stopping acquisition and closing XIMEA camera.");
        XI_RETURN stat = xiStopAcquisition(xi_handle_);
        if (stat != XI_OK)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to stop acquisition.");
        }

        stat = xiCloseDevice(xi_handle_);
        if (stat != XI_OK)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to close XIMEA camera.");
        }
    }
}

bool XimeaCameraNode::initialize_camera()
{
    XI_RETURN stat = XI_OK;

    // Open the camera by device ID
    RCLCPP_INFO(this->get_logger(), "Opening XIMEA camera with device ID: %d", device_id_);
    stat = xiOpenDevice(device_id_, &xi_handle_);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open XIMEA camera with device ID %d.",
                     device_id_);
        return false;
    }

    // Set the downsample factor
    stat = xiSetParamInt(xi_handle_, XI_PRM_DOWNSAMPLING, XI_DWN_1x1);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set downsample factor.");
        xiCloseDevice(xi_handle_);
        xi_handle_ = nullptr;
        return false;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Downsample factor set to 1x1.");
    }

    // Increase the internal buffer queue size
    stat = xiSetParamInt(xi_handle_, XI_PRM_BUFFERS_QUEUE_SIZE, 10);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set buffers queue size.");
        xiCloseDevice(xi_handle_);
        xi_handle_ = nullptr;
        return false;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Buffers queue size set to 10.");
    }

    // This parameter tells the driver how many buffers to commit to the transport layer
    stat = xiSetParamInt(xi_handle_, XI_PRM_ACQ_TRANSPORT_BUFFER_COMMIT, 20);
    if (stat != XI_OK)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to set acq_transport_buffer_commit.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Acq transport buffer commit set to 20.");
    }

    // Enable recent frame mode so that xiGetImage returns the most recent frame
    stat = xiSetParamInt(xi_handle_, XI_PRM_RECENT_FRAME, 1);
    if (stat != XI_OK)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to set recent_frame mode.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Recent frame mode enabled.");
    }

    // Configure hardware trigger
    // Select input 1 as trigger
    stat = xiSetParamInt(xi_handle_, XI_PRM_GPI_SELECTOR, 1);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set GPI selector.");
        xiCloseDevice(xi_handle_);
        xi_handle_ = nullptr;
        return false;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "GPI selector set to input 1.");
    }

    // Set GPI mode to trigger
    stat = xiSetParamInt(xi_handle_, XI_PRM_GPI_MODE, XI_GPI_TRIGGER);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set GPI mode to trigger.");
        xiCloseDevice(xi_handle_);
        xi_handle_ = nullptr;
        return false;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "GPI mode set to trigger.");
    }

    // Select trigger source (falling edge)
    stat = xiSetParamInt(xi_handle_, XI_PRM_TRG_SOURCE, XI_TRG_EDGE_FALLING);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set trigger source.");
        xiCloseDevice(xi_handle_);
        xi_handle_ = nullptr;
        return false;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Trigger source set to falling edge.");
    }

    // Set digital output 1 mode to exposure active
    stat = xiSetParamInt(xi_handle_, XI_PRM_GPO_SELECTOR, 1);
    if (stat != XI_OK)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to set GPO selector.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "GPO selector set to output 1.");
    }

    stat = xiSetParamInt(xi_handle_, XI_PRM_GPO_MODE, XI_GPO_EXPOSURE_ACTIVE);
    if (stat != XI_OK)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to set GPO mode to exposure active.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "GPO mode set to exposure active.");
    }

    // Set auto exposure/gain to off
    stat = xiSetParamInt(xi_handle_, XI_PRM_AEAG, XI_OFF);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set auto exposure/gain.");
        xiCloseDevice(xi_handle_);
        xi_handle_ = nullptr;
        return false;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Auto exposure/gain turned off.");
    }

    // Set exposure time (in microseconds)
    stat = xiSetParamInt(xi_handle_, XI_PRM_EXPOSURE, exposure_time_);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set exposure time.");
        xiCloseDevice(xi_handle_);
        xi_handle_ = nullptr;
        return false;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Exposure time set to %d us.", exposure_time_);
    }

    // Set the gain
    stat = xiSetParamFloat(xi_handle_, XI_PRM_GAIN, gain_);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set gain.");
        xiCloseDevice(xi_handle_);
        xi_handle_ = nullptr;
        return false;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Gain set to %.2f.", gain_);
    }

    // Set image format
    stat = xiSetParamInt(xi_handle_, XI_PRM_IMAGE_DATA_FORMAT, XI_RAW8);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set image format.");
        xiCloseDevice(xi_handle_);
        xi_handle_ = nullptr;
        return false;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Image format set to RAW8.");
    }

    // Initialize software FFC if enabled
    if (enable_ffc_)
    {
        if (!init_software_ffc())
        {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to initialize software FFC. Continuing without FFC.");
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Software FFC disabled.");
    }

    // Start acquisition
    RCLCPP_INFO(this->get_logger(), "Starting acquisition...");
    stat = xiStartAcquisition(xi_handle_);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to start acquisition.");
        xiCloseDevice(xi_handle_);
        xi_handle_ = nullptr;
        return false;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Acquisition started.");
    }

    return true;
}

bool XimeaCameraNode::init_software_ffc()
{
    auto latest_dark_time = std::filesystem::file_time_type::min();
    auto latest_mid_time = std::filesystem::file_time_type::min();

    // Search for the files in the FFC directory
    if (!std::filesystem::exists(ffc_dir_))
    {
        RCLCPP_WARN(this->get_logger(), "FFC directory does not exist: %s",
                    ffc_dir_.c_str());
        return false;
    }

    for (const auto &entry : std::filesystem::directory_iterator(ffc_dir_))
    {
        if (entry.is_regular_file())
        {
            std::string file_path = entry.path().string();
            auto file_time = std::filesystem::last_write_time(entry);

            if (file_path.find("_dark.tif") != std::string::npos)
            {
                if (dark_file_.empty() || file_time > latest_dark_time)
                {
                    dark_file_ = file_path;
                    latest_dark_time = file_time;
                }
            }
            else if (file_path.find("_mid.tif") != std::string::npos)
            {
                if (mid_file_.empty() || file_time > latest_mid_time)
                {
                    mid_file_ = file_path;
                    latest_mid_time = file_time;
                }
            }
        }
    }

    if (dark_file_.empty() || mid_file_.empty())
    {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to find FFC files in the directory: %s",
                    ffc_dir_.c_str());
        return false;
    }

    // Load dark and mid field images
    dark_ = cv::imread(dark_file_, cv::IMREAD_GRAYSCALE);
    mid_ = cv::imread(mid_file_, cv::IMREAD_GRAYSCALE);

    if (dark_.empty() || mid_.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load FFC files.");
        return false;
    }

    // Convert to float for computation
    dark_.convertTo(dark_, CV_32F);
    mid_.convertTo(mid_, CV_32F);

    // Compute mid - dark
    mid_dark_ = mid_ - dark_;

    // Mean of (mid - dark) for normalization
    mid_dark_mean_ = static_cast<float>(cv::mean(mid_dark_)[0]);
    FFC_ = mid_dark_mean_ / (mid_dark_ + 1e-6f);

    RCLCPP_INFO(this->get_logger(),
                "Software FFC initialized with dark: %s, mid: %s",
                dark_file_.c_str(), mid_file_.c_str());

    return true;
}

cv::Mat XimeaCameraNode::apply_software_ffc(cv::Mat &raw)
{
    // Ensure image sizes match
    if (raw.size() != dark_.size() || raw.size() != mid_.size())
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Image sizes do not match for FFC. Raw: %dx%d, Dark: %dx%d, Mid: %dx%d",
                     raw.cols, raw.rows, dark_.cols, dark_.rows, mid_.cols, mid_.rows);
        return raw;
    }

    raw.convertTo(raw, CV_32F);

    // Apply flat field correction formula
    cv::Mat corrected_f = (raw - dark_).mul(FFC_);

    // Clip values to 0-255 and convert back to 8-bit
    cv::Mat clipped, corrected;
    cv::threshold(corrected_f, clipped, 255.0, 255.0, cv::THRESH_TRUNC);
    clipped.convertTo(corrected, CV_8U);

    return corrected;
}

void XimeaCameraNode::capture_thread_func()
{
    RCLCPP_INFO(this->get_logger(),
                "Capture thread started, waiting for hardware triggers...");

    while (running_ && rclcpp::ok())
    {
        if (!camera_initialized_)
        {
            break;
        }

        cv::Mat img = capture_calibrated_image();
        if (!img.empty())
        {
            publish_image(img);
        }
        else
        {
            // If no image received, continue waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    RCLCPP_INFO(this->get_logger(), "Capture thread stopped.");
}

cv::Mat XimeaCameraNode::capture_raw_image()
{
    if (!camera_initialized_ || xi_handle_ == nullptr)
    {
        RCLCPP_WARN(this->get_logger(), "Camera is not initialized.");
        return cv::Mat();
    }

    XI_IMG image;
    memset(&image, 0, sizeof(image));
    image.size = sizeof(XI_IMG);

    // Wait for hardware-triggered image with timeout
    XI_RETURN stat = xiGetImage(xi_handle_, trigger_timeout_ms_, &image);

    if (stat != XI_OK)
    {
        if (stat == XI_TIMEOUT)
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "Timeout waiting for hardware trigger.");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to get image (error code: %d).", stat);
        }
        return cv::Mat();
    }

    if (image.bp == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "Image buffer is null.");
        return cv::Mat();
    }

    cv::Mat cvImageMono(image.height, image.width, CV_8UC1, image.bp);

    return cvImageMono.clone();
}

cv::Mat XimeaCameraNode::capture_calibrated_image()
{
    cv::Mat raw = capture_raw_image();

    // If the raw image is empty, return an empty image
    if (raw.empty())
    {
        return cv::Mat();
    }

    // If raw image size is 0, return an empty image
    if (raw.size().area() == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Raw image size is 0.");
        return cv::Mat();
    }

    // Apply software FFC if enabled and calibration files exist
    cv::Mat img;
    {
        std::lock_guard<std::mutex> lock(ffc_mutex_);
        if (enable_ffc_ && !dark_file_.empty() && !mid_file_.empty())
        {
            img = apply_software_ffc(raw);
        }
        else
        {
            img = raw;
        }
    }

    return img;
}

void XimeaCameraNode::publish_image(cv::Mat &image)
{
    // Create header with timestamp
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = frame_id_;

    // Create ROS Image message
    auto image_msg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();

    // Publish the image
    image_publisher_->publish(*image_msg);

    // Publish camera info (if available)
    if (camera_info_manager_)
    {
        auto camera_info_msg = camera_info_manager_->getCameraInfo();
        camera_info_msg.header = header;
        camera_info_publisher_->publish(camera_info_msg);
    }
}

rcl_interfaces::msg::SetParametersResult XimeaCameraNode::on_parameters_set(
    const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &p : params)
    {
        const auto &name = p.get_name();

        if (name == "gain")
        {
            float new_gain = static_cast<float>(p.as_double());
            if (camera_initialized_ && xi_handle_)
            {
                XI_RETURN stat = xiSetParamFloat(xi_handle_, XI_PRM_GAIN, new_gain);
                if (stat == XI_OK)
                {
                    gain_ = new_gain;
                    RCLCPP_INFO(this->get_logger(), "Gain updated to %.2f dB", gain_);
                }
                else
                {
                    result.successful = false;
                    result.reason = "Failed to set gain via xiAPI.";
                }
            }
        }
        else if (name == "exposure_time")
        {
            int new_exp = p.as_int();
            if (camera_initialized_ && xi_handle_)
            {
                XI_RETURN stat = xiSetParamInt(xi_handle_, XI_PRM_EXPOSURE, new_exp);
                if (stat == XI_OK)
                {
                    exposure_time_ = new_exp;
                    RCLCPP_INFO(this->get_logger(),
                                "Exposure time updated to %d us", exposure_time_);
                }
                else
                {
                    result.successful = false;
                    result.reason = "Failed to set exposure via xiAPI.";
                }
            }
        }
        else if (name == "enable_ffc")
        {
            std::lock_guard<std::mutex> lock(ffc_mutex_);
            enable_ffc_ = p.as_bool();
            RCLCPP_INFO(this->get_logger(), "enable_ffc set to %s",
                        enable_ffc_ ? "true" : "false");
        }
    }

    return result;
}

void XimeaCameraNode::reload_ffc_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Reloading FFC calibration from: %s", ffc_dir_.c_str());

    std::lock_guard<std::mutex> lock(ffc_mutex_);

    // Clear existing calibration state
    dark_file_.clear();
    mid_file_.clear();
    dark_ = cv::Mat();
    mid_ = cv::Mat();
    mid_dark_ = cv::Mat();
    FFC_ = cv::Mat();
    mid_dark_mean_ = 0.0f;

    bool ok = init_software_ffc();

    if (ok)
    {
        RCLCPP_INFO(this->get_logger(), "FFC calibration reloaded successfully.");
        response->success = true;
        response->message = "FFC calibration reloaded: dark=" + dark_file_ + ", mid=" + mid_file_;
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Failed to reload FFC calibration.");
        response->success = false;
        response->message = "Failed to reload FFC calibration. Check ffc_dir and file names.";
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<XimeaCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
