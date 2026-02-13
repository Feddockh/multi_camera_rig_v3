/**
 * @file aruco_detection_node.cpp
 * @brief ROS2 node for ArUco marker detection using OpenCV
 *
 * Subscribes to an image topic, detects ArUco markers, estimates their 3D pose,
 * transforms to map frame, and publishes an annotated image with detected markers drawn.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Dense>
#include <string>
#include <map>
#include <set>
#include <chrono>
#include <mutex>
#include <memory>
#include <fstream>

namespace multi_camera_rig_bringup
{

    /**
     * @brief Simple Exponential Moving Average filter for 3D position tracking
     * Appropriate for static landmarks observed by a moving camera
     */
    class EMAFilter3D
    {
    public:
        EMAFilter3D(double alpha = 0.2) : alpha_(alpha), initialized_(false)
        {
            position_ = Eigen::Vector3d::Zero();
        }

        void setAlpha(double alpha) { alpha_ = std::clamp(alpha, 0.0, 1.0); }
        double getAlpha() const { return alpha_; }

        void update(const Eigen::Vector3d &measurement)
        {
            if (!initialized_)
            {
                // Initialize with first measurement
                position_ = measurement;
                initialized_ = true;
                return;
            }

            // EMA: filtered = alpha * measurement + (1 - alpha) * previous_filtered
            // Higher alpha = more responsive to new measurements
            // Lower alpha = more smoothing
            position_ = alpha_ * measurement + (1.0 - alpha_) * position_;
        }

        Eigen::Vector3d getPosition() const { return position_; }
        bool isInitialized() const { return initialized_; }

    private:
        Eigen::Vector3d position_; // Filtered position
        double alpha_;             // Smoothing factor (0 = max smoothing, 1 = no smoothing)
        bool initialized_;
    };

    struct MarkerTracker
    {
        int id;
        int class_id; // -1 for unknown
        EMAFilter3D ema;
        rclcpp::Time last_update;
        Eigen::Vector3d raw_position;
    };

    class ArucoDetectionNode : public rclcpp::Node
    {
    public:
        ArucoDetectionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node("aruco_detection_node", options),
              tf_buffer_(this->get_clock()),
              tf_listener_(tf_buffer_)
        {            
            // ArUco detector parameters for fine-tuning
            this->declare_parameter<int>("corner_refinement_method", 1); // 0=NONE, 1=SUBPIX, 2=CONTOUR
            this->declare_parameter<int>("corner_refinement_win_size", 5);
            this->declare_parameter<int>("corner_refinement_max_iterations", 30);
            this->declare_parameter<double>("corner_refinement_min_accuracy", 0.1);

            this->declare_parameter<int>("adaptive_thresh_win_size_min", 3);
            this->declare_parameter<int>("adaptive_thresh_win_size_max", 23);
            this->declare_parameter<int>("adaptive_thresh_win_size_step", 10);
            this->declare_parameter<double>("adaptive_thresh_constant", 7.0);

            this->declare_parameter<double>("min_marker_perimeter_rate", 0.03);
            this->declare_parameter<double>("max_marker_perimeter_rate", 4.0);
            this->declare_parameter<double>("polygonal_approx_accuracy_rate", 0.03);

            this->declare_parameter<double>("min_corner_distance_rate", 0.01);
            this->declare_parameter<int>("min_distance_to_border", 1);
            this->declare_parameter<double>("min_marker_distance_rate", 0.01);
            this->declare_parameter<int>("marker_border_bits", 1);

            this->declare_parameter<double>("max_erroneous_bits_in_border_rate", 0.35);
            this->declare_parameter<double>("error_correction_rate", 0.5);
            
            this->declare_parameter<int>("perspective_remove_pixel_per_cell", 4);
            this->declare_parameter<double>("perspective_remove_ignored_margin_per_cell", 0.13);

            // Declare parameters
            this->declare_parameter<std::string>("image_topic", "/firefly_left/image_rect");
            this->declare_parameter<std::string>("camera_info_topic", "/firefly_left/camera_info_rect");
            this->declare_parameter<std::string>("det_topic", "/firefly_left/aruco_det");
            this->declare_parameter<std::string>("marker_topic", "/aruco_markers");
            this->declare_parameter<std::string>("dictionary", "DICT_4X4_50");
            this->declare_parameter<std::string>("map_frame", "map");
            this->declare_parameter<double>("marker_size", 0.05);
            this->declare_parameter<double>("max_process_rate_hz", 2.0);
            this->declare_parameter<bool>("draw_rejected", false);
            this->declare_parameter<double>("ema_alpha", 0.2);
            this->declare_parameter<std::vector<int64_t>>("marker_ids", std::vector<int64_t>{});
            this->declare_parameter<std::vector<int64_t>>("marker_class_ids", std::vector<int64_t>{});
            this->declare_parameter<std::string>("marker_output_file", "aruco_marker_positions.yaml");

            // Get parameters
            image_topic_ = this->get_parameter("image_topic").as_string();
            camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
            det_topic_ = this->get_parameter("det_topic").as_string();
            marker_topic_ = this->get_parameter("marker_topic").as_string();
            dict_name_ = this->get_parameter("dictionary").as_string();
            map_frame_ = this->get_parameter("map_frame").as_string();
            marker_size_ = this->get_parameter("marker_size").as_double();
            max_process_rate_ = this->get_parameter("max_process_rate_hz").as_double();
            draw_rejected_ = this->get_parameter("draw_rejected").as_bool();
            ema_alpha_ = this->get_parameter("ema_alpha").as_double();
            auto marker_ids = this->get_parameter("marker_ids").as_integer_array();
            auto marker_class_ids = this->get_parameter("marker_class_ids").as_integer_array();
            
            if (marker_ids.size() != marker_class_ids.size()) {
                RCLCPP_ERROR(this->get_logger(), 
                    "marker_ids and marker_class_ids must have the same length! Got %zu and %zu",
                    marker_ids.size(), marker_class_ids.size());
                throw std::runtime_error("Mismatched marker_ids and marker_class_ids array lengths");
            }
            
            for (size_t i = 0; i < marker_ids.size(); ++i) {
                int marker_id = static_cast<int>(marker_ids[i]);
                int class_id = static_cast<int>(marker_class_ids[i]);
                marker_to_class_[marker_id] = class_id;
                RCLCPP_INFO(this->get_logger(), "Marker %d -> Class %d", marker_id, class_id);
            }

            output_file_ = this->get_parameter("marker_output_file").as_string();

            // Initialize ArUco detector
            initArucoDictionary();

            // Create publishers
            pub_det_ = this->create_publisher<sensor_msgs::msg::Image>(det_topic_, 10);
            pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);

            // Create subscribers with sensor QoS
            rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
            sub_img_ = this->create_subscription<sensor_msgs::msg::Image>(
                image_topic_, sensor_qos,
                std::bind(&ArucoDetectionNode::imageCallback, this, std::placeholders::_1));

            sub_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                camera_info_topic_, sensor_qos,
                std::bind(&ArucoDetectionNode::cameraInfoCallback, this, std::placeholders::_1));

            // Create timer to publish markers continuously at 1 Hz
            marker_publish_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000),
                std::bind(&ArucoDetectionNode::publishMarkersTimerCallback, this));

            last_process_time_ = this->now();

            RCLCPP_INFO(this->get_logger(), "Listening: %s", image_topic_.c_str());
            RCLCPP_INFO(this->get_logger(), "Camera info: %s", camera_info_topic_.c_str());
            RCLCPP_INFO(this->get_logger(), "Publishing detections: %s", det_topic_.c_str());
            RCLCPP_INFO(this->get_logger(), "Publishing markers: %s", marker_topic_.c_str());
            RCLCPP_INFO(this->get_logger(), "Dictionary: %s", dict_name_.c_str());
            RCLCPP_INFO(this->get_logger(), "Map frame: %s", map_frame_.c_str());
            RCLCPP_INFO(this->get_logger(), "Marker size: %.3f m", marker_size_);
            RCLCPP_INFO(this->get_logger(), "Max process rate: %.1f Hz", max_process_rate_);
            RCLCPP_INFO(this->get_logger(), "EMA alpha: %.2f", ema_alpha_);
        }

    private:
        void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg)
        {
            if (camera_matrix_.empty())
            {
                // Extract camera matrix and distortion coefficients
                camera_matrix_ = cv::Mat(3, 3, CV_64F);
                for (int i = 0; i < 9; i++)
                {
                    camera_matrix_.at<double>(i / 3, i % 3) = msg->k[i];
                }

                dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F);
                for (size_t i = 0; i < msg->d.size(); i++)
                {
                    dist_coeffs_.at<double>(i) = msg->d[i];
                }

                camera_frame_ = msg->header.frame_id;

                RCLCPP_INFO(this->get_logger(), "Received camera calibration from frame: %s", camera_frame_.c_str());
                RCLCPP_INFO(this->get_logger(), "Camera matrix:\n[%.2f, %.2f, %.2f]\n[%.2f, %.2f, %.2f]\n[%.2f, %.2f, %.2f]",
                            camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(0, 1), camera_matrix_.at<double>(0, 2),
                            camera_matrix_.at<double>(1, 0), camera_matrix_.at<double>(1, 1), camera_matrix_.at<double>(1, 2),
                            camera_matrix_.at<double>(2, 0), camera_matrix_.at<double>(2, 1), camera_matrix_.at<double>(2, 2));
            }
        }

        void initArucoDictionary()
        {
            // Map of dictionary names to OpenCV constants
            static const std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> dict_map = {
                {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
                {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
                {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
                {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
                {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
                {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
                {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
                {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
                {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
                {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
                {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
                {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
                {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
                {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
                {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
                {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
                {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
                {"DICT_APRILTAG_16H5", cv::aruco::DICT_APRILTAG_16h5},
                {"DICT_APRILTAG_25H9", cv::aruco::DICT_APRILTAG_25h9},
                {"DICT_APRILTAG_36H10", cv::aruco::DICT_APRILTAG_36h10},
                {"DICT_APRILTAG_36H11", cv::aruco::DICT_APRILTAG_36h11},
            };

            // Convert to uppercase for case-insensitive matching
            std::string dict_upper = dict_name_;
            std::transform(dict_upper.begin(), dict_upper.end(), dict_upper.begin(), ::toupper);

            auto it = dict_map.find(dict_upper);
            if (it == dict_map.end())
            {
                std::string available;
                for (const auto &kv : dict_map)
                {
                    available += kv.first + ", ";
                }
                RCLCPP_ERROR(this->get_logger(),
                             "Unknown dictionary '%s'. Available: %s", dict_name_.c_str(), available.c_str());
                throw std::runtime_error("Unknown ArUco dictionary: " + dict_name_);
            }

            aruco_dict_ = cv::aruco::getPredefinedDictionary(it->second);
            aruco_params_ = cv::aruco::DetectorParameters::create();
            
            // Apply custom detector parameters
            aruco_params_->cornerRefinementMethod = this->get_parameter("corner_refinement_method").as_int();
            aruco_params_->cornerRefinementWinSize = this->get_parameter("corner_refinement_win_size").as_int();
            aruco_params_->cornerRefinementMaxIterations = this->get_parameter("corner_refinement_max_iterations").as_int();
            aruco_params_->cornerRefinementMinAccuracy = this->get_parameter("corner_refinement_min_accuracy").as_double();

            aruco_params_->adaptiveThreshWinSizeMin = this->get_parameter("adaptive_thresh_win_size_min").as_int();
            aruco_params_->adaptiveThreshWinSizeMax = this->get_parameter("adaptive_thresh_win_size_max").as_int();
            aruco_params_->adaptiveThreshWinSizeStep = this->get_parameter("adaptive_thresh_win_size_step").as_int();
            aruco_params_->adaptiveThreshConstant = this->get_parameter("adaptive_thresh_constant").as_double();
            
            aruco_params_->minMarkerPerimeterRate = this->get_parameter("min_marker_perimeter_rate").as_double();
            aruco_params_->maxMarkerPerimeterRate = this->get_parameter("max_marker_perimeter_rate").as_double();
            aruco_params_->polygonalApproxAccuracyRate = this->get_parameter("polygonal_approx_accuracy_rate").as_double();

            aruco_params_->minCornerDistanceRate = this->get_parameter("min_corner_distance_rate").as_double();
            aruco_params_->minDistanceToBorder = this->get_parameter("min_distance_to_border").as_int();
            aruco_params_->minMarkerDistanceRate = this->get_parameter("min_marker_distance_rate").as_double();
            aruco_params_->markerBorderBits = this->get_parameter("marker_border_bits").as_int();

            aruco_params_->maxErroneousBitsInBorderRate = this->get_parameter("max_erroneous_bits_in_border_rate").as_double();
            aruco_params_->errorCorrectionRate = this->get_parameter("error_correction_rate").as_double();

            aruco_params_->perspectiveRemovePixelPerCell = this->get_parameter("perspective_remove_pixel_per_cell").as_int();
            aruco_params_->perspectiveRemoveIgnoredMarginPerCell = this->get_parameter("perspective_remove_ignored_margin_per_cell").as_double();

            RCLCPP_INFO(this->get_logger(), "Initialized ArUco dictionary: %s", dict_upper.c_str());
            RCLCPP_INFO(this->get_logger(), "Corner refinement method: %d (0=NONE, 1=SUBPIX, 2=CONTOUR)", 
                        aruco_params_->cornerRefinementMethod);
            RCLCPP_INFO(this->get_logger(), "Adaptive threshold constant: %.1f", aruco_params_->adaptiveThreshConstant);
            RCLCPP_INFO(this->get_logger(), "Min/Max marker perimeter rate: %.3f / %.3f", 
                        aruco_params_->minMarkerPerimeterRate, aruco_params_->maxMarkerPerimeterRate);
            RCLCPP_INFO(this->get_logger(), "Error correction rate: %.2f", aruco_params_->errorCorrectionRate);
            RCLCPP_INFO(this->get_logger(), "Marker border bits: %d", aruco_params_->markerBorderBits);
        }

        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
        {
            frame_count_++;

            // Skip if we don't have camera calibration yet
            if (camera_matrix_.empty())
            {
                if (frame_count_ % 30 == 0)
                {
                    RCLCPP_WARN(this->get_logger(), "Waiting for camera calibration...");
                }
                return;
            }

            // Rate limiting - skip processing if called too frequently
            auto now = this->now();
            double elapsed = (now - last_process_time_).seconds();
            if (max_process_rate_ > 0.0 && elapsed < (1.0 / max_process_rate_))
            {
                return;
            }

            // Prevent concurrent processing
            if (!processing_mutex_.try_lock())
            {
                return;
            }
            std::lock_guard<std::mutex> lock(processing_mutex_, std::adopt_lock);

            last_process_time_ = now;

            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (const cv_bridge::Exception &e)
            {
                RCLCPP_WARN(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            cv::Mat &img = cv_ptr->image;
            cv::Mat gray;
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

            // Detect markers
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners, rejected;
            cv::aruco::detectMarkers(gray, aruco_dict_, corners, ids, aruco_params_, rejected);

            // Estimate poses if markers detected
            std::vector<cv::Vec3d> rvecs, tvecs;
            if (!ids.empty())
            {
                cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

                // Transform each marker to map frame and update EMA filter
                for (size_t i = 0; i < ids.size(); i++)
                {
                    geometry_msgs::msg::TransformStamped transform_stamped;

                    try
                    {
                        // Get transform at image timestamp
                        transform_stamped = tf_buffer_.lookupTransform(
                            map_frame_, camera_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));

                        // Create point in camera frame from tvec
                        geometry_msgs::msg::PointStamped point_camera;
                        point_camera.header.frame_id = camera_frame_;
                        point_camera.header.stamp = msg->header.stamp;
                        point_camera.point.x = tvecs[i][0];
                        point_camera.point.y = tvecs[i][1];
                        point_camera.point.z = tvecs[i][2];

                        // Transform to map frame
                        geometry_msgs::msg::PointStamped point_map;
                        tf2::doTransform(point_camera, point_map, transform_stamped);

                        // Update or create tracker for this marker
                        int marker_id = ids[i];
                        if (marker_trackers_.find(marker_id) == marker_trackers_.end())
                        {
                            // Create new tracker
                            MarkerTracker tracker;
                            tracker.id = marker_id;
                            tracker.class_id = getClassId(marker_id);
                            tracker.ema.setAlpha(ema_alpha_);
                            tracker.last_update = now;
                            marker_trackers_[marker_id] = tracker;
                            RCLCPP_INFO(this->get_logger(), "Started tracking marker %d (class %d)",
                                        marker_id, tracker.class_id);
                        }

                        auto &tracker = marker_trackers_[marker_id];
                        tracker.raw_position = Eigen::Vector3d(
                            point_map.point.x, point_map.point.y, point_map.point.z);

                        // Update EMA filter
                        tracker.ema.update(tracker.raw_position);
                        tracker.last_update = now;

                        // Get filtered position
                        Eigen::Vector3d filtered_pos = tracker.ema.getPosition();

                        // Print marker position (filtered)
                        RCLCPP_INFO(this->get_logger(),
                                    "Marker %d - Raw: [%.3f, %.3f, %.3f] | Filtered: [%.3f, %.3f, %.3f]",
                                    marker_id,
                                    tracker.raw_position.x(), tracker.raw_position.y(), tracker.raw_position.z(),
                                    filtered_pos.x(), filtered_pos.y(), filtered_pos.z());
                    }
                    catch (const tf2::TransformException &ex)
                    {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                             "Could not transform from %s to %s: %s",
                                             camera_frame_.c_str(), map_frame_.c_str(), ex.what());
                    }
                }
            }

            // Log detection info (throttled)
            if (frame_count_ % 30 == 0)
            {
                RCLCPP_INFO(this->get_logger(),
                            "Frame %zu: Detected=%zu, Rejected=%zu, Tracked markers=%zu",
                            frame_count_, ids.size(), rejected.size(), marker_trackers_.size());
            }

            // Draw detected markers and axes
            cv::Mat output_img = img.clone();
            if (!ids.empty())
            {
                cv::aruco::drawDetectedMarkers(output_img, corners, ids);

                // Draw 3D axes on each marker
                for (size_t i = 0; i < ids.size(); i++)
                {
                    cv::aruco::drawAxis(output_img, camera_matrix_, dist_coeffs_,
                                        rvecs[i], tvecs[i], marker_size_ * 0.5f);
                }
            }

            // Optionally draw rejected candidates
            if (draw_rejected_ && !rejected.empty())
            {
                for (const auto &contour : rejected)
                {
                    std::vector<cv::Point> pts(contour.begin(), contour.end());
                    cv::polylines(output_img, pts, true, cv::Scalar(0, 0, 255), 2);
                }
            }

            // Publish annotated image
            try
            {
                auto output_msg = cv_bridge::CvImage(msg->header, "bgr8", output_img).toImageMsg();
                pub_det_->publish(*output_msg);
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(this->get_logger(), "Failed to publish detection image: %s", e.what());
            }
        }

        void publishMarkersTimerCallback()
        {
            auto stamp = this->now();
            publishMarkers(stamp);
            saveMarkersToFile();
        }

        int getClassId(int marker_id) const
        {
            auto it = marker_to_class_.find(marker_id);
            if (it != marker_to_class_.end())
            {
                return it->second;
            }
            return -1; // Unknown class
        }

        void saveMarkersToFile()
        {
            if (marker_trackers_.empty())
                return;

            std::ofstream file(output_file_);
            if (!file.is_open())
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                     "Failed to open output file: %s", output_file_.c_str());
                return;
            }

            file << "# ArUco Marker Positions\n";
            file << "# Generated at: " << this->now().seconds() << "\n";
            file << "markers:\n";

            for (const auto &[id, tracker] : marker_trackers_)
            {
                if (!tracker.ema.isInitialized())
                    continue;

                Eigen::Vector3d pos = tracker.ema.getPosition();
                double time_since_update = (this->now() - tracker.last_update).seconds();

                file << "  - id: " << id << "\n";
                file << "    class_id: " << tracker.class_id << "\n";
                file << "    position:\n";
                file << "      x: " << pos.x() << "\n";
                file << "      y: " << pos.y() << "\n";
                file << "      z: " << pos.z() << "\n";
                file << "    last_seen: " << time_since_update << "  # seconds ago\n";
            }

            file.close();
        }

        void publishMarkers(const rclcpp::Time &stamp)
        {
            visualization_msgs::msg::MarkerArray marker_array;

            // Color map for markers (cycle through colors)
            std::vector<std::array<float, 3>> colors = {
                {1.0, 0.0, 0.0}, // Red
                {0.0, 1.0, 0.0}, // Green
                {0.0, 0.0, 1.0}, // Blue
                {1.0, 1.0, 0.0}, // Yellow
                {1.0, 0.0, 1.0}, // Magenta
                {0.0, 1.0, 1.0}, // Cyan
                {1.0, 0.5, 0.0}, // Orange
                {0.5, 0.0, 1.0}, // Purple
            };

            for (const auto &[id, tracker] : marker_trackers_)
            {
                if (!tracker.ema.isInitialized())
                    continue;

                Eigen::Vector3d pos = tracker.ema.getPosition();

                // Create sphere marker for filtered position
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = map_frame_;
                marker.header.stamp = stamp;
                marker.ns = "aruco_markers";
                marker.id = id;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;

                marker.pose.position.x = pos.x();
                marker.pose.position.y = pos.y();
                marker.pose.position.z = pos.z();
                marker.pose.orientation.w = 1.0;

                marker.scale.x = marker_size_;
                marker.scale.y = marker_size_;
                marker.scale.z = marker_size_;

                // Color by class ID
                auto color = colors[tracker.class_id % colors.size()];
                marker.color.r = color[0];
                marker.color.g = color[1];
                marker.color.b = color[2];
                marker.color.a = 1.0;

                marker.lifetime = rclcpp::Duration::from_seconds(0); // Persistent
                marker_array.markers.push_back(marker);

                // Create text marker with ID
                visualization_msgs::msg::Marker text_marker;
                text_marker.header = marker.header;
                text_marker.ns = "aruco_ids";
                text_marker.id = id;
                text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                text_marker.action = visualization_msgs::msg::Marker::ADD;

                text_marker.pose.position.x = pos.x();
                text_marker.pose.position.y = pos.y();
                text_marker.pose.position.z = pos.z() + marker_size_ * 1.5;
                text_marker.pose.orientation.w = 1.0;

                text_marker.scale.z = marker_size_ * 0.5;
                text_marker.color.r = 1.0;
                text_marker.color.g = 1.0;
                text_marker.color.b = 1.0;
                text_marker.color.a = 1.0;

                text_marker.text = std::to_string(id);
                text_marker.lifetime = rclcpp::Duration::from_seconds(0); // Persistent
                marker_array.markers.push_back(text_marker);
            }

            pub_markers_->publish(marker_array);
        }

    private:
        // Parameters
        std::string image_topic_;
        std::string camera_info_topic_;
        std::string det_topic_;
        std::string marker_topic_;
        std::string dict_name_;
        std::string map_frame_;
        std::string camera_frame_;
        std::string output_file_;
        double marker_size_{0.05};
        double max_process_rate_{10.0};
        double ema_alpha_{0.2};
        bool draw_rejected_{false};
        std::map<int, int> marker_to_class_;  // Maps marker_id -> class_id

        // ArUco detection
        cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
        cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;

        // Camera calibration
        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;

        // TF
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // ROS interfaces
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_det_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
        rclcpp::TimerBase::SharedPtr marker_publish_timer_;

        // State
        size_t frame_count_{0};
        rclcpp::Time last_process_time_;
        std::mutex processing_mutex_;
        std::map<int, MarkerTracker> marker_trackers_;
    };

} // namespace multi_camera_rig_bringup

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<multi_camera_rig_bringup::ArucoDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
