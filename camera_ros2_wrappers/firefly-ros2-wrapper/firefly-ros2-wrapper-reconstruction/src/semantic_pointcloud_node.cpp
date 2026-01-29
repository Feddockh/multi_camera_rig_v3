/*
Semantic Point Cloud Node
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
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// TODO: Add detection message import when ready
// #include <vision_msgs/msg/detection2_d_array.hpp>

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

static std::string toLower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c)
                   { return static_cast<char>(std::tolower(c)); });
    return s;
}

static rmw_qos_reliability_policy_t parseReliability(const std::string &s)
{
    const auto v = toLower(s);
    if (v == "reliable")
        return RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    if (v == "best_effort" || v == "besteffort" || v == "best-effort")
        return RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    if (v == "system_default" || v == "default")
        return RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    throw std::runtime_error("Invalid reliability: " + s);
}

static rmw_qos_durability_policy_t parseDurability(const std::string &s)
{
    const auto v = toLower(s);
    if (v == "volatile")
        return RMW_QOS_POLICY_DURABILITY_VOLATILE;
    if (v == "transient_local" || v == "transientlocal" || v == "transient-local")
        return RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    if (v == "system_default" || v == "default")
        return RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    throw std::runtime_error("Invalid durability: " + s);
}

static rmw_qos_history_policy_t parseHistory(const std::string &s)
{
    const auto v = toLower(s);
    if (v == "keep_last" || v == "keeplast")
        return RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    if (v == "keep_all" || v == "keepall")
        return RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    if (v == "system_default" || v == "default")
        return RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT;
    throw std::runtime_error("Invalid history: " + s);
}

static rclcpp::QoS makeQos(
    const std::string &reliability,
    const std::string &durability,
    const std::string &history,
    int depth)
{
    auto hist = parseHistory(history);

    rclcpp::QoS qos =
        (hist == RMW_QOS_POLICY_HISTORY_KEEP_ALL)
            ? rclcpp::QoS(rclcpp::KeepAll())
            : rclcpp::QoS(rclcpp::KeepLast(std::max(1, depth)));

    switch (parseReliability(reliability))
    {
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
        qos.reliable();
        break;
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
        qos.best_effort();
        break;
    default:
        break;
    }

    switch (parseDurability(durability))
    {
    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
        qos.durability_volatile();
        break;
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
        qos.transient_local();
        break;
    default:
        break;
    }

    return qos;
}

class SemanticPointCloudNode : public rclcpp::Node
{
public:
    SemanticPointCloudNode() : Node("semantic_pointcloud_node")
    {
        // Mode selection
        use_semantics_ = declare_parameter<bool>("use_semantics", false);

        // Stereo parameters
        baseline_ = declare_parameter<double>("baseline", 0.06);

        // Input topics
        disparity_topic_ = declare_parameter<std::string>("disparity_topic", "/stereo/disparity");
        info_topic_ = declare_parameter<std::string>("camera_info_topic", "/firefly_left/camera_info_rect_scaled");
        image_topic_ = declare_parameter<std::string>("image_topic", "/firefly_left/image_rect_scaled");
        
        // Detection topic (for semantic mode)
        detection_topic_ = declare_parameter<std::string>("detection_topic", "/detections");

        // Point cloud generation parameters
        stride_ = declare_parameter<int>("stride", 2);
        max_range_m_ = declare_parameter<double>("max_range_m", 10.0);
        use_background_ = declare_parameter<bool>("use_background", false);

        // Output topics
        publish_cloud_ = declare_parameter<bool>("publish_cloud", true);
        publish_depth_ = declare_parameter<bool>("publish_depth", true);
        cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/stereo/points");
        depth_topic_ = declare_parameter<std::string>("depth_topic", "/stereo/depth");

        // Semantic parameters
        background_class_id_ = declare_parameter<int>("background_class_id", -1);
        background_confidence_ = declare_parameter<double>("background_confidence", 0.5);

        // QoS parameters
        sub_rel_ = declare_parameter<std::string>("sub_qos.reliability", "best_effort");
        sub_dur_ = declare_parameter<std::string>("sub_qos.durability", "volatile");
        sub_hist_ = declare_parameter<std::string>("sub_qos.history", "keep_last");
        sub_depth_ = declare_parameter<int>("sub_qos.depth", 5);
        pub_rel_ = declare_parameter<std::string>("pub_qos.reliability", "best_effort");
        pub_dur_ = declare_parameter<std::string>("pub_qos.durability", "volatile");
        pub_hist_ = declare_parameter<std::string>("pub_qos.history", "keep_last");
        pub_depth_ = declare_parameter<int>("pub_qos.depth", 5);

        // Depth filtering options
        depth_filter_mode_ = declare_parameter<std::string>("depth_filter.mode", "none");
        depth_flying_ksize_ = declare_parameter<int>("depth_filter.flying_ksize", 5);
        depth_flying_tau_ = declare_parameter<double>("depth_filter.flying_tau", 0.25);
        depth_flying_min_neigh_ = declare_parameter<int>("depth_filter.flying_min_neighbors", 6);
        depth_median_ksize_ = declare_parameter<int>("depth_filter.median_ksize", 5);

        // Point cloud filtering options
        pc_filter_mode_ = declare_parameter<std::string>("pc_filter.mode", "none");
        pc_grid_ksize_ = declare_parameter<int>("pc_filter.grid_ksize", 5);
        pc_grid_tau_ = declare_parameter<double>("pc_filter.grid_tau", 0.25);
        pc_grid_min_neigh_ = declare_parameter<int>("pc_filter.grid_min_neighbors", 4);
        pc_knn_k_ = declare_parameter<int>("pc_filter.knn_k", 20);
        pc_knn_stddev_mul_ = declare_parameter<double>("pc_filter.knn_stddev_multiplier", 2.0);

        auto sub_qos = makeQos(sub_rel_, sub_dur_, sub_hist_, sub_depth_);
        auto pub_qos = makeQos(pub_rel_, pub_dur_, pub_hist_, pub_depth_);

        // Subscribers
        info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            info_topic_, sub_qos,
            std::bind(&SemanticPointCloudNode::onInfo, this, std::placeholders::_1));

        disp_sub_.subscribe(this, disparity_topic_, sub_qos.get_rmw_qos_profile());
        image_sub_.subscribe(this, image_topic_, sub_qos.get_rmw_qos_profile());

        // Publishers
        if (publish_cloud_)
            cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, pub_qos);
        if (publish_depth_)
            depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_topic_, pub_qos);

        // Synchronizer for disparity + image (and optionally detections)
        if (use_semantics_)
        {
            RCLCPP_INFO(get_logger(), "Running in SEMANTIC mode");
            // TODO: Add detection sync when detection message is available
            // For now, just do disparity + image sync
            using Policy = message_filters::sync_policies::ExactTime<
                sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
            sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(
                Policy(5), disp_sub_, image_sub_);
            sync_->registerCallback(std::bind(&SemanticPointCloudNode::onSyncCallback, this,
                                              std::placeholders::_1, std::placeholders::_2));
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Running in NORMAL mode");
            using Policy = message_filters::sync_policies::ExactTime<
                sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
            sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(
                Policy(5), disp_sub_, image_sub_);
            sync_->registerCallback(std::bind(&SemanticPointCloudNode::onSyncCallback, this,
                                              std::placeholders::_1, std::placeholders::_2));
        }

        RCLCPP_INFO(get_logger(), "Semantic PointCloud Node initialized");
        RCLCPP_INFO(get_logger(), "  Disparity: %s", disparity_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  Image: %s", image_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  Camera Info: %s", info_topic_.c_str());
        if (use_semantics_)
            RCLCPP_INFO(get_logger(), "  Detections: %s", detection_topic_.c_str());
    }

private:
    void onInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        img_w_ = msg->width;
        img_h_ = msg->height;
        have_info_ = true;
    }

    void onSyncCallback(const sensor_msgs::msg::Image::ConstSharedPtr &disp_msg,
                        const sensor_msgs::msg::Image::ConstSharedPtr &image_msg)
    {
        if (!have_info_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for camera info...");
            return;
        }

        const auto t0 = now();

        // Convert disparity image
        cv_bridge::CvImageConstPtr disp_cv;
        try
        {
            disp_cv = cv_bridge::toCvShare(disp_msg, "32FC1");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge disp failed: %s", e.what());
            return;
        }

        // Convert RGB image
        cv_bridge::CvImageConstPtr image_cv;
        try
        {
            image_cv = cv_bridge::toCvShare(image_msg, image_msg->encoding);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge image failed: %s", e.what());
            return;
        }

        const int disp_h = disp_cv->image.rows;
        const int disp_w = disp_cv->image.cols;

        // Verify image and disparity sizes match
        if (image_cv->image.rows != disp_h || image_cv->image.cols != disp_w)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                                  "Image (%dx%d) and disparity (%dx%d) size mismatch!",
                                  image_cv->image.cols, image_cv->image.rows, disp_w, disp_h);
            return;
        }

        const double fx = fx_;
        const double fy = fy_;
        const double cx = cx_;
        const double cy = cy_;
        const double B = baseline_;
        const double maxR = max_range_m_;
        const int s = std::max(1, stride_);

        const auto t1 = now();

        // Publish depth image if requested
        if (publish_depth_)
        {
            publishDepthImage(disp_cv, fx, B, maxR, disp_h, disp_w, image_msg->header);
        }

        // Publish point cloud if requested
        if (publish_cloud_)
        {
            if (use_semantics_)
            {
                // TODO: Implement semantic pointcloud when detection sync is ready
                publishSemanticPointCloud(disp_cv, image_cv, fx, fy, cx, cy, B, maxR, s,
                                          disp_h, disp_w, image_msg->header);
            }
            else
            {
                publishNormalPointCloud(disp_cv, image_cv, fx, fy, cx, cy, B, maxR, s,
                                        disp_h, disp_w, image_msg->header);
            }
        }

        const auto t2 = now();
        const double total_ms = (t2 - t0).seconds() * 1e3;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "Processing time: %.1f ms", total_ms);
    }

    void publishDepthImage(const cv_bridge::CvImageConstPtr &disp_cv,
                           double fx, double B, double maxR,
                           int out_h, int out_w,
                           const std_msgs::msg::Header &header)
    {
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

        sensor_msgs::msg::Image depth_msg;
        depth_msg.header = header;
        depth_msg.height = out_h;
        depth_msg.width = out_w;
        depth_msg.encoding = "32FC1";
        depth_msg.is_bigendian = false;
        depth_msg.step = out_w * sizeof(float);
        depth_msg.data.resize(depth.size() * sizeof(float));
        std::memcpy(depth_msg.data.data(), depth.data(), depth.size() * sizeof(float));
        depth_pub_->publish(depth_msg);
    }

    void applyDepthFiltering(std::vector<float> &depth, int out_h, int out_w)
    {
        auto dmode = toLower(depth_filter_mode_);

        if (dmode == "median")
        {
            RCLCPP_INFO_ONCE(get_logger(), "[Depth Filter] Using median filter with ksize=%d", depth_median_ksize_);
            cv::Mat dmat(out_h, out_w, CV_32FC1, depth.data());
            cv::Mat valid = (dmat == dmat) & (dmat > 0.0f);
            cv::Mat tmp = dmat.clone();
            tmp.setTo(0.0f, ~valid);

            int k = std::max(3, depth_median_ksize_ | 1);
            cv::medianBlur(tmp, tmp, k);

            tmp.setTo(std::numeric_limits<float>::quiet_NaN(), ~valid);
            std::memcpy(depth.data(), tmp.ptr<float>(0), depth.size() * sizeof(float));
        }
        else if (dmode == "flying_pixel")
        {
            cv::Mat dmat(out_h, out_w, CV_32FC1, depth.data());

            int k = std::max(3, depth_flying_ksize_ | 1);
            int r = k / 2;
            const double tau = std::max(0.0, depth_flying_tau_);
            RCLCPP_INFO_ONCE(get_logger(), "[Depth Filter] Using flying_pixel filter with ksize=%d, tau=%.2f, min_neighbors=%d",
                             k, tau, depth_flying_min_neigh_);

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
                    if (valid_n < depth_flying_min_neigh_)
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

    void publishNormalPointCloud(const cv_bridge::CvImageConstPtr &disp_cv,
                                  const cv_bridge::CvImageConstPtr &image_cv,
                                  double fx, double fy, double cx, double cy,
                                  double B, double maxR, int s,
                                  int out_h, int out_w,
                                  const std_msgs::msg::Header &header)
    {
        // Helper: pack RGB into a float (PCL convention used by RViz "RGB8")
        auto packRGBFloat = [](uint8_t r, uint8_t g, uint8_t b) -> float
        {
            const uint32_t rgb = (static_cast<uint32_t>(r) << 16) |
                                 (static_cast<uint32_t>(g) << 8) |
                                 (static_cast<uint32_t>(b));
            float f;
            std::memcpy(&f, &rgb, sizeof(float));
            return f;
        };

        // Apply pointcloud filtering to get keep mask
        std::vector<uint8_t> keep_mask((size_t)out_h * out_w, 1);
        applyPointCloudFiltering(disp_cv, fx, fy, cx, cy, B, maxR, s, out_h, out_w, keep_mask);

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
                    if (use_background_)
                        n_valid++;
                    continue;
                }

                const double Z = fx * B / static_cast<double>(d);
                if (Z <= 0.0 || Z > maxR)
                {
                    if (use_background_)
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
                    if (use_background_)
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
                        if (use_background_)
                            use_max_range = true;
                        else
                            continue;
                    }
                    else if (Z > maxR)
                    {
                        if (use_background_)
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

        cloud_pub_->publish(cloud);
    }

    void publishSemanticPointCloud(const cv_bridge::CvImageConstPtr &disp_cv,
                                    const cv_bridge::CvImageConstPtr &image_cv,
                                    double fx, double fy, double cx, double cy,
                                    double B, double maxR, int s,
                                    int out_h, int out_w,
                                    const std_msgs::msg::Header &header)
    {
        // TODO: Implement detection-based semantic labeling
        // For now, create semantic labels and confidences arrays
        // filled with background values

        const size_t N = (size_t)out_h * out_w;
        std::vector<int32_t> labels(N, background_class_id_);
        std::vector<float> confidences(N, static_cast<float>(background_confidence_));

        // TODO: When detection message is available:
        // 1. Scale detections from original resolution to current resolution
        // 2. For each detection bounding box, assign class_id and confidence
        //    to pixels within the box (highest confidence wins for overlaps)

        // Helper: pack RGB into a float
        auto packRGBFloat = [](uint8_t r, uint8_t g, uint8_t b) -> float
        {
            const uint32_t rgb = (static_cast<uint32_t>(r) << 16) |
                                 (static_cast<uint32_t>(g) << 8) |
                                 (static_cast<uint32_t>(b));
            float f;
            std::memcpy(&f, &rgb, sizeof(float));
            return f;
        };

        // Apply pointcloud filtering
        std::vector<uint8_t> keep_mask((size_t)out_h * out_w, 1);
        applyPointCloudFiltering(disp_cv, fx, fy, cx, cy, B, maxR, s, out_h, out_w, keep_mask);

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
                    if (use_background_)
                        n_valid++;
                    continue;
                }

                const double Z = fx * B / static_cast<double>(d);
                if (Z <= 0.0 || Z > maxR)
                {
                    if (use_background_)
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
                    if (use_background_)
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
                        if (use_background_)
                            use_max_range = true;
                        else
                            continue;
                    }
                    else if (Z > maxR)
                    {
                        if (use_background_)
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

        cloud_pub_->publish(cloud);
        RCLCPP_INFO_ONCE(get_logger(), "Published semantic pointcloud with %zu points", n_valid);
    }

    void applyPointCloudFiltering(const cv_bridge::CvImageConstPtr &disp_cv,
                                   double fx, double fy, double cx, double cy,
                                   double B, double maxR, int s,
                                   int out_h, int out_w,
                                   std::vector<uint8_t> &keep_mask)
    {
        auto pcmode = toLower(pc_filter_mode_);
        
        if (pcmode == "grid_outlier")
        {
            int k = std::max(3, pc_grid_ksize_ | 1);
            int r = k / 2;
            const double tau = std::max(0.0, pc_grid_tau_);
            RCLCPP_INFO_ONCE(get_logger(), "[PointCloud Filter] Using grid_outlier filter with ksize=%d, tau=%.2f, min_neighbors=%d",
                             k, tau, pc_grid_min_neigh_);

            // Build depth image from disparity
            std::vector<float> zimg((size_t)out_h * out_w, std::numeric_limits<float>::quiet_NaN());
            for (int v = 0; v < out_h; ++v)
            {
                const float *row = disp_cv->image.ptr<float>(v);
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
                    if (valid_n < pc_grid_min_neigh_)
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

private:
    // Mode
    bool use_semantics_{false};

    // Parameters
    double baseline_{0.06};
    int stride_{2};
    double max_range_m_{10.0};
    bool use_background_{false};
    int background_class_id_{-1};
    double background_confidence_{0.5};

    // Camera intrinsics
    bool have_info_{false};
    double fx_{0}, fy_{0}, cx_{0}, cy_{0};
    uint32_t img_w_{0}, img_h_{0};

    // Topics
    std::string disparity_topic_, info_topic_, image_topic_, detection_topic_;
    std::string cloud_topic_, depth_topic_;
    bool publish_cloud_{true}, publish_depth_{true};

    // QoS params
    std::string sub_rel_, sub_dur_, sub_hist_;
    int sub_depth_{5};
    std::string pub_rel_, pub_dur_, pub_hist_;
    int pub_depth_{5};

    // Filter params
    std::string depth_filter_mode_;
    int depth_flying_ksize_{5};
    double depth_flying_tau_{0.25};
    int depth_flying_min_neigh_{6};
    int depth_median_ksize_{5};

    std::string pc_filter_mode_;
    int pc_grid_ksize_{5};
    double pc_grid_tau_{0.25};
    int pc_grid_min_neigh_{4};
    int pc_knn_k_{20};
    double pc_knn_stddev_mul_{2.0};

    // ROS
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
