/*
Run:
  ros2 run firefly-ros2-wrapper-detection yolov8_trt_node --ros-args \
    -p engine_path:=/path/yolo.plan \
    -p image_topic:=/camera/image_raw \
    -p input_width:=1440 -p input_height:=1088 \
    -p conf_thresh:=0.25 -p iou_thresh:=0.45 \
    -p max_det:=300 \
    -p debug:=true

Notes:
- Supports TRT outputs shaped (1, 4+nc, N) where channels:
    0: cx, 1: cy, 2: w, 3: h   (in letterboxed image coordinates)
    4..(4+nc-1): class scores/probabilities
  Conf is max class score; class is argmax.

- Tensor names are configurable via params:
    input_tensor  (default: "images")
    output_tensor (default: "output0")

- If debug:=true, publishes an annotated image with boxes/labels:
    debug_image_topic (default: "/yolo/debug_image")
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#include "firefly_detection/yolov8_detector.hpp"
#include "multi_camera_rig_common/qos_utils.hpp"

#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using firefly_detection::Yolov8Detector;
using firefly_detection::Yolov8DetectorConfig;
using firefly_detection::Det;

class Yolov8TrtNode : public rclcpp::Node
{
public:
    Yolov8TrtNode() : Node("yolov8_trt_node")
    {
        // Declare parameters
        engine_path_ = declare_parameter<std::string>("engine_path", "");
        image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw");
        det_topic_ = declare_parameter<std::string>("detection_topic", "/yolo/detections");
        
        input_tensor_ = declare_parameter<std::string>("input_tensor", "images");
        output_tensor_ = declare_parameter<std::string>("output_tensor", "output0");

        input_w_ = declare_parameter<int>("input_width", 1440);
        input_h_ = declare_parameter<int>("input_height", 1088);
        stride_ = declare_parameter<int>("stride", 32);
        scaleup_ = declare_parameter<bool>("scaleup", true);

        conf_thresh_ = declare_parameter<double>("conf_thresh", 0.25);
        iou_thresh_ = declare_parameter<double>("iou_thresh", 0.45);
        max_det_ = declare_parameter<int>("max_det", 300);

        debug_ = declare_parameter<bool>("debug", false);
        debug_image_topic_ = declare_parameter<std::string>("debug_image_topic", "/yolo/debug_image");

        // Scale output parameters
        scale_output_ = declare_parameter<bool>("scale_output", false);
        output_width_ = declare_parameter<int>("output_width", 896);
        output_height_ = declare_parameter<int>("output_height", 672);
        det_topic_scaled_ = declare_parameter<std::string>("detection_topic_scaled", "/yolo/detections_scaled");

        // QoS params
        sub_rel_ = declare_parameter<std::string>("sub_qos.reliability", "best_effort");
        sub_dur_ = declare_parameter<std::string>("sub_qos.durability", "volatile");
        sub_hist_ = declare_parameter<std::string>("sub_qos.history", "keep_last");
        sub_depth_ = declare_parameter<int>("sub_qos.depth", 5);

        pub_rel_ = declare_parameter<std::string>("pub_qos.reliability", "reliable");
        pub_dur_ = declare_parameter<std::string>("pub_qos.durability", "volatile");
        pub_hist_ = declare_parameter<std::string>("pub_qos.history", "keep_last");
        pub_depth_ = declare_parameter<int>("pub_qos.depth", 5);

        if (engine_path_.empty())
            throw std::runtime_error("engine_path param is empty");

        // Create detector config
        Yolov8DetectorConfig config;
        config.engine_path = engine_path_;
        config.input_tensor = input_tensor_;
        config.output_tensor = output_tensor_;
        config.input_w = input_w_;
        config.input_h = input_h_;
        config.stride = stride_;
        config.scaleup = scaleup_;
        config.conf_thresh = static_cast<float>(conf_thresh_);
        config.iou_thresh = static_cast<float>(iou_thresh_);
        config.max_det = max_det_;

        // Initialize detector
        detector_ = std::make_unique<Yolov8Detector>(config);

        // QoS profiles
        auto sub_qos = multi_camera_rig_common::makeQos(sub_rel_, sub_dur_, sub_hist_, sub_depth_);
        auto pub_qos = multi_camera_rig_common::makeQos(pub_rel_, pub_dur_, pub_hist_, pub_depth_);

        // Publishers
        pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(det_topic_, pub_qos);
        if (scale_output_)
        {
            pub_scaled_ = create_publisher<vision_msgs::msg::Detection2DArray>(det_topic_scaled_, pub_qos);
            RCLCPP_INFO(get_logger(), "Scaled detection publishing enabled: %s (scale to %dx%d)",
                        det_topic_scaled_.c_str(), output_width_, output_height_);
        }
        if (debug_)
        {
            debug_img_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, pub_qos);
            RCLCPP_INFO(get_logger(), "Debug image publishing enabled: %s", debug_image_topic_.c_str());
        }

        // Subscriber
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic_, sub_qos,
            std::bind(&Yolov8TrtNode::onImage, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "QoS: sub(reliability=%s durability=%s history=%s depth=%d) pub(reliability=%s durability=%s history=%s depth=%d)",
                    sub_rel_.c_str(), sub_dur_.c_str(), sub_hist_.c_str(), sub_depth_,
                    pub_rel_.c_str(), pub_dur_.c_str(), pub_hist_.c_str(), pub_depth_);

        RCLCPP_INFO(get_logger(), "Using tensors: input_tensor=%s output_tensor=%s",
                    input_tensor_.c_str(), output_tensor_.c_str());

        RCLCPP_INFO(get_logger(), "Ready. Subscribing to %s, publishing %s. Input=%dx%d (W x H).",
                    image_topic_.c_str(), det_topic_.c_str(), input_w_, input_h_);
    }

private:
    void publishDebugImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                           const cv::Mat &bgr,
                           const std::vector<Det> &dets)
    {
        if (!debug_ || !debug_img_pub_)
            return;

        cv::Mat vis = bgr.clone();

        for (const auto &d : dets)
        {
            cv::rectangle(vis,
                          cv::Point((int)d.x1, (int)d.y1),
                          cv::Point((int)d.x2, (int)d.y2),
                          cv::Scalar(0, 255, 0), 2);

            std::ostringstream oss;
            oss << "cls:" << d.cls << " " << std::fixed << std::setprecision(2) << d.conf;
            const std::string label = oss.str();

            cv::putText(vis, label,
                        cv::Point((int)d.x1, std::max(0, (int)d.y1 - 5)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }

        sensor_msgs::msg::Image out_img;
        out_img.header = msg->header;

        cv_bridge::CvImage cv_out(out_img.header, "bgr8", vis);
        debug_img_pub_->publish(*cv_out.toImageMsg());
    }

    void onImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        cv_bridge::CvImageConstPtr cvp;
        try
        {
            cvp = cv_bridge::toCvShare(msg, "bgr8");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge failed: %s", e.what());
            return;
        }

        const auto t0 = now();

        // Run detection
        std::vector<Det> dets;
        detector_->detect(cvp->image, dets);

        const auto t1 = now();

        // Publish detections
        vision_msgs::msg::Detection2DArray out_msg;
        out_msg.header = msg->header;
        out_msg.detections.reserve(dets.size());

        for (const auto &d : dets)
        {
            vision_msgs::msg::Detection2D det;
            det.header = msg->header;

            const float cx = 0.5f * (d.x1 + d.x2);
            const float cy = 0.5f * (d.y1 + d.y2);
            const float w = (d.x2 - d.x1);
            const float h = (d.y2 - d.y1);

            det.bbox.center.position.x = cx;
            det.bbox.center.position.y = cy;
            det.bbox.size_x = w;
            det.bbox.size_y = h;

            vision_msgs::msg::ObjectHypothesisWithPose hyp;
            hyp.hypothesis.class_id = std::to_string(d.cls);
            hyp.hypothesis.score = d.conf;
            det.results.push_back(hyp);

            out_msg.detections.push_back(det);
        }

        pub_->publish(out_msg);

        // Publish scaled detections if enabled
        if (scale_output_ && pub_scaled_)
        {
            // Get image dimensions for scaling
            const int orig_width = cvp->image.cols;
            const int orig_height = cvp->image.rows;
            const double scale_x = static_cast<double>(output_width_) / static_cast<double>(orig_width);
            const double scale_y = static_cast<double>(output_height_) / static_cast<double>(orig_height);

            // Scale detections
            std::vector<Det> dets_scaled;
            Yolov8Detector::scaleDetections(dets, scale_x, scale_y, dets_scaled);

            // Publish scaled detections
            vision_msgs::msg::Detection2DArray scaled_msg;
            scaled_msg.header = msg->header;
            scaled_msg.detections.reserve(dets_scaled.size());

            for (const auto &d : dets_scaled)
            {
                vision_msgs::msg::Detection2D det;
                det.header = msg->header;

                const float cx = 0.5f * (d.x1 + d.x2);
                const float cy = 0.5f * (d.y1 + d.y2);
                const float w = (d.x2 - d.x1);
                const float h = (d.y2 - d.y1);

                det.bbox.center.position.x = cx;
                det.bbox.center.position.y = cy;
                det.bbox.size_x = w;
                det.bbox.size_y = h;

                vision_msgs::msg::ObjectHypothesisWithPose hyp;
                hyp.hypothesis.class_id = std::to_string(d.cls);
                hyp.hypothesis.score = d.conf;
                det.results.push_back(hyp);

                scaled_msg.detections.push_back(det);
            }

            pub_scaled_->publish(scaled_msg);
        }

        // Publish debug annotated image if enabled
        publishDebugImage(msg, cvp->image, dets);

        const auto t2 = now();
        const double detect_ms = (t1 - t0).seconds() * 1e3;
        const double publish_ms = (t2 - t1).seconds() * 1e3;

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "detect %.1f ms | publish %.1f ms | dets %zu",
                             detect_ms, publish_ms, dets.size());
    }

    // Params
    std::string engine_path_;
    std::string image_topic_;
    std::string det_topic_;
    std::string input_tensor_;
    std::string output_tensor_;

    int input_w_{1440};
    int input_h_{1088};
    int stride_{32};
    bool scaleup_{true};

    double conf_thresh_{0.25};
    double iou_thresh_{0.45};
    int max_det_{300};

    bool debug_{false};
    std::string debug_image_topic_{"/yolo/debug_image"};

    // Scale output parameters
    bool scale_output_{false};
    int output_width_{896};
    int output_height_{672};
    std::string det_topic_scaled_{"/yolo/detections_scaled"};

    // Detector
    std::unique_ptr<Yolov8Detector> detector_;

    // QoS params
    std::string sub_rel_, sub_dur_, sub_hist_;
    int sub_depth_{5};
    std::string pub_rel_, pub_dur_, pub_hist_;
    int pub_depth_{5};

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_scaled_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Yolov8TrtNode>());
    rclcpp::shutdown();
    return 0;
}
