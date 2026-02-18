#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#include "multi_camera_rig_detection/yolo_detector.hpp"
#include "multi_camera_rig_detection/detection_utils.hpp"
#include "multi_camera_rig_common/qos_utils.hpp"

#include "multi_camera_rig_msgs/msg/instance_segmentation2_d.hpp"
#include "multi_camera_rig_msgs/msg/instance_segmentation2_d_array.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using multi_camera_rig_detection::YoloDetector;
using multi_camera_rig_detection::YoloDetectorConfig;
using multi_camera_rig_detection::Det;

class YoloTrtNode : public rclcpp::Node
{
public:
    YoloTrtNode() : Node("yolo_trt_node")
    {
        // Core params
        engine_path_   = declare_parameter<std::string>("engine_path", "");
        image_topic_   = declare_parameter<std::string>("image_topic", "/camera/image_raw");
        det_topic_     = declare_parameter<std::string>("detection_topic", "/yolo/detections");
        seg_det_topic_ = declare_parameter<std::string>("seg_detection_topic", "/yolo/instance_segmentation");

        // Tensors
        input_tensor_ = declare_parameter<std::string>("input_tensor", "images");
        output_tensor_ = declare_parameter<std::string>("output_tensor", "output0");
        proto_tensor_ = declare_parameter<std::string>("proto_tensor", "output1");

        // Task: auto|det|seg
        task_ = declare_parameter<std::string>("task", "auto");

        // Input shape
        input_w_ = declare_parameter<int>("input_width", 1440);
        input_h_ = declare_parameter<int>("input_height", 1088);
        stride_  = declare_parameter<int>("stride", 32);
        scaleup_ = declare_parameter<bool>("scaleup", true);

        // Postprocess
        conf_thresh_ = declare_parameter<double>("conf_thresh", 0.25);
        iou_thresh_  = declare_parameter<double>("iou_thresh", 0.45);
        max_det_     = declare_parameter<int>("max_det", 300);

        // Debug
        debug_            = declare_parameter<bool>("debug", false);
        debug_masks_   = declare_parameter<bool>("debug_masks", true);
        debug_image_topic_= declare_parameter<std::string>("debug_image_topic", "/yolo/debug_image");

        // Mask behavior
        publish_masks_ = declare_parameter<bool>("publish_masks", true);
        mask_alpha_    = declare_parameter<double>("mask_alpha", 0.45);
        mask_thresh_   = declare_parameter<double>("mask_thresh", 0.50);

        // Scale output parameters
        scale_output_        = declare_parameter<bool>("scale_output", false);
        output_width_        = declare_parameter<int>("output_width", 896);
        output_height_       = declare_parameter<int>("output_height", 672);
        det_topic_scaled_    = declare_parameter<std::string>("detection_topic_scaled", "/yolo/detections_scaled");
        seg_det_topic_scaled_= declare_parameter<std::string>("seg_detection_topic_scaled", "/yolo/instance_segmentation_scaled");

        // QoS params
        sub_rel_   = declare_parameter<std::string>("sub_qos.reliability", "best_effort");
        sub_dur_   = declare_parameter<std::string>("sub_qos.durability", "volatile");
        sub_hist_  = declare_parameter<std::string>("sub_qos.history", "keep_last");
        sub_depth_ = declare_parameter<int>("sub_qos.depth", 5);

        pub_rel_   = declare_parameter<std::string>("pub_qos.reliability", "reliable");
        pub_dur_   = declare_parameter<std::string>("pub_qos.durability", "volatile");
        pub_hist_  = declare_parameter<std::string>("pub_qos.history", "keep_last");
        pub_depth_ = declare_parameter<int>("pub_qos.depth", 5);

        if (engine_path_.empty())
            throw std::runtime_error("engine_path param is empty");

        // Detector config
        YoloDetectorConfig cfg;
        cfg.engine_path  = engine_path_;
        cfg.input_tensor = input_tensor_;
        cfg.output_tensor= output_tensor_;
        cfg.proto_tensor = proto_tensor_;
        cfg.task         = task_;
        cfg.input_w      = input_w_;
        cfg.input_h      = input_h_;
        cfg.stride       = stride_;
        cfg.scaleup      = scaleup_;
        cfg.conf_thresh  = conf_thresh_;
        cfg.iou_thresh   = iou_thresh_;
        cfg.max_det      = max_det_;
        cfg.mask_dim     = 32; // keep as param later if you want

        detector_ = std::make_unique<YoloDetector>(cfg);

        // QoS
        auto sub_qos = multi_camera_rig_common::makeQos(sub_rel_, sub_dur_, sub_hist_, sub_depth_);
        auto pub_qos = multi_camera_rig_common::makeQos(pub_rel_, pub_dur_, pub_hist_, pub_depth_);

        // Publishers
        pub_     = create_publisher<vision_msgs::msg::Detection2DArray>(det_topic_, pub_qos);
        pub_seg_ = create_publisher<multi_camera_rig_msgs::msg::InstanceSegmentation2DArray>(seg_det_topic_, pub_qos);

        if (scale_output_)
        {
            pub_scaled_     = create_publisher<vision_msgs::msg::Detection2DArray>(det_topic_scaled_, pub_qos);
            pub_seg_scaled_ = create_publisher<multi_camera_rig_msgs::msg::InstanceSegmentation2DArray>(seg_det_topic_scaled_, pub_qos);
            RCLCPP_INFO(get_logger(), "Scaled publishing enabled: det=%s seg=%s -> %dx%d",
                        det_topic_scaled_.c_str(), seg_det_topic_scaled_.c_str(), output_width_, output_height_);
        }

        if (debug_)
        {
            debug_img_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, pub_qos);
            RCLCPP_INFO(get_logger(), "Debug image publishing: %s", debug_image_topic_.c_str());
        }

        // Subscriber
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic_, sub_qos,
            std::bind(&YoloTrtNode::onImage, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "QoS: sub(r=%s d=%s h=%s depth=%d) pub(r=%s d=%s h=%s depth=%d)",
                    sub_rel_.c_str(), sub_dur_.c_str(), sub_hist_.c_str(), sub_depth_,
                    pub_rel_.c_str(), pub_dur_.c_str(), pub_hist_.c_str(), pub_depth_);

        RCLCPP_INFO(get_logger(), "Tensors: input=%s output0=%s proto=%s",
                    input_tensor_.c_str(), output_tensor_.c_str(), proto_tensor_.c_str());

        RCLCPP_INFO(get_logger(), "Task=%s | Segmentation=%s",
                    task_.c_str(), detector_->isSegmentation() ? "true" : "false");

        RCLCPP_INFO(get_logger(), "Ready. Sub=%s | det=%s | seg=%s | Input=%dx%d (W x H).",
                    image_topic_.c_str(), det_topic_.c_str(), seg_det_topic_.c_str(), input_w_, input_h_);
    }

private:
    struct ScaleInfo
    {
        bool enabled{false};
        double sx{1.0};
        double sy{1.0};
    };

    static inline int clampi(int v, int lo, int hi)
    {
        return std::max(lo, std::min(v, hi));
    }

    ScaleInfo computeScale(const cv::Mat& img) const
    {
        ScaleInfo s;
        if (!scale_output_) return s;
        if (img.cols <= 0 || img.rows <= 0) return s;

        s.enabled = true;
        s.sx = static_cast<double>(output_width_)  / static_cast<double>(img.cols);
        s.sy = static_cast<double>(output_height_) / static_cast<double>(img.rows);
        return s;
    }

    vision_msgs::msg::Detection2DArray
    makeDetectionMsg(const std_msgs::msg::Header& header, const std::vector<Det>& dets) const
    {
        vision_msgs::msg::Detection2DArray msg;
        msg.header = header;
        msg.detections.reserve(dets.size());

        for (const auto &d : dets)
        {
            vision_msgs::msg::Detection2D det;
            det.header = header;

            const float cx = 0.5f * (d.x1 + d.x2);
            const float cy = 0.5f * (d.y1 + d.y2);
            const float w  = (d.x2 - d.x1);
            const float h  = (d.y2 - d.y1);

            det.bbox.center.position.x = cx;
            det.bbox.center.position.y = cy;
            det.bbox.size_x = w;
            det.bbox.size_y = h;

            vision_msgs::msg::ObjectHypothesisWithPose hyp;
            hyp.hypothesis.class_id = std::to_string(d.cls);
            hyp.hypothesis.score = d.conf;
            det.results.push_back(hyp);

            msg.detections.push_back(std::move(det));
        }

        return msg;
    }

    vision_msgs::msg::Detection2DArray
    scaleDetectionMsg(const vision_msgs::msg::Detection2DArray& in, double sx, double sy) const
    {
        auto out = in;
        for (auto& d : out.detections)
        {
            d.bbox.center.position.x *= (float)sx;
            d.bbox.center.position.y *= (float)sy;
            d.bbox.size_x *= (float)sx;
            d.bbox.size_y *= (float)sy;
        }
        return out;
    }

    multi_camera_rig_msgs::msg::InstanceSegmentation2DArray
    buildInstanceMsg(const std_msgs::msg::Header& header,
                     const std::vector<Det>& dets,
                     bool want_masks)
    {
        multi_camera_rig_msgs::msg::InstanceSegmentation2DArray msg;
        msg.header = header;
        msg.detections.reserve(dets.size());

        const bool can_mask = want_masks && detector_->isSegmentation();

        const auto &lb = detector_->getLetterboxInfo();
        const auto &ps = detector_->protoShape();
        const auto &proto = detector_->proto();

        const bool proto_ok =
            can_mask && ps.nbDims == 4 && ps.d[1] > 0 && ps.d[2] > 0 && ps.d[3] > 0 && !proto.empty();

        const int proto_c = proto_ok ? ps.d[1] : 0;
        const int proto_h = proto_ok ? ps.d[2] : 0;
        const int proto_w = proto_ok ? ps.d[3] : 0;
        const float* proto_ptr = proto_ok ? proto.data() : nullptr;

        for (const auto& d : dets)
        {
            multi_camera_rig_msgs::msg::InstanceSegmentation2D inst;
            inst.header   = header;
            inst.class_id = std::to_string(d.cls);
            inst.score    = d.conf;

            const float cx = 0.5f * (d.x1 + d.x2);
            const float cy = 0.5f * (d.y1 + d.y2);
            const float w  = (d.x2 - d.x1);
            const float h  = (d.y2 - d.y1);

            inst.bbox.center.position.x = cx;
            inst.bbox.center.position.y = cy;
            inst.bbox.size_x = w;
            inst.bbox.size_y = h;

            if (proto_ok && (int)d.mask_coeffs.size() == proto_c)
            {
                cv::Mat full_mask = multi_camera_rig_detection::buildYoloSegMask(
                    proto_ptr, proto_c, proto_h, proto_w,
                    d.mask_coeffs, lb, d, (float)mask_thresh_);

                if (!full_mask.empty())
                {
                    int x1 = clampi((int)std::floor(d.x1), 0, full_mask.cols - 1);
                    int y1 = clampi((int)std::floor(d.y1), 0, full_mask.rows - 1);
                    int x2 = clampi((int)std::ceil (d.x2), 0, full_mask.cols);
                    int y2 = clampi((int)std::ceil (d.y2), 0, full_mask.rows);

                    if (x2 > x1 && y2 > y1)
                    {
                        // cv::Mat roi = full_mask(cv::Rect(x1, y1, x2 - x1, y2 - y1));
                        cv::Mat roi = full_mask(cv::Rect(x1, y1, x2 - x1, y2 - y1)).clone(); // This makes the data contiguous for memcpy below
                        inst.mask_width  = (uint32_t)roi.cols;
                        inst.mask_height = (uint32_t)roi.rows;
                        inst.mask_data.resize((size_t)roi.cols * (size_t)roi.rows);
                        // RCLCPP_INFO(get_logger(),"roi: %dx%d step_bytes=%zu continuous=%d", roi.cols, roi.rows,
                        //     (size_t)roi.step[0], (int)roi.isContinuous());
                        std::memcpy(inst.mask_data.data(), roi.data, inst.mask_data.size());
                    }
                }
            }

            msg.detections.push_back(std::move(inst));
        }

        return msg;
    }

    multi_camera_rig_msgs::msg::InstanceSegmentation2DArray
    scaleInstanceMsg(const multi_camera_rig_msgs::msg::InstanceSegmentation2DArray& in,
                     double sx, double sy) const
    {
        auto out = in;

        for (auto& det : out.detections)
        {
            det.bbox.center.position.x *= (float)sx;
            det.bbox.center.position.y *= (float)sy;
            det.bbox.size_x *= (float)sx;
            det.bbox.size_y *= (float)sy;

            const bool has_mask =
                det.mask_width > 0 && det.mask_height > 0 &&
                det.mask_data.size() == (size_t)det.mask_width * (size_t)det.mask_height;

            if (has_mask)
            {
                cv::Mat m((int)det.mask_height, (int)det.mask_width, CV_8UC1,
                          (void*)det.mask_data.data());

                // Resize ROI mask by scale factors (more stable than using bbox.size)
                int new_w = std::max(1, (int)std::round((double)det.mask_width  * sx));
                int new_h = std::max(1, (int)std::round((double)det.mask_height * sy));

                cv::Mat m2;
                cv::resize(m, m2, cv::Size(new_w, new_h), 0, 0, cv::INTER_NEAREST);

                det.mask_width  = (uint32_t)m2.cols;
                det.mask_height = (uint32_t)m2.rows;
                det.mask_data.resize((size_t)m2.cols * (size_t)m2.rows);
                std::memcpy(det.mask_data.data(), m2.data, det.mask_data.size());
            }
        }

        return out;
    }

    void publishDebugImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                           const cv::Mat &bgr,
                           const std::vector<Det> &dets)
    {
        if (!debug_ || !debug_img_pub_)
            return;

        cv::Mat vis = bgr.clone();

        // Overlay masks first so boxes/text are visible on top
        if (debug_masks_ && detector_->isSegmentation())
        {
            const auto &lb = detector_->getLetterboxInfo();
            const auto &ps = detector_->protoShape();
            const auto &proto = detector_->proto();

            if (ps.nbDims == 4 && ps.d[1] > 0 && ps.d[2] > 0 && ps.d[3] > 0 && !proto.empty())
            {
                const int proto_c = ps.d[1];
                const int proto_h = ps.d[2];
                const int proto_w = ps.d[3];
                const float *proto_ptr = proto.data();

                for (const auto &d : dets)
                {
                    if ((int)d.mask_coeffs.size() != proto_c)
                        continue;

                    const cv::Scalar col = multi_camera_rig_detection::classColor(d.cls);
                    cv::Mat mask = multi_camera_rig_detection::buildYoloSegMask(
                        proto_ptr, proto_c, proto_h, proto_w,
                        d.mask_coeffs, lb, d,
                        (float)mask_thresh_);

                    multi_camera_rig_detection::overlayMask(vis, mask, col, (float)mask_alpha_);
                }
            }
        }

        for (const auto &d : dets)
        {
            const cv::Scalar col = multi_camera_rig_detection::classColor(d.cls);

            cv::rectangle(vis,
                          cv::Point((int)std::round(d.x1), (int)std::round(d.y1)),
                          cv::Point((int)std::round(d.x2), (int)std::round(d.y2)),
                          col, 2);

            std::ostringstream oss;
            oss << "cls:" << d.cls << " " << std::fixed << std::setprecision(2) << d.conf;

            multi_camera_rig_detection::drawLabel(
                vis,
                (int)std::round(d.x1),
                (int)std::round(d.y1),
                oss.str(),
                col);
        }

        cv_bridge::CvImage cv_out(msg->header, "bgr8", vis);
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

        // Detect
        std::vector<Det> dets;
        detector_->detect(cvp->image, dets);

        const auto t1 = now();

        const auto s = computeScale(cvp->image);

        // ---- Publish bbox detections ----
        {
            auto det_msg = makeDetectionMsg(msg->header, dets);
            pub_->publish(det_msg);

            if (s.enabled && pub_scaled_)
            {
                auto det_scaled = scaleDetectionMsg(det_msg, s.sx, s.sy);
                pub_scaled_->publish(det_scaled);
            }
        }

        // ---- Publish instance detections (bbox + optional masks) ----
        {
            auto inst_msg = buildInstanceMsg(msg->header, dets, publish_masks_);
            pub_seg_->publish(inst_msg);

            if (s.enabled && pub_seg_scaled_)
            {
                auto inst_scaled = scaleInstanceMsg(inst_msg, s.sx, s.sy);
                pub_seg_scaled_->publish(inst_scaled);
            }
        }

        // Debug image with optional masks
        publishDebugImage(msg, cvp->image, dets);

        const auto t2 = now();
        const double detect_ms  = (t1 - t0).seconds() * 1e3;
        const double publish_ms = (t2 - t1).seconds() * 1e3;

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "detect %.1f ms | publish %.1f ms | dets %zu | seg=%s | scaled=%s",
                             detect_ms, publish_ms, dets.size(),
                             detector_->isSegmentation() ? "true" : "false",
                             s.enabled ? "true" : "false");
    }

    // Params
    std::string engine_path_;
    std::string image_topic_;

    std::string det_topic_;
    std::string seg_det_topic_;
    bool publish_masks_{true};

    std::string input_tensor_;
    std::string output_tensor_;
    std::string proto_tensor_;
    std::string task_;

    int input_w_{1440};
    int input_h_{1088};
    int stride_{32};
    bool scaleup_{true};

    double conf_thresh_{0.25};
    double iou_thresh_{0.45};
    int max_det_{300};

    bool debug_{false};
    std::string debug_image_topic_{"/yolo/debug_image"};

    bool debug_masks_{true};
    double mask_alpha_{0.45};
    double mask_thresh_{0.50};

    bool scale_output_{false};
    int output_width_{896};
    int output_height_{672};
    std::string det_topic_scaled_{"/yolo/detections_scaled"};
    std::string seg_det_topic_scaled_{"/yolo/instance_segmentation_scaled"};

    // Detector
    std::unique_ptr<YoloDetector> detector_;

    // QoS params
    std::string sub_rel_, sub_dur_, sub_hist_;
    int sub_depth_{5};
    std::string pub_rel_, pub_dur_, pub_hist_;
    int pub_depth_{5};

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_scaled_;
    rclcpp::Publisher<multi_camera_rig_msgs::msg::InstanceSegmentation2DArray>::SharedPtr pub_seg_;
    rclcpp::Publisher<multi_camera_rig_msgs::msg::InstanceSegmentation2DArray>::SharedPtr pub_seg_scaled_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YoloTrtNode>());
    rclcpp::shutdown();
    return 0;
}
