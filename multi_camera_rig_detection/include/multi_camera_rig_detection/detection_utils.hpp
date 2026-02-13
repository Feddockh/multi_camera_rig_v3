#ifndef MULTI_CAMERA_RIG_DETECTION_DETECTION_UTILS_HPP
#define MULTI_CAMERA_RIG_DETECTION_DETECTION_UTILS_HPP

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <vector>
#include <algorithm>
#include <unordered_map>

namespace multi_camera_rig_detection
{

/**
 * @brief Detection result
 */
struct Det
{
    int cls{-1};
    float conf{0.f};
    float x1{0.f}, y1{0.f}, x2{0.f}, y2{0.f};
};

/**
 * @brief Letterbox transformation information
 */
struct LetterboxInfo
{
    float scale{1.f};
    int pad_x{0}; // left pad
    int pad_y{0}; // top pad
    int in_w{0}, in_h{0};
    int out_w{0}, out_h{0};
};

/**
 * @brief Simple tensor dimensions (batch, channels, spatial)
 */
struct TensorDims
{
    int nbDims{0};
    int d[8]{0};
};

/**
 * @brief Apply letterbox padding to image (Ultralytics-like)
 * @param bgr Input BGR image
 * @param out_w Target width
 * @param out_h Target height
 * @param stride Stride for padding alignment (default: 32)
 * @param scaleup Whether to scale up small images (default: true)
 * @param pad_color Padding color (default: gray 114)
 * @param info Optional output letterbox info
 * @return Letterboxed image
 */
inline cv::Mat letterbox(const cv::Mat &bgr,
                         int out_w, int out_h,
                         int /*stride*/ = 32,
                         bool scaleup = true,
                         const cv::Scalar &pad_color = cv::Scalar(114, 114, 114),
                         LetterboxInfo *info = nullptr)
{
    const int in_w = bgr.cols;
    const int in_h = bgr.rows;

    float r = std::min((float)out_w / (float)in_w, (float)out_h / (float)in_h);
    if (!scaleup)
        r = std::min(r, 1.0f);

    int new_w = (int)std::round(in_w * r);
    int new_h = (int)std::round(in_h * r);

    int dw = out_w - new_w;
    int dh = out_h - new_h;

    int pad_left = dw / 2;
    int pad_right = dw - pad_left;
    int pad_top = dh / 2;
    int pad_bottom = dh - pad_top;

    cv::Mat resized;
    if (new_w != in_w || new_h != in_h)
        cv::resize(bgr, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
    else
        resized = bgr;

    cv::Mat out;
    cv::copyMakeBorder(resized, out, pad_top, pad_bottom, pad_left, pad_right,
                       cv::BORDER_CONSTANT, pad_color);

    if (info)
    {
        info->scale = r;
        info->pad_x = pad_left;
        info->pad_y = pad_top;
        info->in_w = in_w;
        info->in_h = in_h;
        info->out_w = out_w;
        info->out_h = out_h;
    }
    return out;
}

/**
 * @brief Convert BGR image to RGB NCHW float format [0,1]
 * @param bgr Input BGR image
 * @param out_nchw Output NCHW float array
 */
inline void bgrToRGBNCHW01(const cv::Mat &bgr, std::vector<float> &out_nchw)
{
    const int H = bgr.rows;
    const int W = bgr.cols;
    const int HW = H * W;
    out_nchw.resize(3 * HW);

    const uint8_t *p = bgr.ptr<uint8_t>(0);
    float *outR = out_nchw.data() + 0 * HW;
    float *outG = out_nchw.data() + 1 * HW;
    float *outB = out_nchw.data() + 2 * HW;

    for (int i = 0; i < HW; ++i)
    {
        const uint8_t b = p[3 * i + 0];
        const uint8_t g = p[3 * i + 1];
        const uint8_t r = p[3 * i + 2];
        outR[i] = (float)r / 255.0f;
        outG[i] = (float)g / 255.0f;
        outB[i] = (float)b / 255.0f;
    }
}

/**
 * @brief Parse YOLO output tensor (1, 4+nc, N) format
 * @param out Output tensor data
 * @param out_dims Output tensor dimensions
 * @param lb Letterbox transformation info
 * @param conf_thresh Confidence threshold
 * @param iou_thresh IoU threshold for NMS
 * @param max_det Maximum detections to return
 * @param dets Output detections
 */
inline void parseYoloCxCyWhClassScores(const float *out,
                                       const TensorDims &shape,
                                       const LetterboxInfo &lb,
                                       float conf_thresh,
                                       float iou_thresh,
                                       int max_det,
                                       std::vector<Det> &dets)
{
    dets.clear();

    if (shape.nbDims != 3 || shape.d[0] != 1 || shape.d[1] < 6)
        throw std::runtime_error("Expected output dims (1,4+nc,N) with nc>=2");

    const int C = (int)shape.d[1]; // 4 + nc
    const int N = (int)shape.d[2];
    const int nc = C - 4;

    std::vector<cv::Rect2f> boxes;
    std::vector<float> scores;
    std::vector<int> class_ids;
    boxes.reserve(N);
    scores.reserve(N);
    class_ids.reserve(N);

    auto get = [&](int chan, int i) -> float
    {
        return out[(size_t)chan * (size_t)N + (size_t)i];
    };

    for (int i = 0; i < N; ++i)
    {
        const float cx = get(0, i);
        const float cy = get(1, i);
        const float w = get(2, i);
        const float h = get(3, i);

        int best_cls = -1;
        float best_score = -1.0f;
        for (int c = 0; c < nc; ++c)
        {
            const float score = get(4 + c, i);
            if (score > best_score)
            {
                best_score = score;
                best_cls = c;
            }
        }

        const float conf = best_score;
        if (best_cls < 0 || conf < conf_thresh)
            continue;

        // cxcywh -> xyxy (letterboxed coords)
        float x1 = cx - 0.5f * w;
        float y1 = cy - 0.5f * h;
        float x2 = cx + 0.5f * w;
        float y2 = cy + 0.5f * h;

        // Undo letterbox
        x1 = (x1 - (float)lb.pad_x) / lb.scale;
        y1 = (y1 - (float)lb.pad_y) / lb.scale;
        x2 = (x2 - (float)lb.pad_x) / lb.scale;
        y2 = (y2 - (float)lb.pad_y) / lb.scale;

        // Clamp + order
        x1 = std::max(0.f, std::min(x1, (float)lb.in_w - 1));
        y1 = std::max(0.f, std::min(y1, (float)lb.in_h - 1));
        x2 = std::max(0.f, std::min(x2, (float)lb.in_w - 1));
        y2 = std::max(0.f, std::min(y2, (float)lb.in_h - 1));
        if (x2 < x1)
            std::swap(x1, x2);
        if (y2 < y1)
            std::swap(y1, y2);

        const float bw = x2 - x1;
        const float bh = y2 - y1;
        if (bw < 1.f || bh < 1.f)
            continue;

        boxes.emplace_back(x1, y1, bw, bh);
        scores.emplace_back(conf);
        class_ids.emplace_back(best_cls);
    }

    // NMS per class
    std::unordered_map<int, std::vector<int>> by_class;
    for (int i = 0; i < (int)class_ids.size(); ++i)
        by_class[class_ids[i]].push_back(i);

    std::vector<Det> out_dets;
    out_dets.reserve((size_t)max_det);

    for (auto &kv : by_class)
    {
        const int cls = kv.first;
        const auto &inds = kv.second;

        std::vector<cv::Rect2d> cls_boxes;
        std::vector<float> cls_scores;
        cls_boxes.reserve(inds.size());
        cls_scores.reserve(inds.size());

        for (int idx : inds)
        {
            const auto &b = boxes[idx];
            cls_boxes.push_back(cv::Rect2d(b.x, b.y, b.width, b.height));
            cls_scores.push_back(scores[idx]);
        }

        std::vector<int> keep;
        cv::dnn::NMSBoxes(cls_boxes, cls_scores, conf_thresh, iou_thresh, keep);

        for (int idx : keep)
        {
            Det d;
            d.cls = cls;
            d.conf = cls_scores[idx];
            const auto &box = cls_boxes[idx];
            d.x1 = box.x;
            d.y1 = box.y;
            d.x2 = box.x + box.width;
            d.y2 = box.y + box.height;
            out_dets.push_back(d);
        }
    }

    std::sort(out_dets.begin(), out_dets.end(),
              [](const Det &a, const Det &b)
              { return a.conf > b.conf; });

    if ((int)out_dets.size() > max_det)
        out_dets.resize(max_det);

    dets = std::move(out_dets);
}

} // namespace multi_camera_rig_detection

#endif // MULTI_CAMERA_RIG_DETECTION_DETECTION_UTILS_HPP
