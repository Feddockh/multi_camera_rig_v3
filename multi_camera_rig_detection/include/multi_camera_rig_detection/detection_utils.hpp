#ifndef MULTI_CAMERA_RIG_DETECTION_DETECTION_UTILS_HPP
#define MULTI_CAMERA_RIG_DETECTION_DETECTION_UTILS_HPP

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn/dnn.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

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

    // Optional: YOLO-seg mask coefficients (usually 32)
    std::vector<float> mask_coeffs;
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

    // resized (unpadded) dimensions inside the letterboxed image
    int new_w{0}, new_h{0};
};

/**
 * @brief Simple tensor dimensions (up to 8 dims)
 */
struct TensorDims
{
    int nbDims{0};
    int d[8]{0};
};

/**
 * @brief Apply letterbox padding to image (Ultralytics-like)
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
        info->new_w = new_w;
        info->new_h = new_h;
    }
    return out;
}

/**
 * @brief Convert BGR image to RGB NCHW float format [0,1]
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
 * @brief Parse YOLO output tensor (1, 4+nc, N) format with cxcywh + class scores
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
        if (x2 < x1) std::swap(x1, x2);
        if (y2 < y1) std::swap(y1, y2);

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

        for (int k : keep)
        {
            Det d;
            d.cls = cls;
            d.conf = cls_scores[k];
            const auto &box = cls_boxes[k];
            d.x1 = (float)box.x;
            d.y1 = (float)box.y;
            d.x2 = (float)(box.x + box.width);
            d.y2 = (float)(box.y + box.height);
            out_dets.push_back(d);
        }
    }

    std::sort(out_dets.begin(), out_dets.end(),
              [](const Det &a, const Det &b) { return a.conf > b.conf; });

    if ((int)out_dets.size() > max_det)
        out_dets.resize(max_det);

    dets = std::move(out_dets);
}

/**
 * @brief Parse End2End / transposed output format: (1, N, C) with columns:
 *   [x1, y1, x2, y2, score, cls, (optional mask coeffs...)]
 *
 * Common for many YOLO-seg TensorRT exports: e.g. (1,300,38) where 38=6+32.
 */
inline void parseYoloEnd2EndTransposed(const float* out,
                                      const TensorDims& shape,
                                      const LetterboxInfo& lb,
                                      float conf_thresh,
                                      int max_det,
                                      int num_mask_coeffs,
                                      std::vector<Det>& dets)
{
    dets.clear();

    if (shape.nbDims != 3 || shape.d[0] != 1)
        throw std::runtime_error("Expected output dims (1,N,C)");

    const int N = shape.d[1];
    const int C = shape.d[2];

    if (C < 6)
        throw std::runtime_error("End2End transposed requires C >= 6");

    const int expected = 6 + std::max(0, num_mask_coeffs);
    if (C < expected)
        throw std::runtime_error("End2End transposed: C too small for requested mask coeffs");

    auto get = [&](int i, int j) -> float {
        return out[(size_t)i * (size_t)C + (size_t)j];
    };

    dets.reserve(std::min(N, max_det));

    for (int i = 0; i < N; ++i)
    {
        float x1 = get(i, 0);
        float y1 = get(i, 1);
        float x2 = get(i, 2);
        float y2 = get(i, 3);
        const float score = get(i, 4);
        const int cls = (int)std::llround((double)get(i, 5)); // often exported as float

        if (score < conf_thresh)
            continue;

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
        if (x2 < x1) std::swap(x1, x2);
        if (y2 < y1) std::swap(y1, y2);

        if ((x2 - x1) < 1.f || (y2 - y1) < 1.f)
            continue;

        Det d;
        d.cls = cls;
        d.conf = score;
        d.x1 = x1; d.y1 = y1; d.x2 = x2; d.y2 = y2;

        if (num_mask_coeffs > 0)
        {
            d.mask_coeffs.resize(num_mask_coeffs);
            for (int k = 0; k < num_mask_coeffs; ++k)
                d.mask_coeffs[k] = get(i, 6 + k);
        }

        dets.push_back(std::move(d));
        if ((int)dets.size() >= max_det)
            break;
    }
}

/**
 * @brief Simple deterministic per-class BGR color
 */
inline cv::Scalar classColor(int cls)
{
    // BGR palette
    // static const cv::Scalar palette[] = {
    //     {0, 255, 0},    // green
    //     {0, 0, 255},    // red
    //     {255, 0, 0},    // blue
    //     {0, 255, 255},  // yellow
    //     {255, 0, 255},  // magenta
    //     {255, 255, 0},  // cyan
    //     {128, 255, 0},
    //     {0, 128, 255},
    // };
    static const cv::Scalar palette[] = {
        {255, 150, 0},    // Blue - class 0 (BGR)
        {75, 25, 230},    // Red - class 1 (BGR)
        {75, 180, 60},    // Green - class 2 (BGR)
        {25, 225, 255},   // Yellow - class 3 (BGR)
        {48, 130, 245},   // Orange - class 4 (BGR)
        {180, 30, 145},   // Purple - class 5 (BGR)
        {240, 240, 70},   // Cyan - class 6 (BGR)
        {230, 50, 240},   // Magenta - class 7 (BGR)
        {60, 245, 210},   // Lime - class 8 (BGR)
        {212, 190, 250},  // Pink - class 9 (BGR)
        {128, 128, 0},    // Teal - class 10 (BGR)
        {255, 190, 220},  // Lavender - class 11 (BGR)
        {40, 110, 170},   // Brown - class 12 (BGR)
        {200, 250, 255},  // Beige - class 13 (BGR)
        {0, 0, 128},      // Maroon - class 14 (BGR)
        {195, 255, 170},  // Mint - class 15 (BGR)
        {0, 128, 128},    // Olive - class 16 (BGR)
        {180, 215, 255},  // Coral - class 17 (BGR)
        {128, 0, 0},      // Navy - class 18 (BGR)
        {128, 128, 128}   // Grey - class 19 (BGR)
    };
    const int n = (int)(sizeof(palette) / sizeof(palette[0]));
    const int idx = (cls >= 0 ? cls : 0) % n;
    return palette[idx];
}

/**
 * @brief Draw readable label: background patch + black outline + white text
 */
inline void drawLabel(cv::Mat& img, int x, int y, const std::string& text, const cv::Scalar& col)
{
    x = std::max(0, std::min(x, img.cols - 1));
    y = std::max(0, std::min(y, img.rows - 1));

    int baseline = 0;
    const double font_scale = 0.65;
    const int thickness = 2;

    cv::Size ts = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);

    const int pad = 4;
    int x0 = x;
    int y0 = y - ts.height - baseline - pad * 2;
    if (y0 < 0) y0 = y;

    int x1 = std::min(img.cols - 1, x0 + ts.width + pad * 2);
    int y1 = std::min(img.rows - 1, y0 + ts.height + baseline + pad * 2);

    // dark background tinted by class color
    cv::Scalar bg(col[0] * 0.20, col[1] * 0.20, col[2] * 0.20);
    cv::rectangle(img, cv::Rect(cv::Point(x0, y0), cv::Point(x1, y1)), bg, cv::FILLED);

    int tx = x0 + pad;
    int ty = y0 + pad + ts.height;

    cv::putText(img, text, {tx, ty}, cv::FONT_HERSHEY_SIMPLEX, font_scale,
                cv::Scalar(0, 0, 0), thickness + 2, cv::LINE_AA);
    cv::putText(img, text, {tx, ty}, cv::FONT_HERSHEY_SIMPLEX, font_scale,
                cv::Scalar(255, 255, 255), thickness, cv::LINE_AA);
}

/**
 * @brief Build instance mask from proto + coefficients (YOLO-seg style)
 *
 * proto assumed shape (1, C, H, W) contiguous in CHW order per channel.
 */
inline cv::Mat buildYoloSegMask(const float* proto,
                               int proto_c, int proto_h, int proto_w,
                               const std::vector<float>& coeffs,
                               const LetterboxInfo& lb,
                               const Det& d,
                               float mask_thresh = 0.5f)
{
    const int HW = proto_h * proto_w;
    if ((int)coeffs.size() != proto_c)
        return cv::Mat(); // empty

    // logits = sum_k coeff[k] * proto[k,:,:]
    cv::Mat logits(proto_h, proto_w, CV_32F, cv::Scalar(0));
    float* L = (float*)logits.data;

    for (int k = 0; k < proto_c; ++k)
    {
        const float a = coeffs[(size_t)k];
        const float* Pk = proto + (size_t)k * (size_t)HW;
        for (int i = 0; i < HW; ++i)
            L[i] += a * Pk[i];
    }

    // sigmoid
    cv::Mat prob;
    cv::exp(-logits, prob);
    prob = 1.0 / (1.0 + prob);

    // Resize to letterboxed input size (out_w/out_h)
    cv::Mat prob_up;
    cv::resize(prob, prob_up, cv::Size(lb.out_w, lb.out_h), 0, 0, cv::INTER_LINEAR);

    // Crop out padding to resized region (new_w/new_h)
    cv::Rect roi(lb.pad_x, lb.pad_y, lb.new_w, lb.new_h);
    roi &= cv::Rect(0, 0, prob_up.cols, prob_up.rows);
    if (roi.width <= 0 || roi.height <= 0)
        return cv::Mat();

    cv::Mat prob_unpad = prob_up(roi);

    // Resize to original image size (in_w/in_h)
    cv::Mat prob_img;
    cv::resize(prob_unpad, prob_img, cv::Size(lb.in_w, lb.in_h), 0, 0, cv::INTER_LINEAR);

    // Threshold and (optionally) restrict to bbox for cleanliness
    cv::Mat mask(lb.in_h, lb.in_w, CV_8U, cv::Scalar(0));

    int x1 = (int)std::floor(d.x1), y1 = (int)std::floor(d.y1);
    int x2 = (int)std::ceil(d.x2),  y2 = (int)std::ceil(d.y2);
    x1 = std::max(0, std::min(x1, lb.in_w - 1));
    y1 = std::max(0, std::min(y1, lb.in_h - 1));
    x2 = std::max(0, std::min(x2, lb.in_w));
    y2 = std::max(0, std::min(y2, lb.in_h));

    if (x2 > x1 && y2 > y1)
    {
        cv::Mat proi = prob_img(cv::Rect(x1, y1, x2 - x1, y2 - y1));
        cv::Mat mroi = mask(cv::Rect(x1, y1, x2 - x1, y2 - y1));

        cv::Mat bin;
        cv::threshold(proi, bin, mask_thresh, 255.0, cv::THRESH_BINARY);
        bin.convertTo(mroi, CV_8U);
    }
    return mask;
}

/**
 * @brief Alpha overlay a binary mask on a BGR image using a class color.
 */
inline void overlayMask(cv::Mat& bgr, const cv::Mat& mask_u8, const cv::Scalar& col, float alpha = 0.45f)
{
    if (mask_u8.empty())
        return;

    CV_Assert(bgr.type() == CV_8UC3);
    CV_Assert(mask_u8.type() == CV_8U);
    CV_Assert(bgr.rows == mask_u8.rows && bgr.cols == mask_u8.cols);

    alpha = std::max(0.f, std::min(alpha, 1.f));

    for (int y = 0; y < bgr.rows; ++y)
    {
        const uint8_t* m = mask_u8.ptr<uint8_t>(y);
        cv::Vec3b* p = bgr.ptr<cv::Vec3b>(y);
        for (int x = 0; x < bgr.cols; ++x)
        {
            if (m[x])
            {
                p[x][0] = (uint8_t)((1.f - alpha) * p[x][0] + alpha * (float)col[0]);
                p[x][1] = (uint8_t)((1.f - alpha) * p[x][1] + alpha * (float)col[1]);
                p[x][2] = (uint8_t)((1.f - alpha) * p[x][2] + alpha * (float)col[2]);
            }
        }
    }
}

} // namespace multi_camera_rig_detection

#endif // MULTI_CAMERA_RIG_DETECTION_DETECTION_UTILS_HPP
