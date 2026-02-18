/*
Standalone TensorRT YOLO tester (no ROS required)
Supports:
  - YOLO detection-style outputs: (1, C, N) e.g. (1,6,32130)
  - End2End / post-NMS outputs:   (1, N, C) e.g. (1,300,38)
  - Segmentation models with proto output: output1 = (1,32,Hp,Wp)

Build (as part of ROS2 workspace):
  This file will be built as part of the multi_camera_rig_detection package.
  It links against multi_camera_rig_common for TrtRunner.

Run:
  ros2 run multi_camera_rig_detection test_yolov8_trt \
    /path/to/model.plan /path/to/image.jpg \
    [conf_thresh] [iou_thresh] [max_det] [debug] [mask_thresh] [mask_alpha] [crop_masks]

Examples:
  Detection:
    ros2 run multi_camera_rig_detection test_yolov8_trt best_sim.plan img.png 0.3 0.45 100 1

  Segmentation (overlay masks):
    ros2 run multi_camera_rig_detection test_yolov8_trt best_sim_seg.plan img.png 0.3 0.45 100 1 0.5 0.45 1
*/

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/dnn/dnn.hpp>

#include "multi_camera_rig_common/trt_runner.hpp"
#include <NvInfer.h>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
    // -------------------------
    // Utility helpers
    // -------------------------
    std::string dimsToString(const nvinfer1::Dims &d)
    {
        std::string s = "(";
        for (int i = 0; i < d.nbDims; ++i)
        {
            if (i)
                s += ",";
            s += std::to_string(d.d[i]);
        }
        s += ")";
        return s;
    }

    size_t volume(const nvinfer1::Dims &d)
    {
        size_t v = 1;
        for (int i = 0; i < d.nbDims; ++i)
            v *= static_cast<size_t>(d.d[i]);
        return v;
    }

    void printVecStats(const std::string &tag, const float *x, size_t n, size_t sample_stride = 1)
    {
        double sum = 0.0, sum2 = 0.0;
        float mn = std::numeric_limits<float>::infinity();
        float mx = -std::numeric_limits<float>::infinity();
        size_t nnan = 0, ninf = 0, nzero = 0, nfinite = 0;

        for (size_t i = 0; i < n; ++i)
        {
            const float v = x[i];
            if (std::isnan(v))
            {
                nnan++;
                continue;
            }
            if (!std::isfinite(v))
            {
                ninf++;
                continue;
            }
            nfinite++;
            if (std::abs(v) < 1e-12f)
                nzero++;
            mn = std::min(mn, v);
            mx = std::max(mx, v);
            sum += (double)v;
            sum2 += (double)v * (double)v;
        }

        const double mean = (nfinite ? (sum / (double)nfinite) : 0.0);
        const double var = (nfinite ? (sum2 / (double)nfinite - mean * mean) : 0.0);
        const double stddev = (var > 0.0) ? std::sqrt(var) : 0.0;

        std::cout << std::fixed << std::setprecision(6);
        std::cout << "[DBG] " << tag
                  << " n=" << n
                  << " min=" << (std::isfinite(mn) ? mn : 0.f)
                  << " max=" << (std::isfinite(mx) ? mx : 0.f)
                  << " mean=" << mean
                  << " std=" << stddev
                  << " zeros=" << nzero
                  << " nan=" << nnan
                  << " inf=" << ninf
                  << "\n";

        std::cout << "[DBG] " << tag << " samples: ";
        size_t printed = 0;
        for (size_t i = 0; i < n && printed < 12; i += sample_stride)
        {
            std::cout << x[i] << " ";
            printed++;
        }
        std::cout << "\n";
    }

    static void drawLabel(cv::Mat& img,
                      int x, int y,
                      const std::string& text,
                      const cv::Scalar& box_color,
                      double font_scale = 0.6,
                      int thickness = 2)
    {
        // Clamp anchor
        x = std::max(0, std::min(x, img.cols - 1));
        y = std::max(0, std::min(y, img.rows - 1));

        int baseline = 0;
        cv::Size ts = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);

        const int pad = 3;
        int x0 = x;
        int y0 = y - ts.height - baseline - pad * 2; // top of bg
        if (y0 < 0) y0 = y;                          // if above image, put below anchor
        int x1 = std::min(img.cols - 1, x0 + ts.width + pad * 2);
        int y1 = std::min(img.rows - 1, y0 + ts.height + baseline + pad * 2);

        // Background: dark (or box_color darkened)
        cv::Scalar bg = box_color * 0.25; // darken class color
        cv::rectangle(img, cv::Rect(cv::Point(x0, y0), cv::Point(x1, y1)), bg, cv::FILLED);

        // Text position inside bg
        int tx = x0 + pad;
        int ty = y0 + pad + ts.height;

        // Outline (black) + fill (white)
        cv::putText(img, text, cv::Point(tx, ty),
                    cv::FONT_HERSHEY_SIMPLEX, font_scale,
                    cv::Scalar(0, 0, 0), thickness + 2, cv::LINE_AA);
        cv::putText(img, text, cv::Point(tx, ty),
                    cv::FONT_HERSHEY_SIMPLEX, font_scale,
                    cv::Scalar(255, 255, 255), thickness, cv::LINE_AA);
    }

    // -------------------------
    // Letterbox + input convert
    // -------------------------
    struct LetterboxInfo
    {
        float scale{1.f};
        int pad_x{0};
        int pad_y{0};
        int in_w{0}, in_h{0};
        int out_w{0}, out_h{0};
        int new_w{0}, new_h{0};
    };

    cv::Mat letterbox(const cv::Mat &bgr,
                      int out_w, int out_h,
                      bool scaleup,
                      const cv::Scalar &pad_color,
                      LetterboxInfo *info)
    {
        const int in_w = bgr.cols;
        const int in_h = bgr.rows;

        float r = std::min((float)out_w / (float)in_w, (float)out_h / (float)in_h);
        if (!scaleup)
            r = std::min(r, 1.0f);

        const int new_w = (int)std::round(in_w * r);
        const int new_h = (int)std::round(in_h * r);

        const int dw = out_w - new_w;
        const int dh = out_h - new_h;

        const int pad_left = dw / 2;
        const int pad_right = dw - pad_left;
        const int pad_top = dh / 2;
        const int pad_bottom = dh - pad_top;

        cv::Mat resized;
        if (new_w != in_w || new_h != in_h)
        {
            cv::resize(bgr, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
        }
        else
        {
            resized = bgr;
        }

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

    void bgrToRGBNCHW01(const cv::Mat &bgr, std::vector<float> &out_nchw)
    {
        const int H = bgr.rows;
        const int W = bgr.cols;
        const int HW = H * W;
        out_nchw.resize((size_t)3 * (size_t)HW);

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

    // -------------------------
    // Detection representation
    // -------------------------
    struct Det
    {
        int cls{-1};
        float conf{0.f};
        float x1{0.f}, y1{0.f}, x2{0.f}, y2{0.f};
        std::vector<float> mask_coeffs; // 32 values for segmentation models
    };

    // -------------------------
    // Debug output
    // -------------------------
    void debugOutput(const std::vector<float> &output, const nvinfer1::Dims &out_dims, float conf_thresh)
    {
        printVecStats("output(flat)", output.data(), output.size(), std::max<size_t>(1, output.size() / 12));

        if (out_dims.nbDims != 3 || out_dims.d[0] != 1)
            return;

        const int d1 = (int)out_dims.d[1];
        const int d2 = (int)out_dims.d[2];

        // If it looks like (1,N,C) where N small and C small
        if (d1 <= 1000 && d2 < 256)
        {
            std::cout << "[DBG] Output looks like (1,N,C) with N=" << d1 << " C=" << d2 << "\n";
            std::cout << "[DBG] First row attrs: ";
            for (int j = 0; j < std::min(d2, 12); ++j)
                std::cout << output[j] << " ";
            std::cout << "\n";
            return;
        }

        // Else assume (1,C,N)
        const int C = d1;
        const int N = d2;
        std::cout << "[DBG] Output looks like (1,C,N) with C=" << C << " N=" << N << "\n";
        for (int c = 0; c < std::min(C, 8); ++c)
        {
            const float *chan = output.data() + (size_t)c * (size_t)N;
            printVecStats("  chan_" + std::to_string(c), chan, N, std::max(1, N / 10));
        }

        // how many pass threshold for "best score" (assumes classes start at 4)
        int pass = 0;
        for (int i = 0; i < N; ++i)
        {
            float best = -1.0f;
            for (int c = 0; c < (C - 4); ++c)
            {
                float score = output[(4 + c) * (size_t)N + (size_t)i];
                if (score > best)
                    best = score;
            }
            if (best >= conf_thresh)
                pass++;
        }
        std::cout << "[DBG] candidates with max_class_score>=thresh: " << pass << "/" << N
                  << " (thresh=" << conf_thresh << ")\n";
    }

    void debugProtoOutput(const std::vector<float> &proto, const nvinfer1::Dims &proto_dims)
    {
        std::cout << "\n[DBG] === Mask Proto Output ===\n";
        printVecStats("proto(flat)", proto.data(), proto.size(), std::max<size_t>(1, proto.size() / 12));

        if (proto_dims.nbDims == 4)
        {
            const int num_protos = (int)proto_dims.d[1];
            const int proto_h = (int)proto_dims.d[2];
            const int proto_w = (int)proto_dims.d[3];
            std::cout << "[DBG] Proto shape: (1, " << num_protos << ", " << proto_h << ", " << proto_w << ")\n";

            // Sample first few proto channels
            const size_t hw = (size_t)proto_h * (size_t)proto_w;
            for (int p = 0; p < std::min(4, num_protos); ++p)
            {
                const float *proto_chan = proto.data() + (size_t)p * hw;
                printVecStats("  proto_" + std::to_string(p), proto_chan, hw, std::max<size_t>(1, hw / 10));
            }
        }
    }

    // -------------------------
    // Parser: (1,C,N) anchor-style YOLO
    // Supports:
    //  - (cx,cy,w,h, class_scores...)           (no objectness)
    //  - (cx,cy,w,h, obj, class_scores...)      (with objectness)
    // -------------------------
    void parseYoloCxCyWhClassScores(const float *out,
                                    const nvinfer1::Dims &out_dims,
                                    const LetterboxInfo &lb,
                                    float conf_thresh,
                                    float iou_thresh,
                                    int max_det,
                                    std::vector<Det> &dets)
    {
        dets.clear();

        if (out_dims.nbDims != 3 || out_dims.d[0] != 1 || out_dims.d[1] < 6)
            throw std::runtime_error("Expected output dims (1,C,N) with C>=6, got " + dimsToString(out_dims));

        const int C = (int)out_dims.d[1];
        const int N = (int)out_dims.d[2];

        // Heuristic: if C == 4 + nc => no obj.
        // If C == 5 + nc => has obj at channel 4.
        // We don't know nc directly, but we can assume "has obj" if C >= 7 (typical).
        // This keeps your current C=6,nc=2 working as no-obj.
        const bool has_obj = (C >= 7);

        const int cls_start = has_obj ? 5 : 4;
        const int nc = C - cls_start;

        std::cout << "[INFO] Parsing (1,C,N): C=" << C << " N=" << N
                  << " has_obj=" << (has_obj ? 1 : 0)
                  << " nc=" << nc << "\n";

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
            const float w  = get(2, i);
            const float h  = get(3, i);

            const float obj = has_obj ? get(4, i) : 1.0f;

            int best_cls = -1;
            float best_score = -1.0f;

            for (int c = 0; c < nc; ++c)
            {
                const float cls_score = get(cls_start + c, i);
                const float score = obj * cls_score;
                if (score > best_score)
                {
                    best_score = score;
                    best_cls = c;
                }
            }

            const float conf = best_score;
            if (conf < conf_thresh || best_cls < 0)
                continue;

            // cxcywh -> xyxy in letterboxed coords
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
            auto &idxs = kv.second;

            std::vector<cv::Rect> b;
            std::vector<float> s;
            b.reserve(idxs.size());
            s.reserve(idxs.size());

            for (int j : idxs)
            {
                b.push_back(cv::Rect((int)boxes[j].x, (int)boxes[j].y,
                                     (int)boxes[j].width, (int)boxes[j].height));
                s.push_back(scores[j]);
            }

            std::vector<int> keep;
            cv::dnn::NMSBoxes(b, s, conf_thresh, iou_thresh, keep);

            for (int k : keep)
            {
                const int orig_idx = idxs[k];
                Det d;
                d.cls  = cls;
                d.conf = scores[orig_idx];
                d.x1   = boxes[orig_idx].x;
                d.y1   = boxes[orig_idx].y;
                d.x2   = boxes[orig_idx].x + boxes[orig_idx].width;
                d.y2   = boxes[orig_idx].y + boxes[orig_idx].height;
                out_dets.push_back(d);
            }
        }

        std::sort(out_dets.begin(), out_dets.end(),
                  [](const Det &a, const Det &b) { return a.conf > b.conf; });

        if ((int)out_dets.size() > max_det)
            out_dets.resize(max_det);

        dets = std::move(out_dets);
    }

    // -------------------------
    // Parser: (1,N,C) transposed / End2End
    // Supports:
    //  - End2End format: [x1,y1,x2,y2,score,cls,(mask...)]
    //  - Fallback scores format: [x1,y1,x2,y2, class_scores..., (mask...)]
    // -------------------------
    void parseYoloTransposed(const float *out,
                             const nvinfer1::Dims &out_dims,
                             const LetterboxInfo &lb,
                             float conf_thresh,
                             int max_det,
                             std::vector<Det> &dets,
                             int num_mask_coeffs = 0)
    {
        dets.clear();

        if (out_dims.nbDims != 3 || out_dims.d[0] != 1)
            throw std::runtime_error("Expected output dims (1,N,C), got " + dimsToString(out_dims));

        const int N = (int)out_dims.d[1];
        const int C = (int)out_dims.d[2];

        auto get = [&](int i, int j) -> float {
            return out[(size_t)i * (size_t)C + (size_t)j];
        };

        // Heuristic: End2End-style if C == 6 (+ masks) OR if column 5 looks like an integer class id
        const bool c_matches_end2end = (C == (6 + num_mask_coeffs)) || (C == 6);
        int looks_int = 0;
        int looks_prob = 0;
        const int sample_n = std::min(N, 50);

        if (C >= 6)
        {
            for (int i = 0; i < sample_n; ++i)
            {
                float cls_f = get(i, 5);
                float score = get(i, 4);
                if (std::isfinite(cls_f))
                {
                    float r = std::round(cls_f);
                    if (std::abs(cls_f - r) < 1e-3f && r >= 0.f && r < 1000.f)
                        looks_int++;
                }
                if (std::isfinite(score) && score >= 0.f && score <= 1.0f)
                    looks_prob++;
            }
        }

        const bool looks_like_end2end =
            c_matches_end2end || (looks_int > sample_n * 0.6f && looks_prob > sample_n * 0.6f);

        if (looks_like_end2end)
        {
            std::cout << "[INFO] Using End2End transposed parser: [xyxy, score, cls, masks...]\n";

            for (int i = 0; i < N; ++i)
            {
                float x1 = get(i, 0);
                float y1 = get(i, 1);
                float x2 = get(i, 2);
                float y2 = get(i, 3);
                float score = get(i, 4);
                int cls = (int)std::round(get(i, 5));

                if (!std::isfinite(score) || score < conf_thresh) continue;
                if (cls < 0) continue;

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

                if ((x2 - x1) < 1.f || (y2 - y1) < 1.f) continue;

                Det d;
                d.cls = cls;
                d.conf = score;
                d.x1 = x1; d.y1 = y1; d.x2 = x2; d.y2 = y2;

                if (num_mask_coeffs > 0 && C >= (6 + num_mask_coeffs))
                {
                    d.mask_coeffs.resize(num_mask_coeffs);
                    for (int k = 0; k < num_mask_coeffs; ++k)
                        d.mask_coeffs[k] = get(i, 6 + k);
                }

                dets.push_back(std::move(d));
            }

            if ((int)dets.size() > max_det) dets.resize(max_det);
            return;
        }

        // Fallback score layout
        const int nc = C - 4 - num_mask_coeffs;
        if (nc <= 0)
            throw std::runtime_error("Transposed fallback: invalid nc=" + std::to_string(nc) + " from C=" + std::to_string(C));

        std::cout << "[INFO] Using class-score transposed parser: [xyxy, scores..., masks...]\n";

        for (int i = 0; i < N; ++i)
        {
            float x1 = get(i, 0);
            float y1 = get(i, 1);
            float x2 = get(i, 2);
            float y2 = get(i, 3);

            int best_cls = -1;
            float best_score = -1.0f;
            for (int c = 0; c < nc; ++c)
            {
                float s = get(i, 4 + c);
                if (s > best_score) { best_score = s; best_cls = c; }
            }

            if (best_score < conf_thresh || best_cls < 0) continue;

            x1 = (x1 - (float)lb.pad_x) / lb.scale;
            y1 = (y1 - (float)lb.pad_y) / lb.scale;
            x2 = (x2 - (float)lb.pad_x) / lb.scale;
            y2 = (y2 - (float)lb.pad_y) / lb.scale;

            x1 = std::max(0.f, std::min(x1, (float)lb.in_w - 1));
            y1 = std::max(0.f, std::min(y1, (float)lb.in_h - 1));
            x2 = std::max(0.f, std::min(x2, (float)lb.in_w - 1));
            y2 = std::max(0.f, std::min(y2, (float)lb.in_h - 1));
            if (x2 < x1) std::swap(x1, x2);
            if (y2 < y1) std::swap(y1, y2);

            if ((x2 - x1) < 1.f || (y2 - y1) < 1.f) continue;

            Det d;
            d.cls = best_cls;
            d.conf = best_score;
            d.x1 = x1; d.y1 = y1; d.x2 = x2; d.y2 = y2;

            if (num_mask_coeffs > 0)
            {
                d.mask_coeffs.resize(num_mask_coeffs);
                for (int k = 0; k < num_mask_coeffs; ++k)
                    d.mask_coeffs[k] = get(i, 4 + nc + k);
            }

            dets.push_back(std::move(d));
        }

        if ((int)dets.size() > max_det) dets.resize(max_det);
    }

    // -------------------------
    // Mask decode + overlay helpers
    // -------------------------
    static inline float sigmoidf(float x)
    {
        return 1.f / (1.f + std::exp(-x));
    }

    // Convert proto tensor (1,32,Hp,Wp) stored as flat CHW into vector<cv::Mat> channels [32] each Hp x Wp.
    static std::vector<cv::Mat> protoToMatsCHW(const float *proto, const nvinfer1::Dims &proto_dims)
    {
        if (proto_dims.nbDims != 4 || proto_dims.d[0] != 1)
            throw std::runtime_error("protoToMatsCHW: expected (1,32,Hp,Wp), got " + dimsToString(proto_dims));

        const int K = (int)proto_dims.d[1];
        const int Hp = (int)proto_dims.d[2];
        const int Wp = (int)proto_dims.d[3];

        std::vector<cv::Mat> chans;
        chans.reserve(K);

        const size_t hw = (size_t)Hp * (size_t)Wp;
        for (int k = 0; k < K; ++k)
        {
            const float *pk = proto + (size_t)k * hw;
            cv::Mat m(Hp, Wp, CV_32F, (void *)pk); // view into proto buffer
            chans.push_back(m);
        }
        return chans;
    }

    // Compose a single instance mask in proto space (Hp x Wp) from coeffs and proto channels.
    // Returns CV_32F mask of probabilities in [0,1] (after sigmoid), same size as proto (Hp x Wp).
    static cv::Mat decodeMaskProtoSpace(const std::vector<cv::Mat> &proto_chans,
                                        const std::vector<float> &coeffs)
    {
        const int K = (int)proto_chans.size();
        if ((int)coeffs.size() != K)
            throw std::runtime_error("decodeMaskProtoSpace: coeffs size mismatch");

        cv::Mat acc = cv::Mat::zeros(proto_chans[0].size(), CV_32F);
        for (int k = 0; k < K; ++k)
            acc += coeffs[k] * proto_chans[k];

        // Sigmoid in-place
        for (int y = 0; y < acc.rows; ++y)
        {
            float *row = acc.ptr<float>(y);
            for (int x = 0; x < acc.cols; ++x)
                row[x] = sigmoidf(row[x]);
        }
        return acc;
    }

    // Map proto-space mask (Hp x Wp) to ORIGINAL image space (in_w x in_h), undoing letterbox.
    // Returns CV_8U binary mask in original image coordinates.
    static cv::Mat maskToOriginalBinary(const cv::Mat &mask_proto_prob,
                                        const LetterboxInfo &lb,
                                        float mask_thresh)
    {
        // A) resize proto prob -> letterboxed network input size (out_w x out_h)
        cv::Mat mask_lb_prob;
        cv::resize(mask_proto_prob, mask_lb_prob, cv::Size(lb.out_w, lb.out_h), 0, 0, cv::INTER_LINEAR);

        // B) crop out padding -> (new_w x new_h)
        cv::Rect roi(lb.pad_x, lb.pad_y, lb.new_w, lb.new_h);
        roi &= cv::Rect(0, 0, mask_lb_prob.cols, mask_lb_prob.rows);
        if (roi.width <= 0 || roi.height <= 0)
            return cv::Mat::zeros(lb.in_h, lb.in_w, CV_8U);

        cv::Mat mask_no_pad = mask_lb_prob(roi);

        // C) resize to original image (in_w x in_h)
        cv::Mat mask_orig_prob;
        cv::resize(mask_no_pad, mask_orig_prob, cv::Size(lb.in_w, lb.in_h), 0, 0, cv::INTER_LINEAR);

        // D) threshold -> binary
        cv::Mat mask_bin;
        cv::threshold(mask_orig_prob, mask_bin, mask_thresh, 255.0, cv::THRESH_BINARY);
        mask_bin.convertTo(mask_bin, CV_8U);

        return mask_bin;
    }

    static void cropMaskToBox(cv::Mat &mask_bin, const Det &d)
    {
        if (mask_bin.empty())
            return;

        cv::Rect box((int)d.x1, (int)d.y1,
                     (int)(d.x2 - d.x1),
                     (int)(d.y2 - d.y1));
        box &= cv::Rect(0, 0, mask_bin.cols, mask_bin.rows);

        cv::Mat keep = cv::Mat::zeros(mask_bin.size(), CV_8U);
        if (box.width > 0 && box.height > 0)
            mask_bin(box).copyTo(keep(box));
        mask_bin = keep;
    }

    static void overlayMask(cv::Mat &bgr,
                            const cv::Mat &mask_bin,
                            const cv::Scalar &color_bgr,
                            float alpha)
    {
        CV_Assert(bgr.type() == CV_8UC3);
        CV_Assert(mask_bin.type() == CV_8U);
        CV_Assert(mask_bin.size() == bgr.size());

        alpha = std::max(0.f, std::min(alpha, 1.f));

        for (int y = 0; y < bgr.rows; ++y)
        {
            const uint8_t *m = mask_bin.ptr<uint8_t>(y);
            cv::Vec3b *p = bgr.ptr<cv::Vec3b>(y);
            for (int x = 0; x < bgr.cols; ++x)
            {
                if (m[x])
                {
                    p[x][0] = (uint8_t)((1.f - alpha) * p[x][0] + alpha * (float)color_bgr[0]);
                    p[x][1] = (uint8_t)((1.f - alpha) * p[x][1] + alpha * (float)color_bgr[1]);
                    p[x][2] = (uint8_t)((1.f - alpha) * p[x][2] + alpha * (float)color_bgr[2]);
                }
            }
        }
    }

} // namespace

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr
            << "Usage: " << argv[0]
            << " <engine.plan> <image>"
            << " [conf_thresh] [iou_thresh] [max_det] [debug]"
            << " [mask_thresh] [mask_alpha] [crop_masks]\n"
            << "\nExamples:\n"
            << "  Detection:\n"
            << "    " << argv[0] << " best_sim.plan img.png 0.3 0.45 100 1\n"
            << "  Segmentation w/ mask overlay:\n"
            << "    " << argv[0] << " best_sim_seg.plan img.png 0.3 0.45 100 1 0.5 0.45 1\n";
        return 1;
    }

    const std::string engine_path = argv[1];
    const std::string image_path = argv[2];

    const float conf_thresh = (argc > 3) ? std::stof(argv[3]) : 0.25f;
    const float iou_thresh  = (argc > 4) ? std::stof(argv[4]) : 0.45f;
    const int   max_det     = (argc > 5) ? std::stoi(argv[5]) : 100;
    const bool  debug       = (argc > 6) ? (std::stoi(argv[6]) != 0) : false;

    const float mask_thresh = (argc > 7) ? std::stof(argv[7]) : 0.5f;
    const float mask_alpha  = (argc > 8) ? std::stof(argv[8]) : 0.45f;
    const bool  crop_masks  = (argc > 9) ? (std::stoi(argv[9]) != 0) : true;

    const std::string input_tensor = "images";
    const std::string output_tensor = "output0";
    const std::string proto_tensor = "output1";

    try
    {
        std::cout << "Loading engine: " << engine_path << "\n";
        auto runner = std::make_unique<multi_camera_rig_common::TrtRunner>(engine_path, true);

        const auto in = runner->shapeOf(input_tensor);
        if (in.nbDims != 4 || in.d[0] != 1 || in.d[1] != 3)
            throw std::runtime_error("Expected input shape (1,3,H,W), got " + dimsToString(in));

        const int input_h = (int)in.d[2];
        const int input_w = (int)in.d[3];
        std::cout << "Model input size: " << input_w << "x" << input_h << " (WxH)\n";

        const auto out_dims = runner->shapeOf(output_tensor);
        std::cout << "Model output dims: " << dimsToString(out_dims) << "\n";

        // Auto-detect segmentation by checking for proto tensor
        bool is_segmentation = false;
        nvinfer1::Dims proto_dims{};
        size_t proto_floats = 0;

        try
        {
            proto_dims = runner->shapeOf(proto_tensor);
            proto_floats = volume(proto_dims);
            is_segmentation = true;
            std::cout << "[INFO] Segmentation model detected!\n";
            std::cout << "Model proto dims: " << dimsToString(proto_dims) << "\n";
        }
        catch (const std::exception &)
        {
            std::cout << "[INFO] Detection-only model (no segmentation)\n";
        }

        const size_t out_floats = volume(out_dims);
        const size_t in_floats = (size_t)1 * 3 * (size_t)input_h * (size_t)input_w;

        cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
        if (img.empty())
            throw std::runtime_error("Failed to load image: " + image_path);
        std::cout << "Image size: " << img.cols << "x" << img.rows << " (WxH)\n";

        auto t0 = std::chrono::high_resolution_clock::now();

        LetterboxInfo lb;
        cv::Mat lb_img = letterbox(img, input_w, input_h, true, cv::Scalar(114, 114, 114), &lb);

        if (debug)
        {
            std::cout << "[DBG] Letterbox: in=(" << lb.in_w << "x" << lb.in_h
                      << ") new=(" << lb.new_w << "x" << lb.new_h
                      << ") out=(" << lb.out_w << "x" << lb.out_h
                      << ") scale=" << lb.scale
                      << " pad_x=" << lb.pad_x << " pad_y=" << lb.pad_y
                      << "\n";
        }

        std::vector<float> input_nchw;
        bgrToRGBNCHW01(lb_img, input_nchw);

        if (debug)
        {
            printVecStats("input_nchw", input_nchw.data(), input_nchw.size(),
                          std::max<size_t>(1, input_nchw.size() / 12));
        }

        auto t1 = std::chrono::high_resolution_clock::now();

        // Prepare outputs
        std::vector<float> output(out_floats, 0.0f);
        std::vector<float> proto_output;

        multi_camera_rig_common::TensorSpec input_spec{input_tensor, input_nchw.data(), nullptr, in_floats};
        multi_camera_rig_common::TensorSpec output_spec{output_tensor, nullptr, output.data(), out_floats};

        std::vector<multi_camera_rig_common::TensorSpec> outputs = {output_spec};

        if (is_segmentation)
        {
            proto_output.resize(proto_floats, 0.0f);
            multi_camera_rig_common::TensorSpec proto_spec{proto_tensor, nullptr, proto_output.data(), proto_floats};
            outputs.push_back(proto_spec);
        }

        runner->run({input_spec}, outputs);

        auto t2 = std::chrono::high_resolution_clock::now();

        if (debug)
        {
            debugOutput(output, out_dims, conf_thresh);
            if (is_segmentation)
                debugProtoOutput(proto_output, proto_dims);
        }

        // Decide format by dims
        bool is_transposed = false;
        int num_mask_coeffs = 0;

        if (out_dims.nbDims == 3 && out_dims.d[0] == 1)
        {
            const int dim1 = (int)out_dims.d[1];
            const int dim2 = (int)out_dims.d[2];

            // If (1,N,C) with N small and C small => transposed/end2end
            if (dim1 <= 1000 && dim2 < 256)
                is_transposed = true;
            else
                is_transposed = false;

            if (is_segmentation)
                num_mask_coeffs = 32;

            if (is_transposed)
                std::cout << "[INFO] Detected transposed/End2End output format (1, N=" << dim1 << ", C=" << dim2 << ")\n";
            else
                std::cout << "[INFO] Detected standard YOLO output format (1, C=" << dim1 << ", N=" << dim2 << ")\n";
        }
        else
        {
            throw std::runtime_error("Unsupported output dims: " + dimsToString(out_dims));
        }

        std::vector<Det> dets;
        if (is_transposed)
            parseYoloTransposed(output.data(), out_dims, lb, conf_thresh, max_det, dets, num_mask_coeffs);
        else
            parseYoloCxCyWhClassScores(output.data(), out_dims, lb, conf_thresh, iou_thresh, max_det, dets);

        auto t3 = std::chrono::high_resolution_clock::now();

        const double prep_ms  = std::chrono::duration<double, std::milli>(t1 - t0).count();
        const double infer_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
        const double post_ms  = std::chrono::duration<double, std::milli>(t3 - t2).count();

        std::cout << "\n=== Results ===\n"
                  << "Model type: " << (is_segmentation ? "Segmentation" : "Detection") << "\n"
                  << "Preprocessing: " << prep_ms << " ms\n"
                  << "Inference: " << infer_ms << " ms\n"
                  << "Postprocessing: " << post_ms << " ms\n"
                  << "Total: " << (prep_ms + infer_ms + post_ms) << " ms\n"
                  << "\nDetections: " << dets.size() << "\n";

        for (size_t i = 0; i < dets.size(); ++i)
        {
            const auto &d = dets[i];
            std::cout << "  [" << i << "] class=" << d.cls
                      << " conf=" << std::fixed << std::setprecision(3) << d.conf
                      << " bbox=(" << d.x1 << "," << d.y1 << "," << d.x2 << "," << d.y2 << ")";
            if (is_segmentation && !d.mask_coeffs.empty())
            {
                std::cout << " mask_coeffs=[";
                for (size_t k = 0; k < std::min<size_t>(4, d.mask_coeffs.size()); ++k)
                {
                    if (k > 0) std::cout << ", ";
                    std::cout << d.mask_coeffs[k];
                }
                if (d.mask_coeffs.size() > 4)
                    std::cout << ", ... (total: " << d.mask_coeffs.size() << ")";
                std::cout << "]";
            }
            std::cout << "\n";
        }

        // -------------------------
        // Visualization (masks first, then boxes)
        // -------------------------
        cv::Mat vis = img.clone();
        
        auto classColor = [&](int cls) -> cv::Scalar {
            // BGR colors
            switch (cls)
            {
            case 0: return cv::Scalar(0, 255, 0);   // green
            case 1: return cv::Scalar(0, 0, 255);   // red
            case 2: return cv::Scalar(255, 0, 0);   // blue
            default: return cv::Scalar(0, 255, 255); // yellow
            }
        };

        if (is_segmentation && !proto_output.empty())
        {
            std::cout << "[INFO] Rendering masks: thresh=" << mask_thresh
                      << " alpha=" << mask_alpha
                      << " crop_masks=" << (crop_masks ? 1 : 0) << "\n";

            auto proto_chans = protoToMatsCHW(proto_output.data(), proto_dims);

            for (const auto &d : dets)
            {
                if (d.mask_coeffs.size() != 32)
                    continue;

                cv::Mat mask_proto_prob = decodeMaskProtoSpace(proto_chans, d.mask_coeffs);
                cv::Mat mask_bin = maskToOriginalBinary(mask_proto_prob, lb, mask_thresh);

                if (crop_masks)
                {
                    cv::Mat mask_crop = mask_bin.clone();
                    cropMaskToBox(mask_crop, d);
                    overlayMask(vis, mask_crop, classColor(d.cls), mask_alpha);
                }
                else
                {
                    overlayMask(vis, mask_bin, classColor(d.cls), mask_alpha);
                }
            }
        }

        // Draw bboxes + labels
        for (const auto& d : dets)
        {
            const cv::Scalar col = classColor(d.cls);

            cv::rectangle(vis,
                        cv::Point((int)std::round(d.x1), (int)std::round(d.y1)),
                        cv::Point((int)std::round(d.x2), (int)std::round(d.y2)),
                        col, 2);

            std::ostringstream oss;
            oss << "cls:" << d.cls << " " << std::fixed << std::setprecision(2) << d.conf;

            // Anchor label at top-left of box
            drawLabel(vis,
                    (int)std::round(d.x1),
                    (int)std::round(d.y1),
                    oss.str(),
                    col,
                    0.6,   // font_scale
                    2);    // thickness
        }

        const std::string out_path = "output_detections.jpg";
        cv::imwrite(out_path, vis);
        std::cout << "\nVisualization saved to: " << out_path << "\n";

        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}