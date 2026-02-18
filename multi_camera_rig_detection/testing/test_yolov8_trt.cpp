/*
Standalone TensorRT YOLOv8 tester (no ROS required)

Build (as part of ROS2 workspace):
  This file will be built as part of the multi_camera_rig_detection package.
  It links against multi_camera_rig_common for TrtRunner.

Run:
  ros2 run multi_camera_rig_detection test_yolov8_trt /path/to/model.plan /path/to/image.jpg [conf_thresh] [iou_thresh] [max_det] [debug]

Example:
  ros2 run multi_camera_rig_detection test_yolov8_trt ~/models/yolov8n.plan ~/test_images/sample.jpg 0.3 0.45 100 1

Notes:
- Supports TRT outputs shaped (1, 4+nc, N) where the first 4 channels are (cx,cy,w,h) in letterboxed coords,
  followed by nc class scores. The best class score is used as confidence.
- Now uses multi_camera_rig_common::TrtRunner with multi-tensor API matching yolov8_detector.cpp
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

    struct Det
    {
        int cls{-1};
        float conf{0.f};
        float x1{0.f}, y1{0.f}, x2{0.f}, y2{0.f};
    };

    void parseYoloCxCyWhClassScores(const float *out,
                                    const nvinfer1::Dims &out_dims,
                                    const LetterboxInfo &lb,
                                    float conf_thresh,
                                    float iou_thresh,
                                    int max_det,
                                    std::vector<Det> &dets)
    {
        dets.clear();

        // Expect (1, 4+nc, N)
        if (out_dims.nbDims != 3 || out_dims.d[0] != 1 || out_dims.d[1] < 6)
        {
            throw std::runtime_error("Expected output dims (1,4+nc,N) with nc>=2, got " + dimsToString(out_dims));
        }

        const int C = (int)out_dims.d[1]; // 4 + nc
        const int N = (int)out_dims.d[2];
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
            if (conf < conf_thresh || best_cls < 0)
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
            auto &idxs = kv.second;

            std::vector<cv::Rect> b;
            std::vector<float> s;
            b.reserve(idxs.size());
            s.reserve(idxs.size());

            for (int j : idxs)
            {
                b.push_back(cv::Rect((int)boxes[j].x, (int)boxes[j].y, (int)boxes[j].width, (int)boxes[j].height));
                s.push_back(scores[j]);
            }

            std::vector<int> keep;
            cv::dnn::NMSBoxes(b, s, conf_thresh, iou_thresh, keep);

            for (int k : keep)
            {
                const int orig_idx = idxs[k];
                Det d;
                d.cls = cls;
                d.conf = scores[orig_idx];
                d.x1 = boxes[orig_idx].x;
                d.y1 = boxes[orig_idx].y;
                d.x2 = boxes[orig_idx].x + boxes[orig_idx].width;
                d.y2 = boxes[orig_idx].y + boxes[orig_idx].height;
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

    void debugOutput(const std::vector<float> &output, const nvinfer1::Dims &out_dims, float conf_thresh)
    {
        printVecStats("output(flat)", output.data(), output.size(), std::max<size_t>(1, output.size() / 12));

        if (out_dims.nbDims == 3 && out_dims.d[0] == 1 && out_dims.d[1] >= 6)
        {
            const int C = (int)out_dims.d[1];
            const int N = (int)out_dims.d[2];
            for (int c = 0; c < std::min(C, 8); ++c)
            {
                const float *chan = output.data() + (size_t)c * (size_t)N;
                printVecStats("  chan_" + std::to_string(c), chan, N, std::max(1, N / 10));
            }

            // how many pass threshold for "best class score"
            int pass = 0;
            for (int i = 0; i < N; ++i)
            {
                float best = -1.0f;
                for (int c = 0; c < (C - 4); ++c)
                {
                    float score = output[(4 + c) * N + i];
                    if (score > best)
                        best = score;
                }
                if (best >= conf_thresh)
                    pass++;
            }
            std::cout << "[DBG] candidates with max_class_score>=thresh: " << pass << "/" << N
                      << " (thresh=" << conf_thresh << ")\n";
        }
    }

} // namespace

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <engine.plan> <image> [conf_thresh] [iou_thresh] [max_det] [debug]\n";
        return 1;
    }

    const std::string engine_path = argv[1];
    const std::string image_path = argv[2];

    const float conf_thresh = (argc > 3) ? std::stof(argv[3]) : 0.25f;
    const float iou_thresh = (argc > 4) ? std::stof(argv[4]) : 0.45f;
    const int max_det = (argc > 5) ? std::stoi(argv[5]) : 100;
    const bool debug = (argc > 6) ? (std::stoi(argv[6]) != 0) : false;

    const std::string input_tensor = "images";
    const std::string output_tensor = "output0";

    try
    {
        std::cout << "Loading engine: " << engine_path << "\n";
        auto runner = std::make_unique<multi_camera_rig_common::TrtRunner>(engine_path, true);

        const auto in = runner->shapeOf(input_tensor);
        if (in.nbDims != 4 || in.d[0] != 1 || in.d[1] != 3)
        {
            throw std::runtime_error("Expected input shape (1,3,H,W), got " + dimsToString(in));
        }

        const int input_h = (int)in.d[2];
        const int input_w = (int)in.d[3];
        std::cout << "Model input size: " << input_w << "x" << input_h << " (WxH)\n";

        const auto out_dims = runner->shapeOf(output_tensor);
        std::cout << "Model output dims: " << dimsToString(out_dims) << "\n";

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

        std::vector<float> output(out_floats, 0.0f);
        
        // Run inference using new multi-tensor API
        multi_camera_rig_common::TensorSpec input_spec{input_tensor, input_nchw.data(), nullptr, in_floats};
        multi_camera_rig_common::TensorSpec output_spec{output_tensor, nullptr, output.data(), out_floats};
        runner->run({input_spec}, {output_spec});

        auto t2 = std::chrono::high_resolution_clock::now();

        if (debug)
        {
            debugOutput(output, out_dims, conf_thresh);
        }

        std::vector<Det> dets;
        parseYoloCxCyWhClassScores(output.data(), out_dims, lb, conf_thresh, iou_thresh, max_det, dets);

        auto t3 = std::chrono::high_resolution_clock::now();

        const double prep_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        const double infer_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
        const double post_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();

        std::cout << "\n=== Results ===\n"
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
                      << " bbox=(" << d.x1 << "," << d.y1 << "," << d.x2 << "," << d.y2 << ")\n";
        }

        // Visualization
        cv::Mat vis = img.clone();
        for (const auto &d : dets)
        {
            cv::rectangle(vis, cv::Point((int)d.x1, (int)d.y1), cv::Point((int)d.x2, (int)d.y2),
                          cv::Scalar(0, 255, 0), 2);

            std::ostringstream oss;
            oss << "cls:" << d.cls << " " << std::fixed << std::setprecision(2) << d.conf;
            const std::string label = oss.str();

            cv::putText(vis, label, cv::Point((int)d.x1, std::max(0, (int)d.y1 - 5)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 255, 0), 2);
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
