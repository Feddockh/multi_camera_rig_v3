/*
Standalone TensorRT YOLOv8 tester (no ROS required)

Build:
  g++ -O3 -o test_yolov8_trt test_yolov8_trt.cpp \
    -I/usr/local/cuda/include \
    -L/usr/local/cuda/lib64 \
    -lnvinfer -lcudart \
    $(pkg-config --cflags --libs opencv4) \
    -std=c++17

Run:
  ./test_yolov8_trt /path/to/model.plan /path/to/image.jpg [conf_thresh] [iou_thresh] [max_det] [debug]

Notes:
- Supports TRT outputs shaped (1, 4+nc, N) where the first 4 channels are (cx,cy,w,h) in letterboxed coords,
  followed by nc class scores. The best class score is used as confidence.
- If your TRT output shape is (1,6,N) and nc=2, this matches your current engine.
*/

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/dnn/dnn.hpp>

#include <NvInfer.h>
#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <fstream>
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

    inline void checkCuda(cudaError_t e, const char *msg)
    {
        if (e != cudaSuccess)
        {
            throw std::runtime_error(std::string(msg) + ": " + cudaGetErrorString(e));
        }
    }

    class TrtLogger final : public nvinfer1::ILogger
    {
    public:
        void log(Severity severity, const char *msg) noexcept override
        {
            if (severity <= Severity::kWARNING)
            {
                std::cerr << "[TRT] " << msg << "\n";
            }
        }
    };

    std::vector<char> readFile(const std::string &path)
    {
        std::ifstream f(path, std::ios::binary);
        if (!f)
            throw std::runtime_error("Failed to open file: " + path);
        f.seekg(0, std::ios::end);
        const size_t n = static_cast<size_t>(f.tellg());
        f.seekg(0, std::ios::beg);
        std::vector<char> data(n);
        f.read(data.data(), n);
        return data;
    }

    const char *dtypeToStr(nvinfer1::DataType t)
    {
        switch (t)
        {
        case nvinfer1::DataType::kFLOAT:
            return "kFLOAT";
        case nvinfer1::DataType::kHALF:
            return "kHALF";
        case nvinfer1::DataType::kINT8:
            return "kINT8";
        case nvinfer1::DataType::kINT32:
            return "kINT32";
        case nvinfer1::DataType::kBOOL:
            return "kBOOL";
        default:
            return "UNKNOWN";
        }
    }

    size_t dtypeBytes(nvinfer1::DataType t)
    {
        switch (t)
        {
        case nvinfer1::DataType::kFLOAT:
            return 4;
        case nvinfer1::DataType::kHALF:
            return 2;
        case nvinfer1::DataType::kINT8:
            return 1;
        case nvinfer1::DataType::kINT32:
            return 4;
        case nvinfer1::DataType::kBOOL:
            return 1;
        default:
            return 0;
        }
    }

    size_t volume(const nvinfer1::Dims &d)
    {
        size_t v = 1;
        for (int i = 0; i < d.nbDims; ++i)
            v *= static_cast<size_t>(d.d[i]);
        return v;
    }

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

    struct IoTensor
    {
        std::string name;
        nvinfer1::Dims shape{};
        nvinfer1::DataType dtype{};
        void *dptr{nullptr};
        size_t bytes{0};
    };

    class TrtRunner
    {
    public:
        TrtRunner(const std::string &engine_path,
                  std::string input_name,
                  std::string output_name,
                  bool debug)
            : input_name_(std::move(input_name)),
              output_name_(std::move(output_name)),
              debug_(debug)
        {
            logger_ = std::make_unique<TrtLogger>();

            const auto blob = readFile(engine_path);
            runtime_.reset(nvinfer1::createInferRuntime(*logger_));
            if (!runtime_)
                throw std::runtime_error("createInferRuntime failed");

            engine_.reset(runtime_->deserializeCudaEngine(blob.data(), blob.size()));
            if (!engine_)
                throw std::runtime_error("deserializeCudaEngine failed");

            context_.reset(engine_->createExecutionContext());
            if (!context_)
                throw std::runtime_error("createExecutionContext failed");

            checkCuda(cudaStreamCreate(&stream_), "cudaStreamCreate");

            const int n = engine_->getNbIOTensors();
            std::cout << "[INFO] Engine IO tensors (" << n << "):\n";
            for (int i = 0; i < n; ++i)
            {
                const char *tn = engine_->getIOTensorName(i);
                const auto mode = engine_->getTensorIOMode(tn);
                const auto shape = engine_->getTensorShape(tn);
                const auto dtype = engine_->getTensorDataType(tn);

                std::cout << "  - " << tn
                          << "  mode=" << (mode == nvinfer1::TensorIOMode::kINPUT ? "INPUT" : "OUTPUT")
                          << "  shape=" << dimsToString(shape)
                          << "  dtype=" << dtypeToStr(dtype)
                          << "\n";

                IoTensor t;
                t.name = tn;
                t.shape = shape;
                t.dtype = dtype;
                t.bytes = volume(t.shape) * dtypeBytes(t.dtype);

                if (debug_)
                    std::cout << "      bytes=" << t.bytes << "\n";

                checkCuda(cudaMalloc(&t.dptr, t.bytes), ("cudaMalloc " + t.name).c_str());
                idx_[t.name] = tensors_.size();
                tensors_.push_back(t);
            }

            requireTensor(input_name_);
            requireTensor(output_name_);

            std::cout << "[INFO] Using input tensor:  " << input_name_ << "\n";
            std::cout << "[INFO] Using output tensor: " << output_name_ << "\n";
        }

        ~TrtRunner()
        {
            for (auto &t : tensors_)
            {
                if (t.dptr)
                    cudaFree(t.dptr);
            }
            if (stream_)
                cudaStreamDestroy(stream_);
        }

        const std::string &inputName() const { return input_name_; }
        const std::string &outputName() const { return output_name_; }

        nvinfer1::Dims shapeOf(const std::string &name) const { return tensors_.at(idx_.at(name)).shape; }
        nvinfer1::DataType dtypeOf(const std::string &name) const { return tensors_.at(idx_.at(name)).dtype; }
        size_t bytesOf(const std::string &name) const { return tensors_.at(idx_.at(name)).bytes; }

        void run(const float *h_images, size_t images_floats, float *h_output, size_t output_floats)
        {
            auto &input = tensors_.at(idx_.at(input_name_));
            auto &output = tensors_.at(idx_.at(output_name_));

            const size_t in_bytes = images_floats * sizeof(float);
            const size_t out_bytes = output_floats * sizeof(float);

            if (debug_)
            {
                std::cout << "[DBG] H2D input bytes=" << in_bytes << " (engine alloc " << input.bytes << ")"
                          << " dtype=" << dtypeToStr(input.dtype) << "\n";
                std::cout << "[DBG] D2H output bytes=" << out_bytes << " (engine alloc " << output.bytes << ")"
                          << " dtype=" << dtypeToStr(output.dtype) << "\n";
            }

            if (in_bytes > input.bytes)
                throw std::runtime_error("input buffer too small for provided input");
            if (out_bytes > output.bytes)
                throw std::runtime_error("output buffer too small for provided output");

            checkCuda(cudaMemcpyAsync(input.dptr, h_images, in_bytes, cudaMemcpyHostToDevice, stream_), "H2D input");

            const int n = engine_->getNbIOTensors();
            for (int i = 0; i < n; ++i)
            {
                const char *tn = engine_->getIOTensorName(i);
                auto &t = tensors_.at(idx_.at(tn));
                context_->setTensorAddress(tn, t.dptr);
            }

            if (!context_->enqueueV3(stream_))
                throw std::runtime_error("enqueueV3 failed");

            checkCuda(cudaMemcpyAsync(h_output, output.dptr, out_bytes, cudaMemcpyDeviceToHost, stream_), "D2H output");
            checkCuda(cudaStreamSynchronize(stream_), "stream sync");
        }

    private:
        void requireTensor(const std::string &name) const
        {
            if (!idx_.count(name))
                throw std::runtime_error("Engine missing tensor: " + name);
        }

        struct TRTDeleter
        {
            template <typename T>
            void operator()(T *p) const noexcept { delete p; }
        };

        bool debug_{false};
        std::unique_ptr<TrtLogger> logger_;
        std::unique_ptr<nvinfer1::IRuntime, TRTDeleter> runtime_;
        std::unique_ptr<nvinfer1::ICudaEngine, TRTDeleter> engine_;
        std::unique_ptr<nvinfer1::IExecutionContext, TRTDeleter> context_;
        cudaStream_t stream_{};

        std::vector<IoTensor> tensors_;
        std::unordered_map<std::string, size_t> idx_;

        std::string input_name_;
        std::string output_name_;
    };

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
                const float s = get(4 + c, i);
                if (s > best_score)
                {
                    best_score = s;
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
                b.emplace_back((int)boxes[j].x, (int)boxes[j].y, (int)boxes[j].width, (int)boxes[j].height);
                s.emplace_back(scores[j]);
            }

            std::vector<int> keep;
            cv::dnn::NMSBoxes(b, s, conf_thresh, iou_thresh, keep);

            for (int k : keep)
            {
                const int j = idxs[k];
                Det d;
                d.cls = cls;
                d.conf = scores[j];
                d.x1 = boxes[j].x;
                d.y1 = boxes[j].y;
                d.x2 = boxes[j].x + boxes[j].width;
                d.y2 = boxes[j].y + boxes[j].height;
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
            { // cap spam
                const float *p = output.data() + (size_t)c * (size_t)N;
                printVecStats("out[" + std::to_string(c) + "]", p, N, std::max(1, N / 12));
            }

            // how many pass threshold for "best class score"
            int pass = 0;
            for (int i = 0; i < N; ++i)
            {
                float best = -1.f;
                for (int c = 4; c < C; ++c)
                    best = std::max(best, output[(size_t)c * (size_t)N + (size_t)i]);
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
        TrtRunner runner(engine_path, input_tensor, output_tensor, debug);

        const auto in = runner.shapeOf(runner.inputName());
        if (in.nbDims != 4 || in.d[0] != 1 || in.d[1] != 3)
        {
            throw std::runtime_error("Expected input shape (1,3,H,W), got " + dimsToString(in));
        }

        const int input_h = (int)in.d[2];
        const int input_w = (int)in.d[3];
        std::cout << "Model input size: " << input_w << "x" << input_h << " (WxH)\n";

        const auto out_dims = runner.shapeOf(runner.outputName());
        std::cout << "Model output dims: " << dimsToString(out_dims)
                  << " dtype=" << dtypeToStr(runner.dtypeOf(runner.outputName())) << "\n";

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
        runner.run(input_nchw.data(), in_floats, output.data(), out_floats);

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

            cv::putText(vis, label,
                        cv::Point((int)d.x1, std::max(0, (int)d.y1 - 5)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
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
