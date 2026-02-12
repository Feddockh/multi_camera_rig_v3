#include "firefly_reconstruction/trt_runner.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace firefly_reconstruction
{

static inline void checkCuda(cudaError_t e, const char *msg)
{
    if (e != cudaSuccess)
    {
        throw std::runtime_error(std::string(msg) + ": " + cudaGetErrorString(e));
    }
}

static std::vector<char> readFile(const std::string &path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        throw std::runtime_error("Failed to open engine: " + path);
    f.seekg(0, std::ios::end);
    size_t n = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    std::vector<char> data(n);
    f.read(data.data(), n);
    return data;
}

static size_t dtypeBytes(nvinfer1::DataType t)
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

static size_t volume(const nvinfer1::Dims &d)
{
    size_t v = 1;
    for (int i = 0; i < d.nbDims; ++i)
        v *= static_cast<size_t>(d.d[i]);
    return v;
}

// TrtLogger implementation
void TrtLogger::log(Severity severity, const char *msg) noexcept
{
    if (severity <= Severity::kWARNING)
    {
        std::cerr << "[TRT] " << msg << std::endl;
    }
}

// TrtRunner implementation
TrtRunner::TrtRunner(const std::string &engine_path)
{
    logger_ = std::make_unique<TrtLogger>();

    auto blob = readFile(engine_path);
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

    // Allocate for all IO tensors
    int n = engine_->getNbIOTensors();
    for (int i = 0; i < n; ++i)
    {
        const char *tn = engine_->getIOTensorName(i);
        IoTensor t;
        t.name = tn;
        t.shape = engine_->getTensorShape(tn);
        t.dtype = engine_->getTensorDataType(tn);
        t.bytes = volume(t.shape) * dtypeBytes(t.dtype);
        checkCuda(cudaMalloc(&t.dptr, t.bytes), ("cudaMalloc " + t.name).c_str());
        idx_[t.name] = tensors_.size();
        tensors_.push_back(t);
    }

    // Sanity check expected tensors
    requireTensor("left");
    requireTensor("right");
    requireTensor("disp");
}

TrtRunner::~TrtRunner()
{
    for (auto &t : tensors_)
    {
        if (t.dptr)
            cudaFree(t.dptr);
    }
    if (stream_)
        cudaStreamDestroy(stream_);
}

nvinfer1::Dims TrtRunner::shapeOf(const std::string &name) const
{
    return tensors_.at(idx_.at(name)).shape;
}

void TrtRunner::run(const float *h_left_nchw, const float *h_right_nchw, float *h_disp,
                    size_t disp_floats)
{
    auto &left = tensors_.at(idx_.at("left"));
    auto &right = tensors_.at(idx_.at("right"));
    auto &disp = tensors_.at(idx_.at("disp"));

    // H2D
    checkCuda(cudaMemcpyAsync(left.dptr, h_left_nchw, left.bytes,
                              cudaMemcpyHostToDevice, stream_),
              "H2D left");
    checkCuda(cudaMemcpyAsync(right.dptr, h_right_nchw, right.bytes,
                              cudaMemcpyHostToDevice, stream_),
              "H2D right");

    // Bind
    int n = engine_->getNbIOTensors();
    for (int i = 0; i < n; ++i)
    {
        const char *tn = engine_->getIOTensorName(i);
        auto &t = tensors_.at(idx_.at(tn));
        context_->setTensorAddress(tn, t.dptr);
    }

    if (!context_->enqueueV3(stream_))
    {
        throw std::runtime_error("enqueueV3 failed");
    }

    // D2H output
    const size_t out_bytes = disp_floats * sizeof(float);
    if (out_bytes > disp.bytes)
    {
        throw std::runtime_error("disp output buffer too small");
    }
    checkCuda(cudaMemcpyAsync(h_disp, disp.dptr, out_bytes,
                              cudaMemcpyDeviceToHost, stream_),
              "D2H disp");
    checkCuda(cudaStreamSynchronize(stream_), "stream sync");
}

void TrtRunner::requireTensor(const std::string &name) const
{
    if (!idx_.count(name))
        throw std::runtime_error("Engine missing tensor: " + name);
}

} // namespace firefly_reconstruction
