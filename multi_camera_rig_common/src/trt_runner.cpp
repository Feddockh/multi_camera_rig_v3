#include "multi_camera_rig_common/trt_runner.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace multi_camera_rig_common
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

static std::string dimsToString(const nvinfer1::Dims &d)
{
    std::string s = "(";
    for (int i = 0; i < d.nbDims; ++i)
    {
        if (i > 0)
            s += ", ";
        s += std::to_string(d.d[i]);
    }
    s += ")";
    return s;
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
TrtRunner::TrtRunner(const std::string &engine_path, bool verbose)
{
    // Must be set before the CUDA context is created (i.e. before any other
    // CUDA/TensorRT call below) so that cudaStreamSynchronize() blocks the
    // calling thread while the GPU works instead of spin-polling it, which
    // otherwise pins a full CPU core for the duration of every inference call.
    checkCuda(cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync), "cudaSetDeviceFlags");

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
    if (verbose)
        std::cerr << "[INFO] Engine IO tensors (" << n << "):\n";
    
    for (int i = 0; i < n; ++i)
    {
        const char *tn = engine_->getIOTensorName(i);
        IoTensor t;
        t.name = tn;
        t.shape = engine_->getTensorShape(tn);
        t.dtype = engine_->getTensorDataType(tn);
        t.bytes = volume(t.shape) * dtypeBytes(t.dtype);
        
        if (verbose)
        {
            auto mode = engine_->getTensorIOMode(tn);
            std::string mode_str = (mode == nvinfer1::TensorIOMode::kINPUT) ? "INPUT" : "OUTPUT";
            std::cerr << "  - " << tn << "  mode=" << mode_str
                      << "  shape=" << dimsToString(t.shape) 
                      << "  dtype_bytes=" << dtypeBytes(t.dtype)
                      << "  total_bytes=" << t.bytes << "\n";
        }
        
        checkCuda(cudaMalloc(&t.dptr, t.bytes), ("cudaMalloc " + t.name).c_str());
        idx_[t.name] = tensors_.size();
        tensors_.push_back(t);
    }
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

void TrtRunner::run(const std::vector<TensorSpec> &inputs,
                    const std::vector<TensorSpec> &outputs)
{
    // Copy inputs H2D
    for (const auto &spec : inputs)
    {
        requireTensor(spec.name);
        auto &tensor = tensors_.at(idx_.at(spec.name));
        
        const size_t bytes = spec.size_floats * sizeof(float);
        if (bytes > tensor.bytes)
            throw std::runtime_error("Input " + spec.name + " buffer too large: " + 
                                   std::to_string(bytes) + " > " + std::to_string(tensor.bytes));
        
        checkCuda(cudaMemcpyAsync(tensor.dptr, spec.host_data, bytes,
                                  cudaMemcpyHostToDevice, stream_),
                  ("H2D " + spec.name).c_str());
    }

    // Bind all IO tensors
    int n = engine_->getNbIOTensors();
    for (int i = 0; i < n; ++i)
    {
        const char *tn = engine_->getIOTensorName(i);
        auto &t = tensors_.at(idx_.at(tn));
        context_->setTensorAddress(tn, t.dptr);
    }

    // Execute
    if (!context_->enqueueV3(stream_))
        throw std::runtime_error("enqueueV3 failed");

    // Copy outputs D2H
    for (const auto &spec : outputs)
    {
        requireTensor(spec.name);
        auto &tensor = tensors_.at(idx_.at(spec.name));
        
        const size_t bytes = spec.size_floats * sizeof(float);
        if (bytes > tensor.bytes)
            throw std::runtime_error("Output " + spec.name + " buffer too small: " + 
                                   std::to_string(bytes) + " > " + std::to_string(tensor.bytes));
        
        checkCuda(cudaMemcpyAsync(spec.host_output, tensor.dptr, bytes,
                                  cudaMemcpyDeviceToHost, stream_),
                  ("D2H " + spec.name).c_str());
    }

    checkCuda(cudaStreamSynchronize(stream_), "stream sync");
}

bool TrtRunner::hasTensor(const std::string &name) const
{
    return idx_.count(name) > 0;
}

void TrtRunner::requireTensor(const std::string &name) const
{
    if (!hasTensor(name))
        throw std::runtime_error("Engine missing tensor: " + name);
}

} // namespace multi_camera_rig_common
