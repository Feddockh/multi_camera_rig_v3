#ifndef FIREFLY_RECONSTRUCTION_TRT_RUNNER_HPP
#define FIREFLY_RECONSTRUCTION_TRT_RUNNER_HPP

#include <NvInfer.h>
#include <cuda_runtime.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace firefly_reconstruction
{

/**
 * @brief TensorRT logger implementation
 */
class TrtLogger : public nvinfer1::ILogger
{
public:
    void log(Severity severity, const char *msg) noexcept override;
};

/**
 * @brief IO Tensor metadata
 */
struct IoTensor
{
    std::string name;
    nvinfer1::Dims shape{};
    nvinfer1::DataType dtype{};
    void *dptr{nullptr};
    size_t bytes{0};
};

/**
 * @brief TensorRT inference runner
 * Manages engine loading, memory allocation, and inference execution
 */
class TrtRunner
{
public:
    /**
     * @brief Construct TrtRunner from engine file
     * @param engine_path Path to serialized TensorRT engine (.plan file)
     */
    explicit TrtRunner(const std::string &engine_path);

    /**
     * @brief Destructor - cleans up CUDA resources
     */
    ~TrtRunner();

    // Disable copy
    TrtRunner(const TrtRunner &) = delete;
    TrtRunner &operator=(const TrtRunner &) = delete;

    /**
     * @brief Get shape of a tensor by name
     * @param name Tensor name
     * @return Tensor dimensions
     */
    nvinfer1::Dims shapeOf(const std::string &name) const;

    /**
     * @brief Run stereo matching inference
     * @param h_left_nchw Left image in NCHW float format (host memory)
     * @param h_right_nchw Right image in NCHW float format (host memory)
     * @param h_disp Output disparity map (host memory)
     * @param disp_floats Number of floats in disparity output
     */
    void run(const float *h_left_nchw, const float *h_right_nchw, float *h_disp,
             size_t disp_floats);

private:
    void requireTensor(const std::string &name) const;

    struct TRTDeleter
    {
        template <typename T>
        void operator()(T *p) const noexcept
        {
            delete p;
        }
    };

    std::unique_ptr<TrtLogger> logger_;
    std::unique_ptr<nvinfer1::IRuntime, TRTDeleter> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine, TRTDeleter> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext, TRTDeleter> context_;
    cudaStream_t stream_{};

    std::vector<IoTensor> tensors_;
    std::unordered_map<std::string, size_t> idx_;
};

} // namespace firefly_reconstruction

#endif // FIREFLY_RECONSTRUCTION_TRT_RUNNER_HPP
