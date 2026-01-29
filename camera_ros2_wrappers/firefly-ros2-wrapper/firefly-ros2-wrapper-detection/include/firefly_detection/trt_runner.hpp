#ifndef FIREFLY_DETECTION_TRT_RUNNER_HPP
#define FIREFLY_DETECTION_TRT_RUNNER_HPP

#include <NvInfer.h>
#include <cuda_runtime.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace firefly_detection
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
 * @brief TensorRT inference runner for YOLO detection
 * Manages engine loading, memory allocation, and inference execution
 */
class TrtRunner
{
public:
    /**
     * @brief Construct TrtRunner from engine file
     * @param engine_path Path to serialized TensorRT engine (.plan file)
     * @param input_name Name of input tensor
     * @param output_name Name of output tensor
     */
    TrtRunner(const std::string &engine_path,
              std::string input_name,
              std::string output_name);

    /**
     * @brief Destructor - cleans up CUDA resources
     */
    ~TrtRunner();

    // Disable copy
    TrtRunner(const TrtRunner &) = delete;
    TrtRunner &operator=(const TrtRunner &) = delete;

    /**
     * @brief Get input tensor name
     */
    const std::string &inputName() const { return input_name_; }

    /**
     * @brief Get output tensor name
     */
    const std::string &outputName() const { return output_name_; }

    /**
     * @brief Get shape of a tensor by name
     * @param name Tensor name
     * @return Tensor dimensions
     */
    nvinfer1::Dims shapeOf(const std::string &name) const;

    /**
     * @brief Run detection inference
     * @param h_images Input image in NCHW float format (host memory)
     * @param images_floats Number of floats in input
     * @param h_output Output detections (host memory)
     * @param output_floats Number of floats in output
     */
    void run(const float *h_images, size_t images_floats, float *h_output, size_t output_floats);

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

    std::string input_name_;
    std::string output_name_;
};

} // namespace firefly_detection

#endif // FIREFLY_DETECTION_TRT_RUNNER_HPP
