#ifndef MULTI_CAMERA_RIG_COMMON_TRT_RUNNER_HPP
#define MULTI_CAMERA_RIG_COMMON_TRT_RUNNER_HPP

#include <NvInfer.h>
#include <cuda_runtime.h>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

namespace multi_camera_rig_common
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
 * @brief Custom deleter for TensorRT objects
 */
struct TRTDeleter
{
    template <typename T>
    void operator()(T *obj) const
    {
        if (obj)
            delete obj;
    }
};

/**
 * @brief Specification for a single tensor in inference
 */
struct TensorSpec
{
    std::string name;        ///< Tensor name in the engine
    const float* host_data;  ///< Pointer to host input data (for inputs)
    float* host_output;      ///< Pointer to host output buffer (for outputs)
    size_t size_floats;      ///< Size in float elements
};

/**
 * @brief TensorRT inference runner with flexible multi-tensor support
 * 
 * This runner can handle arbitrary numbers of inputs and outputs, making it
 * suitable for various models:
 * - Single input, single output (e.g., YOLO detection)
 * - Multiple inputs, single output (e.g., stereo matching)
 * - Multiple inputs, multiple outputs (e.g., multi-task networks)
 * 
 * Example usage:
 * @code
 * // Single input, single output (detection)
 * TrtRunner runner("/path/to/yolo.engine", true);
 * runner.run(
 *     {{"images", input_ptr, input_size}},
 *     {{"output0", output_ptr, output_size}}
 * );
 * 
 * // Multiple inputs, single output (stereo matching)
 * TrtRunner runner("/path/to/stereo.engine");
 * runner.run(
 *     {{"left", left_ptr, left_size}, {"right", right_ptr, right_size}},
 *     {{"disp", disp_ptr, disp_size}}
 * );
 * @endcode
 */
class TrtRunner
{
public:
    /**
     * @brief Construct TensorRT runner with engine file
     * @param engine_path Path to .engine or .plan file
     * @param verbose Print detailed tensor information during construction
     */
    explicit TrtRunner(const std::string &engine_path, bool verbose = false);
    
    /**
     * @brief Destructor - cleans up CUDA resources
     */
    ~TrtRunner();

    // Non-copyable
    TrtRunner(const TrtRunner &) = delete;
    TrtRunner &operator=(const TrtRunner &) = delete;

    /**
     * @brief Get shape of a tensor by name
     * @param name Tensor name
     * @return Tensor dimensions
     * @throws std::out_of_range if tensor doesn't exist
     */
    nvinfer1::Dims shapeOf(const std::string &name) const;

    /**
     * @brief Execute inference with specified inputs and outputs
     * @param inputs Vector of input tensor specifications
     * @param outputs Vector of output tensor specifications
     * @throws std::runtime_error on CUDA or inference errors
     * 
     * All specified tensors must exist in the engine. Data is copied H2D for
     * inputs, inference is executed, and results are copied D2H for outputs.
     */
    void run(const std::vector<TensorSpec> &inputs, 
             const std::vector<TensorSpec> &outputs);

    /**
     * @brief Check if a tensor exists in the engine
     * @param name Tensor name
     * @return true if tensor exists, false otherwise
     */
    bool hasTensor(const std::string &name) const;

private:
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
     * @brief Verify that a tensor exists in the engine
     * @param name Tensor name
     * @throws std::runtime_error if tensor doesn't exist
     */
    void requireTensor(const std::string &name) const;

    std::unique_ptr<TrtLogger> logger_;
    std::unique_ptr<nvinfer1::IRuntime, TRTDeleter> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine, TRTDeleter> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext, TRTDeleter> context_;
    cudaStream_t stream_{nullptr};

    std::vector<IoTensor> tensors_;
    std::unordered_map<std::string, size_t> idx_;
};

} // namespace multi_camera_rig_common

#endif // MULTI_CAMERA_RIG_COMMON_TRT_RUNNER_HPP
