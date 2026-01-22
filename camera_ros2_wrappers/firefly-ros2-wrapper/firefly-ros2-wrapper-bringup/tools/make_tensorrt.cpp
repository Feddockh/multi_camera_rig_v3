/*
Compile with:
  g++ -O3 -std=c++17 tools/make_tensorrt.cpp -o tools/make_tensorrt \
  -I/usr/local/cuda/targets/x86_64-linux/include \
  -L/usr/local/cuda/targets/x86_64-linux/lib \
  -lnvinfer -lnvonnxparser -lnvinfer_plugin -lcudart

Verify with:
ls -l tools/make_tensorrt

Run with:
    ./tools/make_tensorrt /path/to/foundation_stereo.onnx /tmp/test.plan
*/

#include <NvInfer.h>
#include <NvOnnxParser.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class Logger : public nvinfer1::ILogger
{
public:
    explicit Logger(Severity min_severity = Severity::kINFO) : min_(min_severity) {}
    void log(Severity severity, const char *msg) noexcept override
    {
        if (severity <= min_)
            std::cerr << "[TRT] " << msg << "\n";
    }

private:
    Severity min_;
};

static std::vector<char> readFile(const std::string &path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        throw std::runtime_error("Failed to open file: " + path);
    f.seekg(0, std::ios::end);
    size_t n = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    std::vector<char> data(n);
    f.read(data.data(), n);
    return data;
}

static void writeFile(const std::string &path, const void *data, size_t nbytes)
{
    std::ofstream f(path, std::ios::binary);
    if (!f)
        throw std::runtime_error("Failed to write file: " + path);
    f.write(reinterpret_cast<const char *>(data), nbytes);
}

static std::string dimsToString(const nvinfer1::Dims &d)
{
    std::string s = "(";
    for (int i = 0; i < d.nbDims; ++i)
    {
        s += std::to_string(d.d[i]);
        if (i + 1 < d.nbDims)
            s += ", ";
    }
    s += ")";
    return s;
}

template <typename T>
struct TrtDeleter
{
    void operator()(T *p) const noexcept { delete p; }
};

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " /path/model.onnx /path/out.plan\n";
        return 1;
    }

    const std::string onnx_path = argv[1];
    const std::string engine_path = argv[2];

    Logger logger(nvinfer1::ILogger::Severity::kINFO);

    // Builder
    std::unique_ptr<nvinfer1::IBuilder, TrtDeleter<nvinfer1::IBuilder>> builder(
        nvinfer1::createInferBuilder(logger));
    if (!builder)
        throw std::runtime_error("createInferBuilder failed");

    // Network (explicit batch)
    const uint32_t flags =
        1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    std::unique_ptr<nvinfer1::INetworkDefinition, TrtDeleter<nvinfer1::INetworkDefinition>> network(
        builder->createNetworkV2(flags));
    if (!network)
        throw std::runtime_error("createNetworkV2 failed");

    // Parser
    std::unique_ptr<nvonnxparser::IParser, TrtDeleter<nvonnxparser::IParser>> parser(
        nvonnxparser::createParser(*network, logger));
    if (!parser)
        throw std::runtime_error("createParser failed");

    auto onnx = readFile(onnx_path);
    if (!parser->parse(onnx.data(), onnx.size()))
    {
        const int nerr = parser->getNbErrors();
        std::cerr << "ONNX parse failed with " << nerr << " errors:\n";
        for (int i = 0; i < nerr; ++i)
        {
            std::cerr << "  " << parser->getError(i)->desc() << "\n";
        }
        return 2;
    }

    // Config
    std::unique_ptr<nvinfer1::IBuilderConfig, TrtDeleter<nvinfer1::IBuilderConfig>> config(
        builder->createBuilderConfig());
    if (!config)
        throw std::runtime_error("createBuilderConfig failed");

    config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 8ULL << 30);
    config->setBuilderOptimizationLevel(3);

    if (builder->platformHasFastFp16())
    {
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
        std::cerr << "[INFO] FP16 enabled\n";
    }
    else
    {
        std::cerr << "[INFO] FP16 not supported on this platform\n";
    }

    // Optimization profile
    // IMPORTANT: do NOT manage IOptimizationProfile with unique_ptr (protected destructor)
    nvinfer1::IOptimizationProfile *profile = builder->createOptimizationProfile();
    if (!profile)
        throw std::runtime_error("createOptimizationProfile failed");

    const int n_inputs = network->getNbInputs();
    std::cerr << "[INFO] Network inputs: " << n_inputs << "\n";

    for (int i = 0; i < n_inputs; ++i)
    {
        nvinfer1::ITensor *t = network->getInput(i);
        const std::string name = t->getName();
        nvinfer1::Dims d = t->getDimensions();

        // Replace dynamic dims (-1) with 1 (mirrors your python)
        for (int k = 0; k < d.nbDims; ++k)
        {
            if (d.d[k] == -1)
                d.d[k] = 1;
        }

        bool ok = true;
        ok &= profile->setDimensions(name.c_str(), nvinfer1::OptProfileSelector::kMIN, d);
        ok &= profile->setDimensions(name.c_str(), nvinfer1::OptProfileSelector::kOPT, d);
        ok &= profile->setDimensions(name.c_str(), nvinfer1::OptProfileSelector::kMAX, d);

        std::cerr << "[INFO] Input '" << name << "' dims " << dimsToString(t->getDimensions())
                  << " -> profile " << dimsToString(d)
                  << (ok ? " [OK]\n" : " [FAILED]\n");

        if (!ok)
        {
            throw std::runtime_error("Failed to set profile dims for input: " + name);
        }
    }

    // Config takes ownership; returns profile index
    const int profile_idx = config->addOptimizationProfile(profile);
    if (profile_idx < 0)
        throw std::runtime_error("addOptimizationProfile failed");

    // Build serialized engine
    std::unique_ptr<nvinfer1::IHostMemory, TrtDeleter<nvinfer1::IHostMemory>> serialized(
        builder->buildSerializedNetwork(*network, *config));
    if (!serialized)
        throw std::runtime_error("buildSerializedNetwork failed");

    writeFile(engine_path, serialized->data(), serialized->size());
    std::cerr << "Saved: " << engine_path << " (" << serialized->size() << " bytes)\n";

    return 0;
}
