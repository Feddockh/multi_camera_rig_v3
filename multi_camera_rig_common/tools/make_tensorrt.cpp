/*
Unified TensorRT engine builder for ONNX models (YOLO, stereo, etc.)

Compile:
  g++ -O3 -std=c++17 tools/make_tensorrt.cpp -o tools/make_tensorrt \
    -I/usr/local/cuda/targets/x86_64-linux/include \
    -L/usr/local/cuda/targets/x86_64-linux/lib \
    -lnvinfer -lnvonnxparser -lnvinfer_plugin -lcudart

Basic usage (auto-detects input shapes, replaces dynamic dims with 1):
  ./tools/make_tensorrt model.onnx out.plan

Advanced usage (override shapes for specific inputs):
  ./tools/make_tensorrt model.onnx out.plan --min=1x3x1088x1440 --opt=1x3x1088x1440 --max=1x3x1088x1440

For multi-input models, shapes apply to all 4D inputs.
*/

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

class Logger : public nvinfer1::ILogger {
public:
  explicit Logger(Severity min_severity = Severity::kINFO) : min_(min_severity) {}
  void log(Severity severity, const char* msg) noexcept override {
    if (severity <= min_) std::cerr << "[TRT] " << msg << "\n";
  }
private:
  Severity min_;
};

template <typename T>
struct TrtDeleter {
  void operator()(T* p) const noexcept { delete p; }
};

static std::vector<char> readFile(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("Failed to open file: " + path);
  f.seekg(0, std::ios::end);
  size_t n = static_cast<size_t>(f.tellg());
  f.seekg(0, std::ios::beg);
  std::vector<char> data(n);
  f.read(data.data(), n);
  return data;
}

static void writeFile(const std::string& path, const void* data, size_t nbytes) {
  std::ofstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("Failed to write file: " + path);
  f.write(reinterpret_cast<const char*>(data), nbytes);
}

static std::string dimsToString(const nvinfer1::Dims& d) {
  std::string s = "(";
  for (int i = 0; i < d.nbDims; ++i) {
    s += std::to_string(d.d[i]);
    if (i + 1 < d.nbDims) s += ", ";
  }
  s += ")";
  return s;
}

// Parse "1x3x1088x1440" or similar NxCxHxW format
static nvinfer1::Dims parseDims(const std::string& s) {
  std::vector<int32_t> dims;
  std::stringstream ss(s);
  int val;
  char sep;
  
  ss >> val;
  dims.push_back(val);
  
  while (ss >> sep >> val) {
    if (sep != 'x') throw std::runtime_error("Bad shape format '" + s + "', expected 'x' separator");
    dims.push_back(val);
  }
  
  if (dims.empty() || dims.size() > nvinfer1::Dims::MAX_DIMS) {
    throw std::runtime_error("Invalid number of dimensions in '" + s + "'");
  }
  
  nvinfer1::Dims result;
  result.nbDims = static_cast<int32_t>(dims.size());
  for (size_t i = 0; i < dims.size(); ++i) {
    result.d[i] = dims[i];
  }
  return result;
}

static bool startsWith(const std::string& s, const std::string& prefix) {
  return s.rfind(prefix, 0) == 0;
}

// Replace dynamic dimensions (-1) with concrete value
static nvinfer1::Dims fixDynamicDims(const nvinfer1::Dims& d, int32_t replacement = 1) {
  nvinfer1::Dims fixed = d;
  for (int i = 0; i < d.nbDims; ++i) {
    if (d.d[i] == -1) fixed.d[i] = replacement;
  }
  return fixed;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " model.onnx out.plan [--min=NxCxHxW] [--opt=NxCxHxW] [--max=NxCxHxW]\n"
              << "\nExamples:\n"
              << "  Auto mode (replaces dynamic dims with 1):\n"
              << "    " << argv[0] << " stereo.onnx stereo.plan\n"
              << "  Manual shape override:\n"
              << "    " << argv[0] << " yolo.onnx yolo.plan --min=1x3x640x640 --opt=1x3x1088x1440 --max=1x3x1088x1440\n";
    return 1;
  }

  const std::string onnx_path = argv[1];
  const std::string engine_path = argv[2];

  // Optional shape overrides
  bool has_override = false;
  nvinfer1::Dims min_dims, opt_dims, max_dims;

  for (int i = 3; i < argc; ++i) {
    std::string arg = argv[i];
    if (startsWith(arg, "--min=")) {
      min_dims = parseDims(arg.substr(6));
      has_override = true;
    } else if (startsWith(arg, "--opt=")) {
      opt_dims = parseDims(arg.substr(6));
      has_override = true;
    } else if (startsWith(arg, "--max=")) {
      max_dims = parseDims(arg.substr(6));
      has_override = true;
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }

  if (has_override && (min_dims.nbDims != opt_dims.nbDims || opt_dims.nbDims != max_dims.nbDims)) {
    throw std::runtime_error("All shape overrides (--min, --opt, --max) must have same number of dimensions");
  }

  Logger logger(nvinfer1::ILogger::Severity::kINFO);

  // Create builder
  std::unique_ptr<nvinfer1::IBuilder, TrtDeleter<nvinfer1::IBuilder>> builder(
      nvinfer1::createInferBuilder(logger));
  if (!builder) throw std::runtime_error("createInferBuilder failed");

  // Create network with explicit batch
  const uint32_t flags =
      1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  std::unique_ptr<nvinfer1::INetworkDefinition, TrtDeleter<nvinfer1::INetworkDefinition>> network(
      builder->createNetworkV2(flags));
  if (!network) throw std::runtime_error("createNetworkV2 failed");

  // Parse ONNX
  std::unique_ptr<nvonnxparser::IParser, TrtDeleter<nvonnxparser::IParser>> parser(
      nvonnxparser::createParser(*network, logger));
  if (!parser) throw std::runtime_error("createParser failed");

  auto onnx_data = readFile(onnx_path);
  if (!parser->parse(onnx_data.data(), onnx_data.size())) {
    const int n_errors = parser->getNbErrors();
    std::cerr << "ONNX parse failed with " << n_errors << " errors:\n";
    for (int i = 0; i < n_errors; ++i) {
      std::cerr << "  " << parser->getError(i)->desc() << "\n";
    }
    return 2;
  }

  // Create builder config
  std::unique_ptr<nvinfer1::IBuilderConfig, TrtDeleter<nvinfer1::IBuilderConfig>> config(
      builder->createBuilderConfig());
  if (!config) throw std::runtime_error("createBuilderConfig failed");

  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 8ULL << 30);  // 8GB
  config->setBuilderOptimizationLevel(3);

  // Enable FP16 if available
  if (builder->platformHasFastFp16()) {
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
    std::cerr << "[INFO] FP16 enabled\n";
  } else {
    std::cerr << "[INFO] FP16 not available on this platform\n";
  }

  // Create optimization profile
  nvinfer1::IOptimizationProfile* profile = builder->createOptimizationProfile();
  if (!profile) throw std::runtime_error("createOptimizationProfile failed");

  const int n_inputs = network->getNbInputs();
  std::cerr << "[INFO] Network has " << n_inputs << " input(s)\n";

  for (int i = 0; i < n_inputs; ++i) {
    nvinfer1::ITensor* tensor = network->getInput(i);
    const std::string name = tensor->getName();
    const nvinfer1::Dims onnx_dims = tensor->getDimensions();

    std::cerr << "[INFO] Input '" << name << "' ONNX dims: " << dimsToString(onnx_dims) << "\n";

    nvinfer1::Dims profile_min, profile_opt, profile_max;

    if (has_override) {
      // Use user-provided shapes (validate dimension count matches)
      if (min_dims.nbDims != onnx_dims.nbDims) {
        throw std::runtime_error("Shape override dimension count (" + std::to_string(min_dims.nbDims) +
                                 ") doesn't match input '" + name + "' (" + std::to_string(onnx_dims.nbDims) + ")");
      }
      profile_min = min_dims;
      profile_opt = opt_dims;
      profile_max = max_dims;
    } else {
      // Auto mode: replace dynamic dims (-1) with 1
      profile_min = profile_opt = profile_max = fixDynamicDims(onnx_dims, 1);
    }

    bool ok = true;
    ok &= profile->setDimensions(name.c_str(), nvinfer1::OptProfileSelector::kMIN, profile_min);
    ok &= profile->setDimensions(name.c_str(), nvinfer1::OptProfileSelector::kOPT, profile_opt);
    ok &= profile->setDimensions(name.c_str(), nvinfer1::OptProfileSelector::kMAX, profile_max);

    std::cerr << "[INFO] Profile for '" << name << "':\n"
              << "       MIN: " << dimsToString(profile_min) << "\n"
              << "       OPT: " << dimsToString(profile_opt) << "\n"
              << "       MAX: " << dimsToString(profile_max) << "\n";

    if (!ok) {
      throw std::runtime_error("Failed to set optimization profile for input: " + name);
    }
  }

  // Add profile to config
  const int profile_idx = config->addOptimizationProfile(profile);
  if (profile_idx < 0) throw std::runtime_error("addOptimizationProfile failed");

  // Build serialized engine
  std::cerr << "[INFO] Building TensorRT engine (this may take several minutes)...\n";
  std::unique_ptr<nvinfer1::IHostMemory, TrtDeleter<nvinfer1::IHostMemory>> serialized(
      builder->buildSerializedNetwork(*network, *config));
  if (!serialized) throw std::runtime_error("buildSerializedNetwork failed");

  // Save to file
  writeFile(engine_path, serialized->data(), serialized->size());
  std::cerr << "[SUCCESS] Saved TensorRT engine: " << engine_path
            << " (" << serialized->size() << " bytes)\n";

  return 0;
}
