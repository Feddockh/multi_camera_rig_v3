/*
Compile (adjust CUDA path if needed):
  g++ -O3 -std=c++17 tools/make_tensorrt_yolo.cpp -o tools/make_tensorrt_yolo \
    -I/usr/local/cuda/targets/x86_64-linux/include \
    -L/usr/local/cuda/targets/x86_64-linux/lib \
    -lnvinfer -lnvonnxparser -lnvinfer_plugin -lcudart

Run:
  ./tools/make_tensorrt_yolo /path/to/model.onnx /path/to/out.plan

Optional override shapes:
  ./tools/make_tensorrt_yolo model.onnx out.plan --min=1x3x1088x1440 --opt=1x3x1088x1440 --max=1x3x1088x1440
*/

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <sstream>

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
  void operator()(T* p) const noexcept { delete p; } // TRT 10+ uses delete
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

// Parse "1x3x1088x1440"
static nvinfer1::Dims4 parseDims4(const std::string& s) {
  int a=0,b=0,c=0,d=0;
  char x1=0,x2=0,x3=0;
  std::stringstream ss(s);
  ss >> a >> x1 >> b >> x2 >> c >> x3 >> d;
  if (!ss || x1!='x' || x2!='x' || x3!='x') {
    throw std::runtime_error("Bad shape format '" + s + "', expected like 1x3x1088x1440");
  }
  return nvinfer1::Dims4{a,b,c,d};
}

static bool startsWith(const std::string& s, const std::string& p) {
  return s.rfind(p, 0) == 0;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " model.onnx out.plan [--min=...] [--opt=...] [--max=...]\n";
    return 1;
  }

  const std::string onnx_path = argv[1];
  const std::string engine_path = argv[2];

  // Defaults for your case
  nvinfer1::Dims4 minD{1,3,1088,1440};
  nvinfer1::Dims4 optD{1,3,1088,1440};
  nvinfer1::Dims4 maxD{1,3,1088,1440};

  for (int i = 3; i < argc; ++i) {
    std::string a = argv[i];
    if (startsWith(a, "--min=")) minD = parseDims4(a.substr(6));
    else if (startsWith(a, "--opt=")) optD = parseDims4(a.substr(6));
    else if (startsWith(a, "--max=")) maxD = parseDims4(a.substr(6));
    else throw std::runtime_error("Unknown arg: " + a);
  }

  Logger logger(nvinfer1::ILogger::Severity::kINFO);

  std::unique_ptr<nvinfer1::IBuilder, TrtDeleter<nvinfer1::IBuilder>> builder(
      nvinfer1::createInferBuilder(logger));
  if (!builder) throw std::runtime_error("createInferBuilder failed");

  const uint32_t flags =
      1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  std::unique_ptr<nvinfer1::INetworkDefinition, TrtDeleter<nvinfer1::INetworkDefinition>> network(
      builder->createNetworkV2(flags));
  if (!network) throw std::runtime_error("createNetworkV2 failed");

  std::unique_ptr<nvonnxparser::IParser, TrtDeleter<nvonnxparser::IParser>> parser(
      nvonnxparser::createParser(*network, logger));
  if (!parser) throw std::runtime_error("createParser failed");

  auto onnx = readFile(onnx_path);
  if (!parser->parse(onnx.data(), onnx.size())) {
    const int nerr = parser->getNbErrors();
    std::cerr << "ONNX parse failed with " << nerr << " errors:\n";
    for (int i = 0; i < nerr; ++i) std::cerr << "  " << parser->getError(i)->desc() << "\n";
    return 2;
  }

  std::unique_ptr<nvinfer1::IBuilderConfig, TrtDeleter<nvinfer1::IBuilderConfig>> config(
      builder->createBuilderConfig());
  if (!config) throw std::runtime_error("createBuilderConfig failed");

  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 8ULL << 30);
  config->setBuilderOptimizationLevel(3);

  if (builder->platformHasFastFp16()) {
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
    std::cerr << "[INFO] FP16 enabled\n";
  }

  nvinfer1::IOptimizationProfile* profile = builder->createOptimizationProfile();
  if (!profile) throw std::runtime_error("createOptimizationProfile failed");

  const int n_inputs = network->getNbInputs();
  std::cerr << "[INFO] Network inputs: " << n_inputs << "\n";

  for (int i = 0; i < n_inputs; ++i) {
    nvinfer1::ITensor* t = network->getInput(i);
    const std::string name = t->getName();
    const auto d = t->getDimensions();
    std::cerr << "[INFO] Input '" << name << "' ONNX dims " << dimsToString(d) << "\n";

    // Most Ultralytics ONNX exports have one input named "images" with NCHW.
    // We'll apply the user-provided 4D dims to any 4D input.
    if (d.nbDims != 4) {
      throw std::runtime_error("Expected 4D input tensor for YOLO, got nbDims=" + std::to_string(d.nbDims));
    }

    bool ok = true;
    ok &= profile->setDimensions(name.c_str(), nvinfer1::OptProfileSelector::kMIN, minD);
    ok &= profile->setDimensions(name.c_str(), nvinfer1::OptProfileSelector::kOPT, optD);
    ok &= profile->setDimensions(name.c_str(), nvinfer1::OptProfileSelector::kMAX, maxD);

    std::cerr << "[INFO] Profile '" << name << "': "
              << "MIN " << dimsToString(minD)
              << " OPT " << dimsToString(optD)
              << " MAX " << dimsToString(maxD)
              << (ok ? " [OK]\n" : " [FAILED]\n");

    if (!ok) throw std::runtime_error("Failed to set profile dims for input: " + name);
  }

  const int profile_idx = config->addOptimizationProfile(profile);
  if (profile_idx < 0) throw std::runtime_error("addOptimizationProfile failed");

  std::unique_ptr<nvinfer1::IHostMemory, TrtDeleter<nvinfer1::IHostMemory>> serialized(
      builder->buildSerializedNetwork(*network, *config));
  if (!serialized) throw std::runtime_error("buildSerializedNetwork failed");

  writeFile(engine_path, serialized->data(), serialized->size());
  std::cerr << "Saved: " << engine_path << " (" << serialized->size() << " bytes)\n";
  return 0;
}
