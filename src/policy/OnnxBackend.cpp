#include "ismpc_walking/policy/OnnxBackend.h"

#include <cstring>
#include <fstream>
#include <vector>

#ifdef ISMPC_WITH_POLICY
#  include <onnxruntime_cxx_api.h>
#endif

namespace ismpc_walking::policy
{

#ifdef ISMPC_WITH_POLICY

struct OnnxBackend::Impl
{
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "ismpc_policy"};
  Ort::Session session{nullptr};
  Ort::MemoryInfo mem{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
  Ort::RunOptions runOptions{nullptr};
  Ort::Value inTensor{nullptr};
  Ort::Value outTensor{nullptr};

  std::string inName;
  std::string outName;
  std::string contract;
  int obsDim = 0;
  int actDim = 0;
  // Owned buffers the two tensors point into: never resized after load().
  std::vector<float> in;
  std::vector<float> out;
};

OnnxBackend::OnnxBackend() : impl_(std::make_unique<Impl>()) {}
OnnxBackend::~OnnxBackend() = default;

std::unique_ptr<OnnxBackend> OnnxBackend::load(const std::string & path, std::string & err) noexcept
{
  try
  {
    {
      std::ifstream f(path, std::ios::binary);
      if(!f.good())
      {
        err = "cannot open file '" + path + "'";
        return nullptr;
      }
    }

    std::unique_ptr<OnnxBackend> b(new OnnxBackend());
    Impl & m = *b->impl_;

    Ort::SessionOptions so;
    so.SetIntraOpNumThreads(1);
    so.SetInterOpNumThreads(1);
    so.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);
    so.AddConfigEntry("session.intra_op.allow_spinning", "0");
    so.AddConfigEntry("session.inter_op.allow_spinning", "0");

    m.session = Ort::Session(m.env, path.c_str(), so);

    Ort::AllocatorWithDefaultOptions alloc;

    // --- contract metadata
    {
      Ort::ModelMetadata md = m.session.GetModelMetadata();
      Ort::AllocatedStringPtr v = md.LookupCustomMetadataMapAllocated("ismpc_contract", alloc);
      if(!v || v.get() == nullptr || v.get()[0] == '\0')
      {
        err = "the model has no 'ismpc_contract' metadata (was it exported by the ismpc_hybrid runner?)";
        return nullptr;
      }
      m.contract = v.get();
    }

    // --- tensor signature
    if(m.session.GetInputCount() != 1 || m.session.GetOutputCount() != 1)
    {
      err = "expected exactly one input and one output, got " + std::to_string(m.session.GetInputCount()) + " and "
            + std::to_string(m.session.GetOutputCount());
      return nullptr;
    }
    m.inName = m.session.GetInputNameAllocated(0, alloc).get();
    m.outName = m.session.GetOutputNameAllocated(0, alloc).get();

    auto checkTensor = [&](Ort::TypeInfo info, const char * what, int & dimOut) -> bool
    {
      if(info.GetONNXType() != ONNX_TYPE_TENSOR)
      {
        err = std::string(what) + " is not a tensor";
        return false;
      }
      auto ti = info.GetTensorTypeAndShapeInfo();
      if(ti.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
      {
        err = std::string(what) + " is not float32";
        return false;
      }
      const auto shape = ti.GetShape();
      if(shape.size() != 2 || shape[0] != 1 || shape[1] <= 0)
      {
        std::string s = "[";
        for(size_t i = 0; i < shape.size(); ++i) { s += (i ? ", " : "") + std::to_string(shape[i]); }
        err = std::string(what) + " has shape " + s + "], expected fixed [1, N]";
        return false;
      }
      dimOut = static_cast<int>(shape[1]);
      return true;
    };
    if(!checkTensor(m.session.GetInputTypeInfo(0), "input", m.obsDim)) { return nullptr; }
    if(!checkTensor(m.session.GetOutputTypeInfo(0), "output", m.actDim)) { return nullptr; }

    // --- preallocated buffers and tensors
    m.in.assign(static_cast<size_t>(m.obsDim), 0.f);
    m.out.assign(static_cast<size_t>(m.actDim), 0.f);
    const int64_t inShape[2] = {1, m.obsDim};
    const int64_t outShape[2] = {1, m.actDim};
    m.inTensor = Ort::Value::CreateTensor<float>(m.mem, m.in.data(), m.in.size(), inShape, 2);
    m.outTensor = Ort::Value::CreateTensor<float>(m.mem, m.out.data(), m.out.size(), outShape, 2);

    return b;
  }
  catch(const Ort::Exception & e)
  {
    err = std::string("ONNX Runtime: ") + e.what();
  }
  catch(const std::exception & e)
  {
    err = e.what();
  }
  catch(...)
  {
    err = "unknown error while loading the model";
  }
  return nullptr;
}

int OnnxBackend::obsDim() const noexcept { return impl_->obsDim; }
int OnnxBackend::actionDim() const noexcept { return impl_->actDim; }
const std::string & OnnxBackend::contractJson() const noexcept { return impl_->contract; }

bool OnnxBackend::infer(const float * obs, float * action, std::string & err) noexcept
{
  try
  {
    Impl & m = *impl_;
    std::memcpy(m.in.data(), obs, m.in.size() * sizeof(float));
    const char * inNames[] = {m.inName.c_str()};
    const char * outNames[] = {m.outName.c_str()};
    m.session.Run(m.runOptions, inNames, &m.inTensor, 1, outNames, &m.outTensor, 1);
    std::memcpy(action, m.out.data(), m.out.size() * sizeof(float));
    return true;
  }
  catch(const Ort::Exception & e)
  {
    err = std::string("ONNX Runtime: ") + e.what();
  }
  catch(const std::exception & e)
  {
    err = e.what();
  }
  catch(...)
  {
    err = "unknown error during inference";
  }
  return false;
}

#else // ISMPC_WITH_POLICY

struct OnnxBackend::Impl
{
  std::string contract;
};

OnnxBackend::OnnxBackend() : impl_(std::make_unique<Impl>()) {}
OnnxBackend::~OnnxBackend() = default;

std::unique_ptr<OnnxBackend> OnnxBackend::load(const std::string &, std::string & err) noexcept
{
  err = "this build has no ONNX Runtime (configure with -DISMPC_WITH_POLICY=ON)";
  return nullptr;
}

int OnnxBackend::obsDim() const noexcept { return 0; }
int OnnxBackend::actionDim() const noexcept { return 0; }
const std::string & OnnxBackend::contractJson() const noexcept { return impl_->contract; }
bool OnnxBackend::infer(const float *, float *, std::string & err) noexcept
{
  err = "this build has no ONNX Runtime";
  return false;
}

#endif // ISMPC_WITH_POLICY

} // namespace ismpc_walking::policy
