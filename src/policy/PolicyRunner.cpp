#include "ismpc_walking/policy/PolicyRunner.h"

#include <mc_rtc/logging.h>

#ifdef ISMPC_WITH_POLICY
#  include <onnxruntime_c_api.h>
#endif

namespace ismpc_walking::policy
{

bool PolicyRunner::builtWithOnnxRuntime() noexcept
{
#ifdef ISMPC_WITH_POLICY
  return true;
#else
  return false;
#endif
}

std::string PolicyRunner::onnxRuntimeVersion()
{
#ifdef ISMPC_WITH_POLICY
  // C API call: resolves libonnxruntime.so at run time but creates no
  // environment, session or thread.
  const OrtApiBase * base = OrtGetApiBase();
  return base ? std::string(base->GetVersionString()) : std::string();
#else
  return {};
#endif
}

PolicyRunner::PolicyRunner()
{
  if(builtWithOnnxRuntime())
  {
    mc_rtc::log::info("[ismpc_policy] state=NoPolicy, built with ONNX Runtime {}", onnxRuntimeVersion());
  }
  else
  {
    mc_rtc::log::info("[ismpc_policy] state=NoPolicy, built without ONNX Runtime (ISMPC_WITH_POLICY=OFF)");
  }
}

PolicyRunner::~PolicyRunner() = default;

void PolicyRunner::tick() noexcept
{
  if(state_ != PolicyState::Active && state_ != PolicyState::Releasing)
  {
    return;
  }
}

} // namespace ismpc_walking::policy
