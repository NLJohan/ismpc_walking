#include "ismpc_walking/policy/PolicyRunner.h"

#include "ismpc_walking/policy/OnnxBackend.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#ifdef ISMPC_WITH_POLICY
#  include <onnxruntime_c_api.h>
#endif

namespace ismpc_walking::policy
{

const char * toString(PolicyState s) noexcept
{
  switch(s)
  {
    case PolicyState::NoPolicy: return "NoPolicy";
    case PolicyState::Ready: return "Ready";
    case PolicyState::Active: return "Active";
    case PolicyState::Releasing: return "Releasing";
  }
  return "?";
}

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

PolicyRunner::PolicyRunner(Options options) : options_(std::move(options))
{
  if(builtWithOnnxRuntime())
  {
    mc_rtc::log::info("[ismpc_policy] state=NoPolicy, built with ONNX Runtime {}", onnxRuntimeVersion());
  }
  else
  {
    mc_rtc::log::info("[ismpc_policy] state=NoPolicy, built without ONNX Runtime (ISMPC_WITH_POLICY=OFF)");
  }
  if(!options_.file.empty()) { load(options_.file); }
}

PolicyRunner::~PolicyRunner() = default;

std::string PolicyRunner::resolve(const std::string & file) const
{
  if(file.empty() || file.front() == '/' || options_.dir.empty()) { return file; }
  std::string d = options_.dir;
  if(d.back() != '/') { d += '/'; }
  return d + file;
}

bool PolicyRunner::fail(const std::string & path, const std::string & msg)
{
  backend_.reset();
  contract_ = PolicyContract{};
  loadedFile_.clear();
  state_ = PolicyState::NoPolicy;
  lastError_ = msg;
  mc_rtc::log::error("[ismpc_policy] load failed ({}): {}", path, msg);
  return false;
}

bool PolicyRunner::unload()
{
  if(state_ == PolicyState::Active || state_ == PolicyState::Releasing)
  {
    mc_rtc::log::warning("[ismpc_policy] cannot unload while {}", toString(state_));
    return false;
  }
  backend_.reset();
  contract_ = PolicyContract{};
  loadedFile_.clear();
  lastError_.clear();
  state_ = PolicyState::NoPolicy;
  return true;
}

bool PolicyRunner::load(const std::string & file)
{
  if(state_ == PolicyState::Active || state_ == PolicyState::Releasing)
  {
    mc_rtc::log::warning("[ismpc_policy] cannot load a model while {}", toString(state_));
    return false;
  }
  const std::string path = resolve(file);
  // A failed load always ends in NoPolicy: never keep a half-replaced model.
  unload();
  if(path.empty()) { return fail(path, "no file given"); }

  std::string err;

  // 1. ONNX Runtime: file, metadata, tensor signature
  auto backend = OnnxBackend::load(path, err);
  if(!backend) { return fail(path, err); }

  // 2. contract
  PolicyContract contract;
  if(!PolicyContract::parse(backend->contractJson(), contract, err)) { return fail(path, err); }
  if(!contract.validate(err)) { return fail(path, err); }
  if(!contract.validateAgainst(options_.controller_dt, options_.joint_names, err)) { return fail(path, err); }
  if(backend->obsDim() != contract.obs_dim)
  {
    return fail(path, "the network takes " + std::to_string(backend->obsDim()) + " inputs but the contract says obs.dim = "
                          + std::to_string(contract.obs_dim));
  }
  if(backend->actionDim() != contract.action_dim)
  {
    return fail(path, "the network returns " + std::to_string(backend->actionDim())
                          + " outputs but the contract says action.dim = " + std::to_string(contract.action_dim));
  }

  // 3. smoke inference on a zero observation, plus a timing measurement
  std::vector<float> obs(static_cast<size_t>(contract.obs_dim), 0.f);
  std::vector<float> act(static_cast<size_t>(contract.action_dim), std::numeric_limits<float>::quiet_NaN());
  if(!backend->infer(obs.data(), act.data(), err)) { return fail(path, "smoke inference failed: " + err); }
  for(size_t i = 0; i < act.size(); ++i)
  {
    if(!std::isfinite(act[i])) { return fail(path, "smoke inference returned a non finite value at output " + std::to_string(i)); }
  }
  constexpr int kBenchRuns = 200;
  double sumUs = 0, maxUs = 0;
  for(int i = 0; i < kBenchRuns; ++i)
  {
    const auto t0 = std::chrono::steady_clock::now();
    if(!backend->infer(obs.data(), act.data(), err)) { return fail(path, "inference failed: " + err); }
    const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
    sumUs += us;
    maxUs = std::max(maxUs, us);
  }

  // 4. commit
  backend_ = std::move(backend);
  contract_ = std::move(contract);
  loadedFile_ = path;
  lastError_.clear();
  state_ = PolicyState::Ready;
  mc_rtc::log::info(
      "[ismpc_policy] state=Ready: {} | checkpoint {} iteration {} | obs {} -> action {} | latch {} x {} s | "
      "twist rate limit: {} | inference avg {:.0f} us, max {:.0f} us over {} runs",
      loadedFile_, contract_.checkpoint_stem, contract_.iteration, contract_.obs_dim, contract_.action_dim,
      contract_.latch_ticks, contract_.controller_dt,
      contract_.action.twist_max_delta_per_latch ? "yes" : "none", sumUs / kBenchRuns, maxUs, kBenchRuns);
  return true;
}

void PolicyRunner::tick() noexcept
{
  if(state_ != PolicyState::Active && state_ != PolicyState::Releasing) { return; }
}

} // namespace ismpc_walking::policy
