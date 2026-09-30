#pragma once

#include <string>

namespace ismpc_walking::policy
{

/** Lifecycle of the ONNX policy (plan section 7.1). */
enum class PolicyState
{
  NoPolicy, // nothing loaded
  Ready,    // loaded and validated, not driving          (not reachable yet)
  Active,   // driving the controller                      (not reachable yet)
  Releasing // ramping back to manual                      (not reachable yet)
};

/**
 * Step 2 skeleton: the only piece Walking_controller knows about.
 *
 * Always in NoPolicy for now; tick() returns immediately. Constructing it
 * creates no ONNX Runtime object (no session, no threads, no allocation
 * beyond this object).
 */
class PolicyRunner
{
public:
  PolicyRunner();
  ~PolicyRunner();

  PolicyRunner(const PolicyRunner &) = delete;
  PolicyRunner & operator=(const PolicyRunner &) = delete;

  /** Called once per controller step from run(). No-op unless Active/Releasing. */
  void tick() noexcept;

  PolicyState state() const noexcept { return state_; }

  /** True once the policy owns walking / Ts / twist (used by the GUI guards later). */
  bool ownsWalking() const noexcept { return false; }

  /** True if this build links ONNX Runtime (ISMPC_WITH_POLICY=ON). */
  static bool builtWithOnnxRuntime() noexcept;

  /** ONNX Runtime version string, or "" when built without it. */
  static std::string onnxRuntimeVersion();

private:
  PolicyState state_ = PolicyState::NoPolicy;
};

} // namespace ismpc_walking::policy
