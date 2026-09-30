#pragma once

#include "ismpc_walking/policy/PolicyBackend.h"
#include "ismpc_walking/policy/PolicyContract.h"

#include <memory>
#include <string>
#include <vector>

namespace ismpc_walking::policy
{

/** Lifecycle of the ONNX policy. */
enum class PolicyState
{
  NoPolicy, // nothing loaded (see lastError() if a load was attempted)
  Ready,    // loaded and validated, not driving
  Active,   // driving the controller        (not reachable yet)
  Releasing // ramping back to manual        (not reachable yet)
};

const char * toString(PolicyState s) noexcept;

/**
 * The only policy object Walking_controller knows about.
 *
 * Step 3: can load and validate a model (never throws, never moves the robot).
 * tick() is still a no-op.
 */
class PolicyRunner
{
public:
  struct Options
  {
    double controller_dt = 0; // the running controller's step (s)
    std::vector<std::string> joint_names; // robot joints, refJointOrder
    std::string dir;  // policies directory (may be empty)
    std::string file; // model to load at start-up (absolute, or relative to dir); empty = none
  };

  explicit PolicyRunner(Options options);
  ~PolicyRunner();

  PolicyRunner(const PolicyRunner &) = delete;
  PolicyRunner & operator=(const PolicyRunner &) = delete;

  /**
   * Loads and validates a model (absolute path, or relative to Options::dir).
   * On success the state is Ready; on any failure it is NoPolicy and lastError() says why.
   * Refused while Active/Releasing. Synchronous, takes milliseconds.
   */
  bool load(const std::string & file);

  /** Drops the loaded model (back to NoPolicy, lastError cleared). Refused while Active/Releasing. */
  bool unload();

  /** Called once per controller step from run(). No-op unless Active/Releasing. */
  void tick() noexcept;

  PolicyState state() const noexcept { return state_; }
  const std::string & lastError() const noexcept { return lastError_; }
  const std::string & loadedFile() const noexcept { return loadedFile_; }
  const PolicyContract * contract() const noexcept { return state_ == PolicyState::NoPolicy ? nullptr : &contract_; }

  /** True once the policy owns walking / Ts / twist (used by the GUI guards later). */
  bool ownsWalking() const noexcept { return false; }

  /** True if this build links ONNX Runtime (ISMPC_WITH_POLICY=ON). */
  static bool builtWithOnnxRuntime() noexcept;

  /** ONNX Runtime version string, or "" when built without it. */
  static std::string onnxRuntimeVersion();

private:
  bool fail(const std::string & path, const std::string & msg);
  std::string resolve(const std::string & file) const;

  Options options_;
  PolicyState state_ = PolicyState::NoPolicy;
  std::string lastError_;
  std::string loadedFile_;
  PolicyContract contract_;
  std::unique_ptr<PolicyBackend> backend_;
};

} // namespace ismpc_walking::policy
