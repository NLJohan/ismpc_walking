#pragma once

#include "ismpc_walking/policy/ActionDecoder.h"
#include "ismpc_walking/policy/ControlSink.h"
#include "ismpc_walking/policy/PolicyBackend.h"
#include "ismpc_walking/policy/PolicyContract.h"
#include "ismpc_walking/policy/PolicyLibrary.h"
#include "ismpc_walking/policy/ObservationBuilder.h"
#include "ismpc_walking/policy/RobotState.h"
#include "ismpc_walking/policy/ZScoreMonitor.h"

#include <mc_rtc/gui/StateBuilder.h>

#include <atomic>
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
  Active,   // driving the controller (step 7A: ownership and the walk gate only)
  Releasing // ramping back to manual        (not reachable yet)
};

const char * toString(PolicyState s) noexcept;

/**
 * The only policy object Walking_controller knows about.
 *
 * Step 4: can load and validate a model (never throws, never moves the robot) and shows it in the GUI
 * (file dropdown, Refresh/Reload, status labels). tick() only serves deferred GUI rebuilds.
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
    bool debug_no_apply = false; // TEMPORARY (removed in step 9): log the observation while Ready, never move the robot
  };

  /**
   * `source` (may be null) provides the robot state for the observation; the runner owns it.
   * `sink` (may be null, then the policy can never become Active) is the write side of the same adapter;
   * it is NOT owned and must live as long as `source`.
   */
  explicit PolicyRunner(Options options, std::unique_ptr<StateSource> source = nullptr, ControlSink * sink = nullptr);
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

  /**
   * Called once per controller step from run(), on the controller thread. Serves GUI rebuilds and activation
   * requests, then runs the shadow loop (Ready + debug_no_apply) or the Active loop.
   */
  void tick() noexcept;

  /**
   * Adds the Policy GUI under `category` (checkbox, file dropdown, Refresh/Reload, status labels).
   * Call once, after the GUI exists. `gui` must outlive this object's use of it (same controller).
   */
  void addGui(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category);

  /**
   * GUI "Policy" checkbox. Safe from any thread: it only validates the state and queues a request, which tick()
   * carries out on the controller thread. Returns whether the request was accepted (a refusal is logged;
   * activation preconditions that depend on the controller are checked in tick() and reported in lastError()).
   */
  bool setActive(bool on);

  PolicyState state() const noexcept { return state_; }
  const std::string & lastError() const noexcept { return lastError_; }
  const std::string & loadedFile() const noexcept { return loadedFile_; }
  const PolicyContract * contract() const noexcept { return state_ == PolicyState::NoPolicy ? nullptr : &contract_; }

  /** True while the policy owns walking / Ts / twist (read by the GUI, joystick and datastore guards). */
  bool ownsWalking() const noexcept { return state_ == PolicyState::Active || state_ == PolicyState::Releasing; }

  /** True if this build links ONNX Runtime (ISMPC_WITH_POLICY=ON). */
  static bool builtWithOnnxRuntime() noexcept;

  /** ONNX Runtime version string, or "" when built without it. */
  static std::string onnxRuntimeVersion();

private:
  bool fail(const std::string & path, const std::string & msg);
  std::string resolve(const std::string & file) const;
  void buildGui();
  void debugTick() noexcept; // TEMPORARY (step 6): shadow inference + z-score monitor + 1 Hz log while Ready
  void activate() noexcept;  // controller thread: Ready -> Active (or refuse, with lastError)
  void activeTick() noexcept; // controller thread: the Active loop
  // Active -> Ready at once. failure: stop walking, set lastError and log an error. Step 8 adds the ramps.
  void release(const std::string & reason, bool failure) noexcept;

  Options options_;
  PolicyLibrary library_;
  std::string selected_; // file last picked/attempted (what the dropdown shows)
  double benchAvgUs_ = 0, benchMaxUs_ = 0; // load-time inference benchmark
  int benchRuns_ = 0;
  mc_rtc::gui::StateBuilder * gui_ = nullptr;
  std::vector<std::string> guiCategory_;
  bool guiRebuildPending_ = false; // rebuilt from tick(): never remove GUI elements from inside their own callback
  std::atomic<PolicyState> state_{PolicyState::NoPolicy}; // read from the GUI/joystick side too
  ControlSink * sink_ = nullptr; // not owned: same object as source_
  std::atomic<int> request_{0};  // 0 none, 1 activate, 2 release: set by setActive(), consumed by tick()
  long activeTicks_ = 0;         // controller steps since activation
  std::string lastError_;
  std::string loadedFile_;
  PolicyContract contract_;
  std::unique_ptr<PolicyBackend> backend_;
  std::unique_ptr<StateSource> source_;
  std::unique_ptr<ObservationBuilder> builder_;
  std::unique_ptr<ActionDecoder> decoder_; // latch bookkeeping and the current (previous-latch) command
  std::vector<double> obs_;  // observation being built (contract order), sized at load
  std::vector<float> obsF_;  // the same, cast to float32 for the network
  long debugTicks_ = 0;
  // TEMPORARY (step 6B.2, removed in step 9): shadow inference state, only used by debugTick()
  std::unique_ptr<ZScoreMonitor> zmon_;
  ActionDecoder::Raw lastRaw_{}; // last raw network output
  long latchCount_ = 0;          // shadow latches since load
  double inferSumUs_ = 0, inferMaxUs_ = 0; // per-latch inference time (build excluded)
  std::string shadowError_;      // last shadow failure, reported once per second, "" when fine
};

} // namespace ismpc_walking::policy
