#pragma once

#include "ismpc_walking/policy/ControlSink.h"
#include "ismpc_walking/policy/RobotState.h"

#include <array>
#include <string>
#include <vector>

struct Walking_controller;

namespace ismpc_walking::policy
{

/**
 * The only policy code that touches Walking_controller (one `friend` declaration).
 * Read side (StateSource): never writes to the controller.
 * Write side (ControlSink, step 7): ownership of the velocity source and of Ts, the walk gate (7A) and the
 * whole command: sine parameters, Ts and twist (7B).
 */
class ControllerAdapter final : public StateSource, public ControlSink
{
public:
  explicit ControllerAdapter(Walking_controller & ctl);

  bool bind(const std::vector<std::string> & contract_joints, std::string & err) override;
  void sample() noexcept override;
  bool read(RobotState & out, std::string & err) noexcept override;

  bool canTakeOver(std::string & err) noexcept override;
  void takeOwnership() noexcept override;
  void releaseOwnership() noexcept override;
  void applyWalkGate(bool walk) noexcept override;
  void applyCommand(const DecodedAction & cmd) noexcept override;

private:
  Walking_controller & ctl_;
  std::vector<size_t> refIndex_; // contract joint i -> index in refJointOrder() / encoderValues()
  std::vector<size_t> mbIndex_;  // contract joint i -> joint index in the robot's MultiBody

  // Per-step samples of the two terms training reads through the datastore (see kComLinVelLagTicks in the .cpp).
  struct Sample
  {
    Vec3 com_lin_vel{};
    double wants_stop = 0;
  };
  std::array<Sample, 3> hist_{}; // hist_[0] = this step, hist_[1] = one step ago, hist_[2] = two steps ago
  size_t filled_ = 0;            // valid entries in hist_

  // Ownership (step 7): values to give back on release.
  bool owning_ = false;
  bool savedRlVelocityControl_ = false;
  bool savedPolicyControlsTs_ = true;
};

} // namespace ismpc_walking::policy
