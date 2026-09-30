#pragma once

#include "ismpc_walking/policy/RobotState.h"

#include <array>
#include <string>
#include <vector>

struct Walking_controller;

namespace ismpc_walking::policy
{

/**
 * The only policy code that touches Walking_controller (one `friend` declaration).
 * Step 6A: read side only. It never writes to the controller.
 */
class ControllerAdapter final : public StateSource
{
public:
  explicit ControllerAdapter(Walking_controller & ctl);

  bool bind(const std::vector<std::string> & contract_joints, std::string & err) override;
  void sample() noexcept override;
  bool read(RobotState & out, std::string & err) noexcept override;

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
};

} // namespace ismpc_walking::policy
