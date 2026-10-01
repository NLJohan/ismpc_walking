#pragma once

#include <string>

namespace ismpc_walking::policy
{

/**
 * The write side of the controller adapter (the read side is StateSource, RobotState.h).
 * Implemented by ControllerAdapter, the only policy code that touches Walking_controller.
 *
 * Every method is called from the controller thread (PolicyRunner::tick()) and never throws.
 */
class ControlSink
{
public:
  virtual ~ControlSink() = default;

  /** Can the policy take over right now? On false, `err` says why. */
  virtual bool canTakeOver(std::string & err) noexcept = 0;

  /**
   * The policy becomes the owner of the velocity source and of Ts: the previous values of rlVelocityControl and
   * policyControlsTs are remembered, both are forced to true and the RL twist is zeroed (reset default).
   */
  virtual void takeOwnership() noexcept = 0;

  /** Gives rlVelocityControl and policyControlsTs back their remembered values. Does nothing if not owning. */
  virtual void releaseOwnership() noexcept = 0;

  /** The walk gate: Walking_controller::SetPolicyWantsWalk(walk) (true = may walk, false = Stop). */
  virtual void applyWalkGate(bool walk) noexcept = 0;
};

} // namespace ismpc_walking::policy
