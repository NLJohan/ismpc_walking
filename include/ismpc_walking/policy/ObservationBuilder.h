#pragma once

#include "ismpc_walking/policy/ActionDecoder.h"
#include "ismpc_walking/policy/PolicyContract.h"
#include "ismpc_walking/policy/RobotState.h"

#include <memory>
#include <string>
#include <vector>

namespace ismpc_walking::policy
{

/**
 * Assembles the observation vector in the contract's term order from named term builders.
 * Built in doubles; the cast to float32 happens once, at the end (step 6B).
 *
 * Two groups of terms:
 *  - robot-state terms (com_lin_vel, base_ang_vel, projected_gravity, joint_pos, joint_vel and the three controller-side
 *    filtered signals filt_perturbation, filt_zmp_error, filt_dcm_bias): fillState();
 *  - latched / command terms (last_*, ismpc_wants_stop, target_twist): fillLatched().
 * build() does both. toFloat() is the single float32 cast at the end.
 */
class ObservationBuilder
{
public:
  enum class TermId
  {
    ComLinVel,
    BaseAngVel,
    ProjectedGravity,
    JointPos,
    JointVel,
    LastSineParams,
    LastWalkAction,
    LastStepTimingAction,
    LastTwistAction,
    IsmpcWantsStop,
    TargetTwist,
    FiltPerturbation,
    FiltZmpError,
    FiltDcmBias
  };

  struct Term
  {
    std::string name;
    TermId id;
    int offset = 0;
    int dim = 0;
  };

  /** Refuses (nullptr + err) unknown term names, wrong dims, or terms that do not add up to obs_dim. */
  static std::unique_ptr<ObservationBuilder> create(const PolicyContract & contract, std::string & err);

  int obsDim() const noexcept { return obsDim_; }
  const std::vector<Term> & terms() const noexcept { return terms_; }
  const Term * find(const std::string & name) const noexcept;

  /** True for the terms fillState() writes (the others are written by fillLatched()). */
  static bool isStateTerm(TermId id) noexcept;

  /**
   * Writes the robot-state terms into `obs` (size obsDim()); other slots are left untouched.
   * False + err (naming the term) on a size mismatch or a non finite value; `obs` may then be
   * partly written and must not be used.
   */
  bool fillState(const RobotState & state, std::vector<double> & obs, std::string & err) const;

  /**
   * Writes the remaining terms. `last` is the decoder's CURRENT command, i.e. the previous latch's values (or the
   * reset defaults before the first latch), as in training. target_twist is clamped to the contract's command
   * ranges. Same failure behaviour as fillState().
   */
  bool fillLatched(const RobotState & state, const DecodedAction & last, std::vector<double> & obs,
                   std::string & err) const;

  /** fillState() then fillLatched(): the complete observation in contract order, in doubles. */
  bool build(const RobotState & state, const DecodedAction & last, std::vector<double> & obs, std::string & err) const;

  /** The only double -> float32 cast. False + err if any value is not finite after the cast. */
  static bool toFloat(const std::vector<double> & obs, std::vector<float> & out, std::string & err);

private:
  ObservationBuilder() = default;

  int obsDim_ = 0;
  std::vector<Term> terms_;
  std::vector<double> defaultPos_; // contract order
  CommandRanges command_;          // target_twist clamp
};

} // namespace ismpc_walking::policy
