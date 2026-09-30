#include "ismpc_walking/policy/ObservationBuilder.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace ismpc_walking::policy
{

namespace
{

struct Known
{
  const char * name;
  ObservationBuilder::TermId id;
};

using Id = ObservationBuilder::TermId;

// Keep in sync with expectedObsTermDim() in PolicyContract.cpp and the env cfg.
constexpr Known kKnown[] = {
    {"com_lin_vel", Id::ComLinVel},
    {"base_ang_vel", Id::BaseAngVel},
    {"projected_gravity", Id::ProjectedGravity},
    {"joint_pos", Id::JointPos},
    {"joint_vel", Id::JointVel},
    {"last_sine_params", Id::LastSineParams},
    {"last_walk_action", Id::LastWalkAction},
    {"last_step_timing_action", Id::LastStepTimingAction},
    {"last_twist_action", Id::LastTwistAction},
    {"ismpc_wants_stop", Id::IsmpcWantsStop},
    {"target_twist", Id::TargetTwist},
};

bool lookup(const std::string & name, Id & id)
{
  for(const auto & k : kKnown)
  {
    if(name == k.name)
    {
      id = k.id;
      return true;
    }
  }
  return false;
}

bool bad(std::string & err, const std::string & msg)
{
  err = "observation builder: " + msg;
  return false;
}

double clampTo(double x, const std::array<double, 2> & r) { return std::min(std::max(x, r[0]), r[1]); }

bool allFinite(const double * v, size_t n)
{
  for(size_t i = 0; i < n; ++i)
  {
    if(!std::isfinite(v[i])) { return false; }
  }
  return true;
}

} // namespace

bool ObservationBuilder::isStateTerm(TermId id) noexcept
{
  switch(id)
  {
    case Id::ComLinVel:
    case Id::BaseAngVel:
    case Id::ProjectedGravity:
    case Id::JointPos:
    case Id::JointVel: return true;
    default: return false;
  }
}

const ObservationBuilder::Term * ObservationBuilder::find(const std::string & name) const noexcept
{
  for(const auto & t : terms_)
  {
    if(t.name == name) { return &t; }
  }
  return nullptr;
}

std::unique_ptr<ObservationBuilder> ObservationBuilder::create(const PolicyContract & contract, std::string & err)
{
  std::unique_ptr<ObservationBuilder> b(new ObservationBuilder());
  const int nJoints = static_cast<int>(contract.joint_names.size());
  if(contract.default_pos.size() != contract.joint_names.size())
  {
    bad(err, "joints.default_pos and joints.names differ in size");
    return nullptr;
  }
  int offset = 0;
  for(const auto & spec : contract.terms)
  {
    Term t;
    t.name = spec.name;
    if(!lookup(spec.name, t.id))
    {
      bad(err, "unknown observation term '" + spec.name + "'");
      return nullptr;
    }
    const int expected = expectedObsTermDim(spec.name, nJoints);
    if(expected != spec.dim)
    {
      bad(err, "term '" + spec.name + "' has dim " + std::to_string(spec.dim) + ", the builder expects "
                   + std::to_string(expected));
      return nullptr;
    }
    t.offset = offset;
    t.dim = spec.dim;
    offset += spec.dim;
    b->terms_.push_back(std::move(t));
  }
  if(offset != contract.obs_dim)
  {
    bad(err, "terms add up to " + std::to_string(offset) + " but obs.dim is " + std::to_string(contract.obs_dim));
    return nullptr;
  }
  b->obsDim_ = offset;
  b->defaultPos_ = contract.default_pos;
  b->command_ = contract.command;
  return b;
}

bool ObservationBuilder::fillState(const RobotState & s, std::vector<double> & obs, std::string & err) const
{
  if(static_cast<int>(obs.size()) != obsDim_)
  {
    return bad(err, "observation vector has " + std::to_string(obs.size()) + " values, expected "
                        + std::to_string(obsDim_));
  }
  const size_t n = defaultPos_.size();
  for(const auto & t : terms_)
  {
    double * out = obs.data() + t.offset;
    switch(t.id)
    {
      case Id::ComLinVel:
        for(int i = 0; i < 3; ++i) { out[i] = s.com_lin_vel[static_cast<size_t>(i)]; }
        break;
      case Id::BaseAngVel:
        for(int i = 0; i < 3; ++i) { out[i] = s.base_ang_vel[static_cast<size_t>(i)]; }
        break;
      case Id::ProjectedGravity:
        for(int i = 0; i < 3; ++i) { out[i] = s.projected_gravity[static_cast<size_t>(i)]; }
        break;
      case Id::JointPos:
        if(s.joint_pos.size() != n)
        {
          return bad(err, "joint_pos has " + std::to_string(s.joint_pos.size()) + " values, expected "
                              + std::to_string(n));
        }
        for(size_t i = 0; i < n; ++i) { out[i] = s.joint_pos[i] - defaultPos_[i]; }
        break;
      case Id::JointVel:
        if(s.joint_vel.size() != n)
        {
          return bad(err, "joint_vel has " + std::to_string(s.joint_vel.size()) + " values, expected "
                              + std::to_string(n));
        }
        // mjlab: joint_vel - default_joint_vel, and the default velocity is zero.
        for(size_t i = 0; i < n; ++i) { out[i] = s.joint_vel[i]; }
        break;
      default: continue; // latched terms: fillLatched()
    }
    if(!allFinite(out, static_cast<size_t>(t.dim))) { return bad(err, "term '" + t.name + "' is not finite"); }
  }
  return true;
}

bool ObservationBuilder::fillLatched(const RobotState & s, const DecodedAction & last, std::vector<double> & obs,
                                     std::string & err) const
{
  if(static_cast<int>(obs.size()) != obsDim_)
  {
    return bad(err, "observation vector has " + std::to_string(obs.size()) + " values, expected "
                        + std::to_string(obsDim_));
  }
  for(const auto & t : terms_)
  {
    double * out = obs.data() + t.offset;
    switch(t.id)
    {
      case Id::LastSineParams:
        out[0] = last.offset;
        out[1] = last.frequency;
        out[2] = last.sin_amp;
        out[3] = last.cos_amp;
        break;
      case Id::LastWalkAction: out[0] = last.walk ? 1.0 : 0.0; break;
      case Id::LastStepTimingAction: out[0] = last.ts; break; // before the controller's own ts_range clamp, as in training
      case Id::LastTwistAction:
        for(size_t i = 0; i < 3; ++i) { out[i] = last.twist[i]; }
        break;
      case Id::IsmpcWantsStop:
        if(!std::isfinite(s.ismpc_wants_stop)) { return bad(err, "term '" + t.name + "' is not finite"); }
        out[0] = s.ismpc_wants_stop > 0.5 ? 1.0 : 0.0;
        break;
      case Id::TargetTwist:
        out[0] = clampTo(s.target_twist[0], command_.lin_vel_x);
        out[1] = clampTo(s.target_twist[1], command_.lin_vel_y);
        out[2] = clampTo(s.target_twist[2], command_.ang_vel_z);
        break;
      default: continue; // robot-state terms: fillState()
    }
    if(!allFinite(out, static_cast<size_t>(t.dim))) { return bad(err, "term '" + t.name + "' is not finite"); }
  }
  return true;
}

bool ObservationBuilder::build(const RobotState & s, const DecodedAction & last, std::vector<double> & obs,
                               std::string & err) const
{
  return fillState(s, obs, err) && fillLatched(s, last, obs, err);
}

bool ObservationBuilder::toFloat(const std::vector<double> & obs, std::vector<float> & out, std::string & err)
{
  out.resize(obs.size());
  for(size_t i = 0; i < obs.size(); ++i)
  {
    out[i] = static_cast<float>(obs[i]);
    if(!std::isfinite(out[i])) { return bad(err, "observation[" + std::to_string(i) + "] is not finite as float32"); }
  }
  return true;
}

} // namespace ismpc_walking::policy
