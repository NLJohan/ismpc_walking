#include "ismpc_walking/policy/ControllerAdapter.h"

#include "ismpc_walking/Walking_controller.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <unordered_map>

namespace ismpc_walking::policy
{

namespace
{
// Body sensor named in the ObserverPipelines block of ismpc_walking.in.yaml (diagnostics only).
constexpr const char * kGyroSensor = "FloatingBase";
// mbIndex_ value for a contract joint that has no single DoF in the robot model (mbc.alpha not readable).
constexpr size_t kNoMbIndex = static_cast<size_t>(-1);

// Training reads com_lin_vel and ismpc_wants_stop through the datastore, and the action base hands the controller
// output of the PREVIOUS dispatch to the observation (McRtcActionBase._advance_control_period collects before it
// dispatches). For a latch at step k the policy saw robot terms of step k-1, com_lin_vel computed at step k-2 and
// the wants_stop flag as left by the end of step k-2's run, which is what sample() sees at the start of step k-1.
// [I] Inferred from reading mc_rtc_action_base.py / ismpc_sine_action.py, not measured. Set both to 0 to disable.
constexpr size_t kComLinVelLagTicks = 2;
constexpr size_t kWantsStopLagTicks = 1;

// ISMPC_Solver's own default for m_rl_com_z_offset (the value manual mode has always run with).
constexpr double kManualOffset = 0.95;

Vec3 toVec3(const Eigen::Vector3d & v) { return {v.x(), v.y(), v.z()}; }
} // namespace

ControllerAdapter::ControllerAdapter(Walking_controller & ctl) : ctl_(ctl) {}

bool ControllerAdapter::bind(const std::vector<std::string> & contract_joints, std::string & err)
{
  try
  {
    const auto & robot = ctl_.realRobot();
    const auto & order = robot.refJointOrder();
    std::unordered_map<std::string, size_t> refPos;
    for(size_t i = 0; i < order.size(); ++i) { refPos.emplace(order[i], i); }

    std::vector<size_t> ref, mb;
    std::string notOneDof;
    ref.reserve(contract_joints.size());
    mb.reserve(contract_joints.size());
    for(const auto & name : contract_joints)
    {
      const auto it = refPos.find(name);
      if(it == refPos.end())
      {
        err = "robot binding: joint '" + name + "' is not in the robot's refJointOrder";
        return false;
      }
      if(!robot.hasJoint(name))
      {
        err = "robot binding: joint '" + name + "' does not exist in the robot model";
        return false;
      }
      // joint_pos / joint_vel come from encoderValues() / encoderVelocities() by refJointOrder index, so a joint
      // without a single DoF in the model is still readable. Only the mbc.alpha diagnostic needs one DoF.
      const auto mi = robot.jointIndexByName(name);
      const int dof = robot.mb().joint(static_cast<int>(mi)).dof();
      ref.push_back(it->second);
      if(dof == 1) { mb.push_back(static_cast<size_t>(mi)); }
      else
      {
        mb.push_back(kNoMbIndex);
        notOneDof += (notOneDof.empty() ? "" : ", ") + name + " (dof " + std::to_string(dof) + ")";
      }
    }
    if(!notOneDof.empty())
    {
      mc_rtc::log::warning(
          "[ismpc_policy] robot binding: contract joints that are not 1-DoF in the robot model: {}. Their values are read "
          "from encoderValues()/encoderVelocities() by name; mbc.alpha is not used for them.",
          notOneDof);
    }
    refIndex_ = std::move(ref);
    mbIndex_ = std::move(mb);
    filled_ = 0;
    return true;
  }
  catch(const std::exception & e)
  {
    err = std::string("robot binding failed: ") + e.what();
    return false;
  }
}

void ControllerAdapter::sample() noexcept
{
  try
  {
    hist_[2] = hist_[1];
    hist_[1] = hist_[0];
    hist_[0].com_lin_vel = toVec3(ctl_.estimatedComLinVel());
    hist_[0].wants_stop = ctl_.ismpcWantsStop() ? 1.0 : 0.0;
    filled_ = std::min<size_t>(filled_ + 1, hist_.size());
  }
  catch(...)
  {
  }
}

bool ControllerAdapter::read(RobotState & out, std::string & err) noexcept
{
  try
  {
    if(refIndex_.empty())
    {
      err = "robot state: no joints bound";
      return false;
    }
    const auto & robot = ctl_.realRobot();
    const size_t nRef = robot.refJointOrder().size();

    // Frames. sva: posW().rotation() maps world axes into body axes (world -> body).
    const Eigen::Matrix3d E = robot.posW().rotation();
    // com_lin_vel is already in base axes. Lagged as in training when the history is long enough, else the
    // oldest sample we have (only the first two steps after a load).
    if(filled_ > 0)
    {
      out.com_lin_vel = hist_[std::min(kComLinVelLagTicks, filled_ - 1)].com_lin_vel;
      out.ismpc_wants_stop = hist_[std::min(kWantsStopLagTicks, filled_ - 1)].wants_stop;
    }
    else
    {
      out.com_lin_vel = toVec3(ctl_.estimatedComLinVel());
      out.ismpc_wants_stop = ctl_.ismpcWantsStop() ? 1.0 : 0.0;
    }
    out.target_twist = toVec3(ctl_.user_reference_velocity); // human/joystick intent; the builder clamps it
    out.base_ang_vel = toVec3(E * robot.velW().angular());               // world -> body
    out.projected_gravity = toVec3(E * Eigen::Vector3d(0., 0., -1.));    // mjlab: R^T * g_w

    // Joints, by name (contract order).
    const auto & enc = robot.encoderValues();
    if(enc.size() != nRef)
    {
      err = "robot state: encoderValues() has " + std::to_string(enc.size()) + " values, expected "
            + std::to_string(nRef);
      return false;
    }
    const auto & encVel = robot.encoderVelocities();
    const bool haveEncVel = (encVel.size() == nRef);
    const auto & alpha = robot.mbc().alpha;

    const size_t n = refIndex_.size();
    out.joint_pos.assign(n, 0.);
    out.joint_vel.assign(n, 0.);
    out.diag = RobotState::Diagnostics{};
    out.diag.encoder_vel_max_abs = haveEncVel ? 0. : -1.;
    for(size_t i = 0; i < n; ++i)
    {
      out.joint_pos[i] = enc[refIndex_[i]];
      const double a = (mbIndex_[i] == kNoMbIndex) ? 0. : alpha[mbIndex_[i]][0];
      out.diag.alpha_max_abs = std::max(out.diag.alpha_max_abs, std::fabs(a));
      if(haveEncVel)
      {
        const double v = encVel[refIndex_[i]];
        out.diag.encoder_vel_max_abs = std::max(out.diag.encoder_vel_max_abs, std::fabs(v));
        out.joint_vel[i] = v;
      }
      else { out.joint_vel[i] = a; }
    }
    out.diag.joint_vel_source = haveEncVel ? "encoderVelocities()" : "mbc.alpha (encoderVelocities() is empty)";

    if(robot.hasBodySensor(kGyroSensor))
    {
      out.diag.has_gyro = true;
      out.diag.gyro = toVec3(robot.bodySensor(kGyroSensor).angularVelocity());
    }
    return true;
  }
  catch(const std::exception & e)
  {
    err = std::string("robot state: ") + e.what();
    return false;
  }
  catch(...)
  {
    err = "robot state: unknown error";
    return false;
  }
}

bool ControllerAdapter::canTakeOver(std::string & err) noexcept
{
  try
  {
    if(!ctl_.MPC_thread_ready)
    {
      err = "the MPC thread is not ready yet";
      return false;
    }
    // 7B: the policy writes the RlSine parameters; with another CoM-height signal selected they would be ignored.
    if(ctl_.ismpc_solver().TestSignal() != CoMHeightTestSignal::RlSine)
    {
      err = "the CoM height signal is not RlSine (" + ToString(ctl_.ismpc_solver().TestSignal())
            + "); select RL in the Walking GUI first";
      return false;
    }
    if(!ctl_.active)
    {
      err = "the controller is not active (tick Active in the Walking GUI, or set walking_controller.auto_start.activate)";
      return false;
    }
    return true;
  }
  catch(...)
  {
    err = "unknown error while checking the controller";
    return false;
  }
}

void ControllerAdapter::takeOwnership() noexcept
{
  if(owning_) { return; }
  savedRlVelocityControl_ = ctl_.rlVelocityControl;
  savedPolicyControlsTs_ = ctl_.policyControlsTs;
  ctl_.rlVelocityControl = true;
  ctl_.policyControlsTs = true;
  ctl_.rl_reference_velocity.setZero(); // reset default; never leave a stale twist to the first tick
  owning_ = true;
}

void ControllerAdapter::releaseOwnership() noexcept
{
  if(!owning_) { return; }
  // Back to the manual defaults, so nothing the policy last wrote is left frozen in the controller (CoM height sine,
  // Ts, RL twist). Immediate, no ramp (ramps are step 8). Stop is deliberately left as the policy set it.
  auto & solver = ctl_.ismpc_solver();
  solver.SetOffset(kManualOffset);
  solver.SetFrequency(0.);
  solver.SetSinAmp(0.);
  solver.SetCosAmp(0.);
  ctl_.ts(Walking_controller::kDefaultTSteps);
  ctl_.rl_reference_velocity.setZero();
  ctl_.rlVelocityControl = savedRlVelocityControl_;
  ctl_.policyControlsTs = savedPolicyControlsTs_;
  owning_ = false;
}

void ControllerAdapter::applyWalkGate(bool walk) noexcept { ctl_.SetPolicyWantsWalk(walk); }

// Same seven writes, same order as IsmpcSineAction._advance_sine_period + the datastore inputs of training. Controller
// thread only, no lock: the sine setters and Ts are plain doubles that the MPC thread reads outside mutex_mpc_ (as in
// training), and rl_reference_velocity is read by the mux on this same thread.
void ControllerAdapter::applyCommand(const DecodedAction & c) noexcept
{
  auto & solver = ctl_.ismpc_solver();
  solver.SetOffset(c.offset);
  solver.SetFrequency(c.frequency);
  solver.SetSinAmp(c.sin_amp);
  solver.SetCosAmp(c.cos_amp);
  ctl_.SetPolicyWantsWalk(c.walk);
  ctl_.SetPolicyStepTiming(c.ts); // goes through the controller's own ts_range clamp
  ctl_.rl_reference_velocity = Eigen::Vector3d(c.twist[0], c.twist[1], c.twist[2]);
}

} // namespace ismpc_walking::policy