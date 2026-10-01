#pragma once

#include <array>
#include <string>
#include <vector>

namespace ismpc_walking::policy
{

using Vec3 = std::array<double, 3>;

/**
 * Raw robot quantities the observation is built from, in the frames and units the training
 * used. Plain data: no mc_rtc, no Eigen. Joint vectors are in the CONTRACT joint order
 * (`joints.names`), absolute values (the builder subtracts the contract's default_pos).
 */
struct RobotState
{
  Vec3 com_lin_vel{};        // CoM linear velocity, base (body) axes, m/s
  Vec3 base_ang_vel{};       // base angular velocity, body axes, rad/s
  Vec3 projected_gravity{};  // world (0, 0, -1) expressed in body axes
  std::vector<double> joint_pos; // rad
  std::vector<double> joint_vel; // rad/s
  double ismpc_wants_stop = 0;   // 1 = ISMPC would have stopped on its own (advisory), else 0
  Vec3 target_twist{};           // user reference velocity (vx, vy, omega), NOT clamped: the builder clamps it
  Vec3 filt_signals{};           // low-passed (perturbation, ZMP error, DCM bias) norms, m, already lagged by the source

  /** TEMPORARY (step 6 diagnostics, removed in step 9): not used by the builder. */
  struct Diagnostics
  {
    bool has_gyro = false;
    Vec3 gyro{};                      // body sensor angular velocity, if the robot has one
    std::string joint_vel_source;     // where joint_vel came from
    double encoder_vel_max_abs = -1;  // max |encoderVelocities()|, -1 if not available
    double alpha_max_abs = 0;         // max |mbc.alpha| over the contract joints
  } diag;
};

/** Where a RobotState comes from. The only implementation is ControllerAdapter. */
class StateSource
{
public:
  virtual ~StateSource() = default;

  /** Resolves the contract joint names on the robot. False + err when a joint cannot be read. */
  virtual bool bind(const std::vector<std::string> & contract_joints, std::string & err) = 0;

  /**
   * Called once per controller step while a policy is loaded, whether or not an observation is built this step.
   * Lets the source keep the short history it needs to reproduce training's datastore-output lag.
   */
  virtual void sample() noexcept = 0;

  /** Reads the current state. Never throws; false + err when a value is unavailable. */
  virtual bool read(RobotState & out, std::string & err) noexcept = 0;
};

} // namespace ismpc_walking::policy
