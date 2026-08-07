#pragma once
#include "Polygon.h"
#include <eigen3/Eigen/Dense>
#include <vector>

struct MPC_state
{

  Eigen::Vector3d Get_CoM_planarTarget(const size_t indx)
  {
    if(indx < X_MPC.size())
    {
      // Time-Varying Fix: z used to be hardcoded to 0 here, silently discarding any
      // time-varying CoM height trajectory computed by the solver. CoM_height must be
      // populated (same indexing/size as X_MPC/Y_MPC) by the controller from
      // ISMPC_Solver::CoM_height_vec() every time X_MPC/Y_MPC are refreshed.
      const double z = (indx < CoM_height.size()) ? CoM_height[indx] : default_CoM_height;
      return Eigen::Vector3d{X_MPC[indx][0], Y_MPC[indx][0], z};
    }
    std::cout << "[CoM access] Warning wrong index returning 0 vector" << std::endl;
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d Get_CoMVel_planarTarget(const size_t indx)
  {
    if(indx < X_MPC.size())
    {
      // Time-Varying Fix (CoM-height feedforward): z used to be hardcoded to 0
      // here, discarding the solver's analytic zc_dot feedforward the same way
      // Get_CoM_planarTarget's z was hardcoded before the earlier Time-Varying
      // Fix. CoM_height_vel must be populated (same indexing/size as X_MPC/
      // Y_MPC/CoM_height) by the controller from ISMPC_Solver::CoM_height_vel_vec()
      // every time X_MPC/Y_MPC are refreshed -- see that accessor's own doc
      // comment for which signal cases actually populate it (empty otherwise,
      // hence the fallback to 0 here rather than an out-of-range read).
      const double zdot = (indx < CoM_height_vel.size()) ? CoM_height_vel[indx] : 0.0;
      return Eigen::Vector3d{X_MPC[indx][1], Y_MPC[indx][1], zdot};
    }
    std::cout << "[CoMd access] Warning wrong index returning 0 vector" << std::endl;
    return Eigen::Vector3d::Zero();
  }

  /**
   * Analogous to Get_CoMVel_planarTarget, but for the analytic CoM-height
   * ACCELERATION feedforward (CoM_height_acc, from
   * ISMPC_Solver::CoM_height_acc_vec()). Only the z-component is meaningful
   * here (unlike Get_CoM_planarTarget/Get_CoMVel_planarTarget, there is no
   * existing planar-XY acceleration signal in X_MPC/Y_MPC to pair it with,
   * so this returns z alone rather than a padded/misleading Vector3d) --
   * callers wanting a feedforward acceleration term should read this
   * directly into the z-component of their own acc_com vector, not treat
   * the return value as a drop-in replacement for a planar acceleration.
   */
  double Get_CoMHeightAccel_target(const size_t indx)
  {
    if(indx < CoM_height_acc.size())
    {
      return CoM_height_acc[indx];
    }
    return 0.0;
  }

  Eigen::Vector3d Get_ZMP_planarTarget(const size_t indx)
  {
    if(indx < X_MPC.size())
    {
      return Eigen::Vector3d{X_MPC[indx][2], Y_MPC[indx][2], 0};
    }
    std::cout << "[ZMP access] Warning wrong index returning 0 vector" << std::endl;
    return Eigen::Vector3d::Zero();
  }

  std::vector<Eigen::Vector2d> zmp_references()
  {
    std::vector<Eigen::Vector2d> output;
    // for (int indx = Index ; indx < X_MPC.size() ; indx++)
    // {
    //   output.push_back(Eigen::Vector2d{X_MPC[indx][2], Y_MPC[indx][2]});
    // }
    for(auto & ref : QP_zmp)
    {
      output.push_back(ref.segment(0, 2));
    }
    return output;
  }

  sva::PTransformd & Get_CorrectedFootstep(int indx)
  {

    return optimal_steps_[indx];
  }

  sva::PTransformd & Get_PlannedFootstep(int indx)
  {

    return planned_steps_[indx];
  }

  const std::vector<sva::PTransformd> & planned_steps() const noexcept
  {
    return planned_steps_;
  }

  const std::vector<sva::PTransformd> & optimal_steps() const noexcept
  {
    return optimal_steps_;
  }

  std::vector<Eigen::Vector2d> admittance_references()
  {
    std::vector<Eigen::Vector2d> output;
    for(auto & ref : admittance_ref_)
    {
      output.push_back(ref.segment(0, 2));
    }
    return output;
  }

  const std::vector<Eigen::Vector3d> & get_SupPolygon()
  {
    return SupPolygon;
  }
  const Eigen::VectorXd & get_Trajant()
  {
    return Traj_ant;
  }
  const std::vector<Eigen::Vector3d> & get_RefTraj()
  {
    return P_traj;
  }

  const std::vector<double> & getTimeStamp()
  {
    return optimal_timesteps_;
  }
  double get_Ts(size_t indx)
  {
    if(indx < optimal_timesteps_.size())
    {
      return optimal_timesteps_[indx];
    }
    return optimal_tds + 1;
    ;
  }
  double get_tds()
  {
    return optimal_tds;
  }
  void set_input_tds(double t)
  {
    input_tds = t;
  }

  const Eigen::Vector3d & getPzk()
  {
    return p_z_k;
  }
  const Eigen::Vector3d & getPck()
  {
    return p_c_k;
  }
  const Eigen::Vector3d & getVck()
  {
    return v_c_k;
  }
  Eigen::Vector3d getPuk()
  {
    return p_c_k + v_c_k / eta;
  }

  /**
   * Returns the pendulum frequency eta applicable at horizon index indx.
   * Falls back to the scalar `eta` member (current-step value) if eta_vec is empty
   * or indx is out of range, e.g. before the first MPC solve has populated it.
   */
  double getEta(size_t indx) const
  {
    if(indx < eta_vec.size())
    {
      return eta_vec[indx];
    }
    return eta;
  }

  Eigen::Vector3d get_u(int indx)
  {
    if(indx / 2 >= mpc_u_.size())
    {
      std::cout << "[U access] Warning wrong index returning 0 vector" << std::endl;
      return Eigen::Vector3d::Zero();
    }
    double horizon_size = static_cast<double>(mpc_u_.size()) / 2;
    return Eigen::Vector3d{mpc_u_[indx], mpc_u_[indx + static_cast<int>(horizon_size)], 0.};
  }

  Eigen::Vector3d get_Lc_dot(int indx)
  {
    if(indx / 2 >= mpc_Lc_dot_.size())
    {
      std::cout << "[mpc_Lc_dot_ access] Warning wrong index returning 0 vector" << std::endl;
      return Eigen::Vector3d::Zero();
    }
    double horizon_size = static_cast<double>(mpc_Lc_dot_.size()) / 2;
    return Eigen::Vector3d{mpc_Lc_dot_[indx], mpc_Lc_dot_[indx + static_cast<int>(horizon_size)], 0.};
  }

  std::vector<Eigen::Vector3d>
      X_MPC; // Contain 3d vectors that represents in that order the CoM the CoMd and the ZMP for each timestep
  std::vector<Eigen::Vector3d>
      Y_MPC; // Contain 3d vectors that represents in that order the CoM the CoMd and the ZMP for each timestep
  // Time-Varying Fix: per-horizon-step CoM height z_c(t), same indexing/size as X_MPC/Y_MPC.
  // Must be populated from ISMPC_Solver::CoM_height_vec() every time X_MPC/Y_MPC are refreshed.
  std::vector<double> CoM_height;
  double default_CoM_height = 0.78; // fallback only, used if CoM_height is unpopulated/out of range
  // Time-Varying Fix (CoM-height feedforward): analytic first/second time-
  // derivatives of CoM_height, same indexing/size as CoM_height/X_MPC/Y_MPC.
  // Must be populated from ISMPC_Solver::CoM_height_vel_vec()/
  // CoM_height_acc_vec() every time X_MPC/Y_MPC are refreshed -- empty for
  // signal cases that don't compute a genuine closed-form derivative (see
  // those accessors' doc comments in ISMPC_Solver.h). Consumed by
  // Get_CoMVel_planarTarget's z-component and Get_CoMHeightAccel_target.
  std::vector<double> CoM_height_vel;
  std::vector<double> CoM_height_acc;
  // Time-Varying Fix: per-horizon-step pendulum frequency, same indexing/size as X_MPC/Y_MPC.
  // Must be populated from ISMPC_Solver::eta_vec() every time X_MPC/Y_MPC are refreshed.
  std::vector<double> eta_vec;
  std::vector<Eigen::Vector3d> SupPolygon;
  std::vector<Eigen::Vector3d> FeasibilityPolygon;
  SupportPolygon FeasibilityPolygonStandingSwitch; // standing feasibility region if support foot is switched
  Eigen::VectorXd Traj_ant;
  std::vector<Eigen::Vector3d> P_traj; // Vector containing the reference trajectory
  bool Tail = true;
  int kfoot = 0;
  Eigen::Vector2d stab_error;
  bool QPSuccess;
  Eigen::Vector2d Pu_min;
  Eigen::Vector2d Pu_max;
  double alpha = 0;
  Eigen::Vector2d ref_zmp_ = Eigen::Vector2d::Zero();
  int Index = 0;

  Eigen::Vector3d initial_zmp_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d delayed_zmp_ = Eigen::Vector3d::Zero();

  double t_k = 0;
  double t;
  double t_lift = 0;
  Eigen::Vector3d p_c_k = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_c_k = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_z_k = Eigen::Vector3d::Zero();
  Eigen::Vector3d Uk = Eigen::Vector3d::Zero();
  Eigen::Vector3d Lck = Eigen::Vector3d::Zero();
  Eigen::Vector3d ComBias = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> admittance_ref_;
  Eigen::Vector3d p_u = Eigen::Vector3d::Zero();
  Eigen::Vector3d w; // Perturbation
  double kappa = 1;
  Eigen::VectorXd mpc_u_;
  Eigen::VectorXd mpc_Lc_dot_;
  std::vector<Eigen::Vector3d> QP_zmp;
  std::vector<Eigen::Vector3d> QP_dcm;
  std::vector<sva::MotionVecd> input_v_;
  double input_eta = 3.5;
  double input_mass = 40;
  double eta = 3.5;
  std::vector<sva::PTransformd> input_ref_pose_; // planner reference steps
  std::vector<sva::PTransformd> planned_steps_;
  std::vector<double> input_timesteps_; // Input desired steps timings
  std::vector<double> planned_timesteps_; // planner reference timesteps
  std::vector<sva::PTransformd> optimal_steps_; // Outputs steps from the
  std::vector<double> optimal_timesteps_; // Outputs timesteps from the mpc
  std::string input_Support_FootName;
  sva::PTransformd X_0_SupportFoot = sva::PTransformd::Identity();
  sva::PTransformd X_0_Initial_SwingFoot = sva::PTransformd::Identity();
  sva::PTransformd X_0_SwingFoot = sva::PTransformd::Identity();
  sva::PTransformd X_0_Step_Target = sva::PTransformd::Identity();

  double tds = 0.25;
  double input_tds = 0.25;
  double optimal_tds = 0.25;
  bool stop = true;
  bool standing_mode = true;
  bool doubleSupport = true;

  /**
 * @brief Reset all solve-derived/horizon state to its "no MPC solve has
 * happened yet" values, without touching persistent-but-not-solve-derived
 * fields (default_CoM_height, input_eta, input_mass, eta, kappa, tds,
 * input_tds, optimal_tds -- these are configuration/tuning values, not
 * outputs of a previous solve, and keeping them is correct/intended).
 *
 * Call this on BOTH mpc_state_ and mpc_thread_state at the start of a fresh
 * episode (see Walking_controller::reset()) -- leaving either one stale
 * lets UpdateInitialVectors()'s `X_MPC.size() != 0` branch read leftover
 * horizon data from the previous episode's last solve, which is exactly
 * the mechanism traced down in this project's reset-state investigation
 * (a stale, non-empty X_MPC/Index right after reset silently overwrites
 * the freshly-reset p_c_k with a stale trajectory sample).
 */
void ClearSolveState()
{
  X_MPC.clear();
  Y_MPC.clear();
  CoM_height.clear();
  CoM_height_vel.clear();
  CoM_height_acc.clear();
  eta_vec.clear();
  SupPolygon.clear();
  FeasibilityPolygon.clear();
  FeasibilityPolygonStandingSwitch = SupportPolygon();
  Traj_ant = Eigen::VectorXd();
  P_traj.clear();
  Tail = true;
  kfoot = 0;
  stab_error.setZero();
  QPSuccess = false;
  Pu_min.setZero();
  Pu_max.setZero();
  alpha = 0;
  ref_zmp_.setZero();
  Index = 0;
  initial_zmp_.setZero();
  delayed_zmp_.setZero();
  t_k = 0;
  t_lift = 0;
  p_c_k.setZero();
  v_c_k.setZero();
  p_z_k.setZero();
  Uk.setZero();
  Lck.setZero();
  ComBias.setZero();
  admittance_ref_.clear();
  p_u.setZero();
  w.setZero();
  mpc_u_.resize(0);
  mpc_Lc_dot_.resize(0);
  QP_zmp.clear();
  QP_dcm.clear();
  input_v_.clear();
  input_ref_pose_.clear();
  planned_steps_.clear();
  input_timesteps_.clear();
  planned_timesteps_.clear();
  optimal_steps_.clear();
  optimal_timesteps_.clear();
  input_Support_FootName.clear();
  X_0_SupportFoot = sva::PTransformd::Identity();
  X_0_Initial_SwingFoot = sva::PTransformd::Identity();
  X_0_SwingFoot = sva::PTransformd::Identity();
  X_0_Step_Target = sva::PTransformd::Identity();
  stop = true;
  standing_mode = true;
  doubleSupport = true;
  }
};