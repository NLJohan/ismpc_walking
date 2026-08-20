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
      const double z = (indx < CoM_height_fine.size()) ? CoM_height_fine[indx] : default_CoM_height;
      return Eigen::Vector3d{X_MPC[indx][0], Y_MPC[indx][0], z};
    }
    std::cout << "[CoM access] Warning wrong index returning 0 vector" << std::endl;
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d Get_CoMVel_planarTarget(const size_t indx)
  {
    if(indx < X_MPC.size())
    {
      const double zdot = (indx < CoM_height_vel_fine.size()) ? CoM_height_vel_fine[indx] : 0.0;
      return Eigen::Vector3d{X_MPC[indx][1], Y_MPC[indx][1], zdot};
    }
    std::cout << "[CoMd access] Warning wrong index returning 0 vector" << std::endl;
    return Eigen::Vector3d::Zero();
  }

  double Get_CoMHeightAccel_target(const size_t indx)
  {
    if(indx < CoM_height_acc_fine.size())
    {
      return CoM_height_acc_fine[indx];
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
  // Fine-resolution (X_MPC/Y_MPC-matching, m_delta_control-spaced) CoM-height
  // reference, for smooth task-target consumption via Get_CoM_planarTarget/
  // Get_CoMVel_planarTarget/Get_CoMHeightAccel_target below. Populated
  // ADDITIONALLY to (not instead of) CoM_height/CoM_height_vel/CoM_height_acc
  // above -- those coarse arrays are unrelated to this change and continue to
  // exclusively feed ISMPC_Solver's m_eta/Integrate()/Compute_Riccati_Kernel(),
  // completely untouched by the fine-resolution addition. Must be populated
  // from ISMPC_Solver::CoM_height_fine_vec()/CoM_height_vel_fine_vec()/
  // CoM_height_acc_fine_vec() every time X_MPC/Y_MPC are refreshed -- same
  // indexing/size as X_MPC/Y_MPC directly (no ratio/mapping needed, unlike the
  // coarse arrays' previous com_height_substep_ratio-based mapping, now
  // retired since it's no longer needed).
  std::vector<double> CoM_height_fine;
  std::vector<double> CoM_height_vel_fine;
  std::vector<double> CoM_height_acc_fine;
  // Time-Varying Fix: per-horizon-step pendulum frequency, same indexing/size as X_MPC/Y_MPC.  
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

  // Add this method inside struct MPC_state (MPC_state.h), e.g. right after
  // ClearSolveState(). Call as mpc_state_.DumpState("some_tag") and
  // mpc_thread_state.DumpState("some_tag_thread") at the same points
  // Walking_controller::reset() already calls MPCSolver.DumpState(...) --
  // see the reset_enter_prevCount*/reset_after_ResetEpisodeState tag pair.
  //
  // Covers every member declared in MPC_state.h. Vector/container fields print
  // size plus front/back (or a representative element) rather than full
  // contents, mirroring the horizonZmpRef_ front/back convention already used
  // elsewhere in this investigation -- size alone would have hidden the
  // horizonZmpRef_ staleness bug found earlier this session, so front/back is
  // treated as the minimum useful signal for any container here.

  void DumpState(const std::string & tag)
  {
    // --- Scalars, in declaration order. ---
    mc_rtc::log::warning("[MPC_state][DumpState][{}] Tail={}", tag, Tail);
    mc_rtc::log::warning("[MPC_state][DumpState][{}] kfoot={}", tag, kfoot);
    mc_rtc::log::warning("[MPC_state][DumpState][{}] stab_error=({},{})", tag, stab_error.x(), stab_error.y());
    mc_rtc::log::warning("[MPC_state][DumpState][{}] QPSuccess={}", tag, QPSuccess);
    mc_rtc::log::warning("[MPC_state][DumpState][{}] Pu_min=({},{})", tag, Pu_min.x(), Pu_min.y());
    mc_rtc::log::warning("[MPC_state][DumpState][{}] Pu_max=({},{})", tag, Pu_max.x(), Pu_max.y());
    mc_rtc::log::warning("[MPC_state][DumpState][{}] alpha={}", tag, alpha);
    mc_rtc::log::warning("[MPC_state][DumpState][{}] ref_zmp_=({},{})", tag, ref_zmp_.x(), ref_zmp_.y());
    mc_rtc::log::warning("[MPC_state][DumpState][{}] Index={}", tag, Index);
    mc_rtc::log::warning("[MPC_state][DumpState][{}] initial_zmp_=({},{},{})", tag, initial_zmp_.x(), initial_zmp_.y(),
                          initial_zmp_.z());
    mc_rtc::log::warning("[MPC_state][DumpState][{}] delayed_zmp_=({},{},{})", tag, delayed_zmp_.x(), delayed_zmp_.y(),
                          delayed_zmp_.z());
    mc_rtc::log::warning("[MPC_state][DumpState][{}] t_k={}", tag, t_k);
    // t: NOT touched by ClearSolveState() and NOT mentioned in its doc comment
    // (unlike every other solve-derived field) -- also has no default member
    // initializer, unlike almost everything else in this struct. Flagged this
    // session as a likely-missed reset target; dumped here specifically to
    // check whether it carries a stale/garbage value across episode
    // boundaries the same way t_k/NextOptimalTs did.
    mc_rtc::log::warning("[MPC_state][DumpState][{}] t={} (NOT cleared by ClearSolveState -- check for staleness)",
                          tag, t);
    mc_rtc::log::warning("[MPC_state][DumpState][{}] t_lift={}", tag, t_lift);
    mc_rtc::log::warning("[MPC_state][DumpState][{}] p_c_k=({},{},{})", tag, p_c_k.x(), p_c_k.y(), p_c_k.z());
    mc_rtc::log::warning("[MPC_state][DumpState][{}] v_c_k=({},{},{})", tag, v_c_k.x(), v_c_k.y(), v_c_k.z());
    mc_rtc::log::warning("[MPC_state][DumpState][{}] p_z_k=({},{},{})", tag, p_z_k.x(), p_z_k.y(), p_z_k.z());
    mc_rtc::log::warning("[MPC_state][DumpState][{}] Uk=({},{},{})", tag, Uk.x(), Uk.y(), Uk.z());
    mc_rtc::log::warning("[MPC_state][DumpState][{}] Lck=({},{},{})", tag, Lck.x(), Lck.y(), Lck.z());
    mc_rtc::log::warning("[MPC_state][DumpState][{}] ComBias=({},{},{})", tag, ComBias.x(), ComBias.y(), ComBias.z());
    mc_rtc::log::warning("[MPC_state][DumpState][{}] p_u=({},{},{})", tag, p_u.x(), p_u.y(), p_u.z());
    mc_rtc::log::warning("[MPC_state][DumpState][{}] w=({},{},{})", tag, w.x(), w.y(), w.z());
    mc_rtc::log::warning("[MPC_state][DumpState][{}] kappa={}", tag, kappa);
    mc_rtc::log::warning("[MPC_state][DumpState][{}] input_eta={}", tag, input_eta);
    mc_rtc::log::warning("[MPC_state][DumpState][{}] input_mass={}", tag, input_mass);
    mc_rtc::log::warning("[MPC_state][DumpState][{}] eta={}", tag, eta);
    mc_rtc::log::warning("[MPC_state][DumpState][{}] input_Support_FootName={}", tag, input_Support_FootName);
    mc_rtc::log::warning("[MPC_state][DumpState][{}] X_0_SupportFoot.translation()=({},{},{})", tag,
                          X_0_SupportFoot.translation().x(), X_0_SupportFoot.translation().y(),
                          X_0_SupportFoot.translation().z());
    mc_rtc::log::warning("[MPC_state][DumpState][{}] X_0_Initial_SwingFoot.translation()=({},{},{})", tag,
                          X_0_Initial_SwingFoot.translation().x(), X_0_Initial_SwingFoot.translation().y(),
                          X_0_Initial_SwingFoot.translation().z());
    mc_rtc::log::warning("[MPC_state][DumpState][{}] X_0_SwingFoot.translation()=({},{},{})", tag,
                          X_0_SwingFoot.translation().x(), X_0_SwingFoot.translation().y(),
                          X_0_SwingFoot.translation().z());
    mc_rtc::log::warning("[MPC_state][DumpState][{}] X_0_Step_Target.translation()=({},{},{})", tag,
                          X_0_Step_Target.translation().x(), X_0_Step_Target.translation().y(),
                          X_0_Step_Target.translation().z());
    mc_rtc::log::warning("[MPC_state][DumpState][{}] tds={}", tag, tds);
    mc_rtc::log::warning("[MPC_state][DumpState][{}] input_tds={}", tag, input_tds);
    mc_rtc::log::warning("[MPC_state][DumpState][{}] optimal_tds={}", tag, optimal_tds);
    mc_rtc::log::warning("[MPC_state][DumpState][{}] stop={}", tag, stop);
    mc_rtc::log::warning("[MPC_state][DumpState][{}] standing_mode={}", tag, standing_mode);
    mc_rtc::log::warning("[MPC_state][DumpState][{}] doubleSupport={}", tag, doubleSupport);
    mc_rtc::log::warning("[MPC_state][DumpState][{}] default_CoM_height={}", tag, default_CoM_height);

    // --- Containers: size + front/back (or a representative element), not
    // full contents. Empty containers print a single "(empty)" line so an
    // unexpectedly-nonempty one stands out clearly in a diff. ---
    auto dump_vec3_container = [&](const char * name, const std::vector<Eigen::Vector3d> & v) {
      if(v.empty())
      {
        mc_rtc::log::warning("[MPC_state][DumpState][{}] {}.size()=0 (empty)", tag, name);
      }
      else
      {
        mc_rtc::log::warning("[MPC_state][DumpState][{}] {}.size()={} front=({},{},{}) back=({},{},{})", tag, name,
                              v.size(), v.front().x(), v.front().y(), v.front().z(), v.back().x(), v.back().y(),
                              v.back().z());
      }
    };
    auto dump_double_container = [&](const char * name, const std::vector<double> & v) {
      if(v.empty())
      {
        mc_rtc::log::warning("[MPC_state][DumpState][{}] {}.size()=0 (empty)", tag, name);
      }
      else
      {
        mc_rtc::log::warning("[MPC_state][DumpState][{}] {}.size()={} front={} back={}", tag, name, v.size(),
                              v.front(), v.back());
      }
    };
    auto dump_pose_container = [&](const char * name, const std::vector<sva::PTransformd> & v) {
      if(v.empty())
      {
        mc_rtc::log::warning("[MPC_state][DumpState][{}] {}.size()=0 (empty)", tag, name);
      }
      else
      {
        mc_rtc::log::warning(
            "[MPC_state][DumpState][{}] {}.size()={} front.translation()=({},{},{}) back.translation()=({},{},{})",
            tag, name, v.size(), v.front().translation().x(), v.front().translation().y(),
            v.front().translation().z(), v.back().translation().x(), v.back().translation().y(),
            v.back().translation().z());
      }
    };

    dump_vec3_container("X_MPC", X_MPC);
    dump_vec3_container("Y_MPC", Y_MPC);
    dump_double_container("CoM_height", CoM_height);
    dump_double_container("CoM_height_vel", CoM_height_vel);
    dump_double_container("CoM_height_acc", CoM_height_acc);
    dump_double_container("CoM_height_fine", CoM_height_fine);
    dump_double_container("CoM_height_vel_fine", CoM_height_vel_fine);
    dump_double_container("CoM_height_acc_fine", CoM_height_acc_fine);
    dump_double_container("eta_vec", eta_vec);
    dump_vec3_container("SupPolygon", SupPolygon);
    dump_vec3_container("FeasibilityPolygon", FeasibilityPolygon);
    // FeasibilityPolygonStandingSwitch: SupportPolygon type, not a plain
    // vector -- dumped as size only unless SupportPolygon exposes something
    // richer; check its own header if this needs more than size.
    mc_rtc::log::warning("[MPC_state][DumpState][{}] FeasibilityPolygonStandingSwitch=<SupportPolygon, see its own "
                          "type for a richer dump if needed>",
                          tag);
    if(Traj_ant.size() == 0)
    {
      mc_rtc::log::warning("[MPC_state][DumpState][{}] Traj_ant.size()=0 (empty)", tag);
    }
    else
    {
      mc_rtc::log::warning("[MPC_state][DumpState][{}] Traj_ant.size()={} front={} back={}", tag, Traj_ant.size(),
                            Traj_ant(0), Traj_ant(Traj_ant.size() - 1));
    }
    dump_vec3_container("P_traj", P_traj);
    dump_vec3_container("admittance_ref_", admittance_ref_);
    if(mpc_u_.size() == 0)
    {
      mc_rtc::log::warning("[MPC_state][DumpState][{}] mpc_u_.size()=0 (empty)", tag);
    }
    else
    {
      mc_rtc::log::warning("[MPC_state][DumpState][{}] mpc_u_.size()={} front={} back={}", tag, mpc_u_.size(),
                            mpc_u_(0), mpc_u_(mpc_u_.size() - 1));
    }
    if(mpc_Lc_dot_.size() == 0)
    {
      mc_rtc::log::warning("[MPC_state][DumpState][{}] mpc_Lc_dot_.size()=0 (empty)", tag);
    }
    else
    {
      mc_rtc::log::warning("[MPC_state][DumpState][{}] mpc_Lc_dot_.size()={} front={} back={}", tag,
                            mpc_Lc_dot_.size(), mpc_Lc_dot_(0), mpc_Lc_dot_(mpc_Lc_dot_.size() - 1));
    }
    dump_vec3_container("QP_zmp", QP_zmp);
    dump_vec3_container("QP_dcm", QP_dcm);
    // input_v_: std::vector<sva::MotionVecd>, not ForceVecd/PTransformd/double
    // -- dumped as size only for now; add a MotionVecd-specific lambda if
    // front/back content turns out to matter here too.
    mc_rtc::log::warning("[MPC_state][DumpState][{}] input_v_.size()={}", tag, input_v_.size());
    dump_pose_container("input_ref_pose_", input_ref_pose_);
    dump_pose_container("planned_steps_", planned_steps_);
    dump_double_container("input_timesteps_", input_timesteps_);
    dump_double_container("planned_timesteps_", planned_timesteps_);
    dump_pose_container("optimal_steps_", optimal_steps_);
    dump_double_container("optimal_timesteps_", optimal_timesteps_);
  }

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
  CoM_height_fine.clear();
  CoM_height_vel_fine.clear();
  CoM_height_acc_fine.clear();
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