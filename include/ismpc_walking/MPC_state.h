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
      return Eigen::Vector3d{X_MPC[indx][1], Y_MPC[indx][1], 0};
    }
    std::cout << "[CoMd access] Warning wrong index returning 0 vector" << std::endl;
    return Eigen::Vector3d::Zero();
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
};