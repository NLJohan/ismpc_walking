#pragma once
#include <mc_control/mc_controller.h>

struct ControllerConfiguration
{
  double SwingFootWeight = 5000;
  double SwingFootStiffness = 300;

  double controller_timestep = 5e-3;

  double momentumTaskWeight = 10000;

  // range is min max
  Eigen::Vector2d ts_range{0.6, 2};
  Eigen::Vector2d tds_range{0.2, 1.5};
  Eigen::Vector2d tss_range{0.4, 1.5};

  Eigen::Vector2d Footsteps_Generation_Kinematics_cstr{0.3, 0.1};

  double Beta_zmp_vel = 1;
  double Beta_step = 1e4; // 1e4;
  double Beta_stab = 1e7;
  double Beta_zmp_traj = 0.;
  double Beta_zmp_traj_static = 10;
  double Beta_Ld = 1.;
  double Beta_dcm_static = 200;
  double Beta_dcm_vel = 0;
  double Beta_dcm_vel_static = 0;
  double Beta_dcm = 1;
  double lambda_ = 100.;
  double lambda_sg_supp = 100;
  double lambda_dbl_supp = 100;
  double lambda_standing = 100;
  double zmp_delay = 1e-2;
  Eigen::Vector2d Beta_range{1e10, 1e2};
  bool sliding_zmp_cstr_region = false;
  bool use_stability_task = false;
  double footStepHeight = 0.04;
  double maxVelX = 0.15;
  double minVelX = -0.15;
  double delta = 5e-2; // MPC TimeStep;
  double Tc = 1.5; // Control Time
  double Tp = 4; // Preview Time
  double feet_ditance_ = 0.2;
  double foot_landing_offset = 0.;
  double Double_Step_Ratio = 0.25;
  double impact_threshold = 30;
  double max_stability_error = 0.01; // If the stability error is beyond that value, stop will be called
  double max_swing_foot_velocity = 2.5;

  double landing_time = 0.2;

  double MPC_ZMP_Constraint_max_size = 0.2;
  double MPC_ZMP_Constraint_min_size = 0.01;
  Eigen::Vector2d MPC_ZMP_Constraint_size{0.1, 0.08};
  Eigen::Vector2d MPC_U_Constraint_size{0.2, 0.2};
  Eigen::Vector2d MPC_ZMP_cstr_square_static = {0.1, 0.1};

  double MPC_ZMP_next_stp_cstr_ratio = 2;
  bool MPC_allow_None = true;
  Eigen::Vector2d MPC_ZMP_cstr_square_offset = Eigen::Vector2d::Zero();
  Eigen::Vector2d MPC_ZMP_static_cstr_square_offset = Eigen::Vector2d::Zero();
  Eigen::Vector2d MPC_ZMP_ref_offset_sg_supp = Eigen::Vector2d::Zero();
  Eigen::Vector2d MPC_ZMP_ref_offset_end_step = {0.02, 0};
  Eigen::Vector2d MPC_ZMP_ref_offset_start_step = {0.02, 0};
  Eigen::Vector2d MPC_Footsteps_Constraint_size{0.1, 0.1};
  Eigen::Vector2d MPC_Footsteps_kin_Constraint_size{0.1, 0.1};

  bool Stabilizer_dcm_bias_estimator = true;
  bool Stabilizer_dcm_filter = true;
  sva::MotionVecd CoPMaxVel = sva::MotionVecd(Eigen::Vector3d{1.5, 1.5, 0.3}, Eigen::Vector3d{.5, .5, 0.3});

  double wrench_filter_cutoff;
  double gamma_filter_cutoff;
  double safety_roll_error_ = 0.1;

  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration stab_config;
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration stab_config_standing;
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration stab_config_sg_supp;
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration stab_config_dbl_supp;

  double Ts_ = 0.6;
  double Ls_ = 0.15;
  double alpha = 0.4; // Average Step duration/lentgh/speed (To process the timestamp)
                      //
  // Low-pass filter frequency for external disturbance
  bool with_external_disturbance_highpass_filter_ = true;
  double external_disturbance_cutoff_period_ = 1.0;
};
