#pragma once
#include <mc_control/api.h>
#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>
#include <mc_observers/KinematicInertialObserver.h>
#include <mc_observers/ObserverPipeline.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/clock.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/CoPTask.h>
#include <mc_tasks/MomentumTask.h>
#include <mc_tasks/SurfaceTransformTask.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>
#include <Tasks/QPContactConstr.h>
#include "ControllerConfiguration.h"
#include "FootTrajectory.h"
#include "ISMPC_Solver.h"
#include "MPC_state.h"
#include "api.h"
#include "eigen-quadprog/QuadProg.h"
#include "eigen-quadprog/eigen_quadprog_api.h"
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <unistd.h>

enum class StabilizerState
{
  Standing,
  DoubleSupport,
  SingleSupport,
  None
};

struct Walking_controller_DLLAPI Walking_controller : public mc_control::fsm::Controller
{
public:
  Walking_controller(mc_rbdyn::RobotModulePtr rm,
                     double dt,
                     const mc_rtc::Configuration & config,
                     mc_control::ControllerParameters params = {});

  ~Walking_controller() override;

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  ControllerConfiguration controller_config_;
  mc_rtc::Configuration planner_config_;

  void Configure(const mc_rtc::Configuration & config)
  {

    controller_config_.Beta_zmp_vel = config("ismpc")("beta_zmp_vel");
    controller_config_.Beta_step = config("ismpc")("beta_step");
    controller_config_.Beta_range = config("ismpc")("safety_thresholds")("beta_range");
    controller_config_.MPC_ZMP_Constraint_min_size = config("ismpc")("safety_thresholds")("zmp_cstr_square_min");
    controller_config_.MPC_ZMP_Constraint_max_size = config("ismpc")("safety_thresholds")("zmp_cstr_square_max");
    controller_config_.MPC_Footsteps_Constraint_size = config("ismpc")("footsteps_cstr_square");
    controller_config_.MPC_Footsteps_kin_Constraint_size = config("ismpc")("foosteps_kin_cstr");
    controller_config_.Double_Step_Ratio = config("ismpc")("double_support_ratio");
    controller_config_.sliding_zmp_cstr_region = config("ismpc")("sliding_zmp_cstr_region");
    controller_config_.delta = config("ismpc")("delta");
    controller_config_.MPC_ZMP_next_stp_cstr_ratio = config("ismpc")("next_stp_cstr_ratio");
    controller_config_.MPC_ZMP_cstr_square_offset = config("ismpc")("offset");
    controller_config_.MPC_ZMP_static_cstr_square_offset = config("ismpc")("offset_static");
    controller_config_.MPC_ZMP_ref_offset_sg_supp = config("ismpc")("zmp_ref_offset");
    controller_config_.MPC_ZMP_Constraint_size = config("ismpc")("zmp_cstr_square");
    controller_config_.MPC_U_Constraint_size = config("ismpc")("u_cstr_square");
    controller_config_.MPC_ZMP_cstr_square_static = config("ismpc")("zmp_cstr_square_static");
    controller_config_.MPC_allow_None = config("ismpc")("allow_none_tail");
    controller_config_.Tc = config("ismpc")("Tc");
    controller_config_.delta = config("ismpc")("delta");
    controller_config_.use_stability_task = config("ismpc")("use_stability_task");
    controller_config_.Beta_stab = config("ismpc")("beta_stab");
    controller_config_.Beta_zmp_traj = config("ismpc")("beta_traj");
    controller_config_.lambda_sg_supp = config("ismpc")("lambda");
    controller_config_.lambda_dbl_supp = controller_config_.lambda_sg_supp;
    if(config("ismpc").has("lambda_dbl_supp"))
    {
      controller_config_.lambda_dbl_supp = config("ismpc")("lambda_dbl_supp");
    }
    if(config("ismpc").has("beta_Ld"))
    {
      controller_config_.Beta_Ld = config("ismpc")("beta_Ld");
    }
    if(config("ismpc").has("beta_dcm"))
    {
      controller_config_.Beta_dcm = config("ismpc")("beta_dcm");
    }
    if(config("ismpc").has("beta_dcm_static"))
    {
      controller_config_.Beta_dcm_static = config("ismpc")("beta_dcm_static");
    }
    if(config("ismpc").has("ts"))
    {
      ts(config("ismpc")("ts"));
    }
    if(config("ismpc").has("tds_ratio"))
    {
      controller_config_.Double_Step_Ratio = config("ismpc")("tds_ratio");
    }
    if(config("ismpc").has("zmp_ref_start_step"))
    {
      controller_config_.MPC_ZMP_ref_offset_start_step = config("ismpc")("zmp_ref_start_step");
    }
    if(config("ismpc").has("zmp_ref_end_step"))
    {
      controller_config_.MPC_ZMP_ref_offset_end_step = config("ismpc")("zmp_ref_end_step");
    }
    controller_config_.zmp_delay = config("ismpc")("zmp_delay");
    controller_config_.zmp_delay -= std::fmod(controller_config_.zmp_delay, controller_config_.controller_timestep);
    controller_config_.feet_ditance_ = config("ismpc")("feet_distance");
    config("ismpc")("with_external_disturbance_highpass_filter",
                    controller_config_.with_external_disturbance_highpass_filter_);
    config("ismpc")("external_disturbance_cutoff_frequency", controller_config_.external_disturbance_cutoff_period_);

    if(config("walking_controller").has("foot_landing_offset"))
    {
      controller_config_.foot_landing_offset = config("walking_controller")("foot_landing_offset");
    }
    if(config("walking_controller").has("stability_error_threshold"))
    {
      controller_config_.max_stability_error = config("walking_controller")("stability_error_threshold");
    }
    if(config("walking_controller").has("max_swing_foot_velocity"))
    {
      controller_config_.max_swing_foot_velocity = config("walking_controller")("max_swing_foot_velocity");
    }

    controller_config_.ts_range = config("ismpc")("ts_range");
    controller_config_.tss_range = config("ismpc")("tss_range");
    controller_config_.tds_range = config("ismpc")("tds_range");
    controller_config_.impact_threshold = config("walking_controller")("impact_threshold");
    controller_config_.safety_roll_error_ = config("walking_controller")("safety_foot_roll_error");
    controller_config_.wrench_filter_cutoff = config("walking_controller")("wrench_filter_cutoff_T");
    controller_config_.gamma_filter_cutoff = config("walking_controller")("gamma_filter_cutoff_T");
    controller_config_.footStepHeight = config("walking_controller")("footstep_height");

    controller_config_.SwingFootStiffness = config("tasks")("swingfoot_stiffness");
    controller_config_.SwingFootWeight = config("tasks")("swingfoot_weight");
    if(config("tasks").has("momentum_task_weight"))
    {
      controller_config_.momentumTaskWeight = config("tasks")("momentum_task_weight");
    }

    Configure(controller_config_);
  }

  void reconfigure(const mc_rtc::Configuration & config)
  {

    std::vector<double> qp_weight = config("QP Weight (zmp_vel ; step ; zmp traj ; stab ; Ld ; dcm ; dcm_vel)");
    controller_config_.Beta_zmp_vel = qp_weight[0];
    controller_config_.Beta_step = qp_weight[1];
    controller_config_.Beta_zmp_traj = qp_weight[2];
    controller_config_.Beta_stab = qp_weight[3];
    controller_config_.Beta_Ld = qp_weight[4];
    controller_config_.Beta_dcm = qp_weight[5];
    controller_config_.Beta_dcm_vel = qp_weight[6];
    std::vector<double> static_weight = config("Beta dcm static (pos , vel)");
    controller_config_.Beta_dcm_static = static_weight[0];
    controller_config_.Beta_dcm_vel_static = static_weight[1];
    controller_config_.Beta_zmp_traj_static = config("Beta zmp static (pos)");
    controller_config_.Tc = config("Tc");
    controller_config_.delta = config("delta");
    controller_config_.MPC_Footsteps_kin_Constraint_size = config("step kinematics cstr");
    controller_config_.MPC_ZMP_cstr_square_static = config("zmp cstr square static");
    controller_config_.MPC_ZMP_Constraint_size = config("zmp cstr square");
    controller_config_.MPC_U_Constraint_size = config("u cstr square");
    controller_config_.MPC_ZMP_cstr_square_offset = config("zmp cstr square offset");
    controller_config_.MPC_ZMP_ref_offset_sg_supp = config("zmp ref offset");
    controller_config_.feet_ditance_ = config("feet distance");
    controller_config_.MPC_ZMP_ref_offset_end_step = config("zmp ref end step");
    controller_config_.MPC_ZMP_ref_offset_start_step = config("zmp ref start step");
    controller_config_.external_disturbance_cutoff_period_ = config("External disturbance cutoff period");
    controller_config_.with_external_disturbance_highpass_filter_ = config("Include high frequency hand disturbance");
    Configure(controller_config_);
  }

  void Configure(const ControllerConfiguration & config)
  {
    controller_config_ = config;
    controller_config_.stab_config_sg_supp.copFzLambda = controller_config_.lambda_sg_supp * Eigen::Vector3d::Ones();
    controller_config_.stab_config_dbl_supp.copFzLambda = controller_config_.lambda_dbl_supp * Eigen::Vector3d::Ones();

    controller_config_.Beta_step = std::min(controller_config_.Beta_range(1),
                                            std::max(controller_config_.Beta_range(0), controller_config_.Beta_step));
    controller_config_.MPC_ZMP_Constraint_size.x() = std::min(
        controller_config_.MPC_ZMP_Constraint_max_size,
        std::max(controller_config_.MPC_ZMP_Constraint_min_size, controller_config_.MPC_ZMP_Constraint_size.x()));
    controller_config_.MPC_ZMP_Constraint_size.y() = std::min(
        controller_config_.MPC_ZMP_Constraint_max_size,
        std::max(controller_config_.MPC_ZMP_Constraint_min_size, controller_config_.MPC_ZMP_Constraint_size.y()));
    leftHandDisturbanceFilter_.cutoffPeriod(controller_config_.external_disturbance_cutoff_period_);
    rightHandDisturbanceFilter_.cutoffPeriod(controller_config_.external_disturbance_cutoff_period_);
    MPCSolver.configure(controller_config_);
  }

  bool double_support_state() noexcept
  {
    return doubleSupport_state;
  }
  bool stop_phase() noexcept
  {
    return Stop;
  }
  bool robot_walking() noexcept
  {
    return Robot_Walking;
  }
  void start_stop() noexcept
  {
    if(!(Stop && !stabilizer_active_))
    {
      Stop = !Stop;
    }
  }
  double get_t() noexcept
  {
    return t;
  }
  double next_ts()
  {
    if(mpc_state_.optimal_timesteps_.size() != 0)
    {
      return mpc_state_.get_Ts(0);
    }
    return 0.;
  }
  double tds() noexcept
  {
    return mpc_state_.get_tds();
  }
  void tds(double t_ds)
  {
    input_tds = mc_filter::utils::clampAndWarn(t_ds, controller_config_.tds_range(0), controller_config_.tds_range(1),
                                               "Tds capped");
  }
  double ts() noexcept
  {
    return T_Steps;
  }
  void ts(double ts)
  {
    T_Steps =
        mc_filter::utils::clampAndWarn(ts, controller_config_.ts_range(0), controller_config_.ts_range(1), "Ts capped");
  }
  int n_steps() noexcept
  {
    return N_Steps_Desired_std;
  }
  void n_steps(int steps) noexcept
  {
    N_Steps_Desired_std = steps;
  }
  void SwitchFootSupport_manual()
  {
    if(!Robot_Walking)
    {
      mc_rtc::log::info("[ismpc_walking] switching support foot manually");
      switchFootSupport();
      updateTasks();
      MoveFeet(0);
      // UpdatePlanner_input();
      compute_trajectory_once.notify_all();
    }
  }
  const MPC_state & MPCState()
  {
    return mpc_state_;
  }

  /**
   * @brief Compute lambda such as zmp is under the model
   * v_z = -lambda(z - z_0 - u_0\lambda )
   *
   * @return double
   */
  Eigen::Vector2d estimated_lambda()
  {
    if(mpc_state_.mpc_u_.size() != 0)
    {
      double lambda_x =
          -zmp_vel_.eval().x() / (mpc_state_.p_z_k.x() - mpc_state_.initial_zmp_.x() - mpc_state_.get_u(0).x());
      double lambda_y =
          -zmp_vel_.eval().y() / (mpc_state_.p_z_k.y() - mpc_state_.initial_zmp_.y() - mpc_state_.get_u(0).y());

      return Eigen::Vector2d{lambda_x, lambda_y};
    }
    return Eigen::Vector2d::Zero();
  }

protected:
  void getTransformations();
  sva::ForceVecd compute_momentum_contact_point();
  Eigen::Vector3d computeInSupportFootFlat(const Eigen::Vector3d & t_world);
  Eigen::Vector3d computeVelocityInSupportFoot(const Eigen::Vector3d & v_world);
  sva::ForceVecd measuredContactWrench();
  void computeExternalContact(const std::string & surfaceName,
                              const sva::ForceVecd & surfaceWrenchW,
                              Eigen::Vector3d & pos,
                              Eigen::Vector3d & force,
                              Eigen::Vector3d & moment);
  Eigen::Vector3d computeZMP();
  /**
   * Update the stabilizer task with the ISMPC outputs stored in X_MPC and Y_MPC vectors
   */
  void MoveCoM();
  /**
   * Handle the contact of the foot and the trajectory depending on the planned footsteps
   */
  bool MoveFeet(double t);
  /**
   * Compute the planned footsteps/timings and the CoM/ZMP trajectory
   * Footsteps coordinates are stored in XfCorr/YfCorr/ThetafCorr
   * CoM/ZMP trajectory is stored in X_MPC and Y_MPC vectors that contains for each timestep a 3d vector with in that
   * order the CoM, the CoMd, and the ZMP
   */
  void ComputeWalkingTrajectory();
  void WalkingTrajectoryLoop();
  void switchFootSupport();
  /**
   * Update the values of p_c_k v_c_k and p_z_k from the robot
   */
  void UpdateInitialVectors();
  /**
   * Generate the Reference Velocity for constant inputs throught the preveiw horizon
   */

  void UpdatePlanner_input();

  void CheckStepRecovery();

  void JoystickInputs();

  void updateTasks();

  void addToGUI();

  void add_ISMPC_Config_GUI();

  /**
   * @brief Computes two perturbation set, one from the hands, the other from the swing foot
   *
   * @param offset
   * @param kappa
   * @param offset_inf
   * @param kappa_inf
   */
  void ComputePerturbances(Eigen::Vector3d & offset, double & kappa, Eigen::Vector3d & offset_inf, double & kappa_inf);

  void AddToLog();

  void add_FootSteps_GUI();

  void Stabilizer_GUI(mc_rbdyn::lipm_stabilizer::StabilizerConfiguration & config, std::string name);

  void create_datastore()
  {

    datastore().make_call("ismpc_walking::stop_phase", [this]() -> bool { return Stop; });
    datastore().make_call("ismpc_walking::robot_walking", [this]() -> bool { return Robot_Walking; });
    datastore().make_call("ismpc_walking::double_support", [this]() -> bool { return doubleSupport_state; });
    datastore().make_call("ismpc_walking::start/stop", [this]() {
      if(stabilizer_active_ && Stop)
      {
        compute_trajectory_once.notify_all();
      }
      Stop = !Stop;
    });
    datastore().make_call("ismpc_walking::configure",
                          [this](const ControllerConfiguration & config) { Configure(config); });
    datastore().make_call("ismpc_walking::get_config",
                          [this]() -> ControllerConfiguration & { return controller_config_; });
    datastore().make_call("ismpc_walking::set_com_height", [this](const double & h) { comHeight(h); });
    datastore().make_call("ismpc_walking::set_torso_pitch", [this](const double & h) { torsoPitch(h); });
    datastore().make_call("ismpc_walking::zmp_target", [this]() -> Eigen::Vector3d { return zmpTarget; });
    datastore().make_call("ismpc_walking::dcm_target", [this]() -> Eigen::Vector3d { return dcmTarget; });
    datastore().make_call("ismpc_walking::zmp", [this]() -> Eigen::Vector3d { return stabTask->measuredDCM(); });
    datastore().make_call("ismpc_walking::dcm", [this]() -> Eigen::Vector3d { return stabTask->measuredZMP(); });
    datastore().make_call("ismpc_walking::support_foot_name", [this]() -> std::string { return supportFootName; });
    datastore().make_call("ismpc_walking::swing_foot_name", [this]() -> std::string { return swingFootName; });
    datastore().make_call("ismpc_walking::t", [this]() -> double { return t; });
    datastore().make_call("ismpc_walking::t_lift", [this]() -> double { return t_lift; });
    datastore().make_call("ismpc_walking::t_contact", [this]() -> double { return t_contact; });
    datastore().make_call("ismpc_walking::next_ts", [this]() -> double { return ts(); });
    datastore().make_call("ismpc_walking::get_ts_target", [this]() -> double { return T_Steps; });
    datastore().make_call("ismpc_walking::set_ts", [this](double t) { return ts(t); });
    datastore().make_call("ismpc_walking::set_tds", [this](double t) { return tds(t); });
    datastore().make_call("ismpc_walking::get_tds", [this]() -> double { return input_tds; });
    datastore().make_call("ismpc_walking::set_n_step", [this](int n) { N_Steps_Desired_std = n; });
    datastore().make_call("ismpc_walking::get_ref_vel",
                          [this]() -> const Eigen::Vector3d & { return reference_velocity; });
    datastore().make_call("ismpc_walking::set_ref_vel", [this](Eigen::Vector3d vel) {
      reference_velocity = vel;
      velocityControl = true;
    });
    datastore().make_call("ismpc_walking::set_ref_pose", [this](sva::PTransformd pose) {
      target_pose_ = pose;
      velocityControl = false;
    });
    datastore().make_call("ismpc_walking::tds_by_ratio", [this](bool val) { Tds_by_ratio = val; });
    datastore().make_call("ismpc_walking::arm_swing_off", [this]() { armTask->weight(0); });
    datastore().make_call("ismpc_walking::arm_swing_on", [this]() { armTask->weight(10); });
    datastore().make_call("ismpc_walking::switch_support_foot", [this]() { SwitchFootSupport_manual(); });
    datastore().make_call("ismpc_walking::set_disturbance", [this](bool val){ Use_w = val; });
  }

  void comHeight(double h)
  {
    controller_config_.stab_config_dbl_supp.comHeight = h;
    controller_config_.stab_config_sg_supp.comHeight = h;
    controller_config_.stab_config_standing.comHeight = h;
    stabilizer_state_ = StabilizerState::None;
    // mc_rtc::log::info("height t {}",h);
  }
  void torsoPitch(double p)
  {
    controller_config_.stab_config_dbl_supp.torsoPitch = p;
    controller_config_.stab_config_sg_supp.torsoPitch = p;
    controller_config_.stab_config_standing.torsoPitch = p;
    stabilizer_state_ = StabilizerState::None;
    // mc_rtc::log::info("pitch t {}",p);;
  }

  void activate()
  {
    if(!Robot_Walking)
    {
      stabTask->enable();
      stabilizer_state_ = StabilizerState::None;
      active = true;
      debugMode = false;
    }
  }
  void deactivate()
  {
    if(!Robot_Walking)
    {
      stabTask->disable();
      // comTask->weight(0);
      active = false;
    }
  }


  bool wait_for_mpc_thread();

  std::shared_ptr<mc_tasks::lipm_stabilizer::StabilizerTask> stabTask;

  std::shared_ptr<mc_tasks::SurfaceTransformTask> swingFootTask;
  std::shared_ptr<mc_tasks::force::CoPTask> rightLandingTask;
  std::shared_ptr<mc_tasks::force::CoPTask> leftLandingTask;
  std::shared_ptr<mc_tasks::force::CoPTask> landingTask;

  std::shared_ptr<mc_tasks::SurfaceTransformTask> SupportFootTask;
  std::shared_ptr<mc_tasks::SurfaceTransformTask> leftSwingFootTask;
  std::shared_ptr<mc_tasks::SurfaceTransformTask> rightSwingFootTask;
  std::shared_ptr<mc_tasks::PostureTask> armTask;
  std::shared_ptr<mc_tasks::MomentumTask> momentumTask;
  std::shared_ptr<mc_tasks::CoMTask> comTask;

private:
  std::mutex mutex_mpc_;
  MPC_state mpc_thread_state;
  MPC_state mpc_state_;
  std::atomic<bool> MPC_thread_on = false;
  std::atomic<bool> MPC_thread_ready = false;
  std::atomic<bool> NewThreadState = false;
  std::atomic<bool> NewConfigState = true;
  bool stabilizer_active_ = true;
  std::thread walkingTrajectoryThread;

  Eigen::Vector3d dcmTarget = Eigen::Vector3d::Zero();
  Eigen::Vector3d dcmMeasured = Eigen::Vector3d::Zero();
  Eigen::Vector3d zmpCorr = Eigen::Vector3d::Zero();
  Eigen::Vector3d zmpTarget = Eigen::Vector3d::Zero();
  Eigen::Vector3d admittanceTarget = Eigen::Vector3d::Zero();
  Eigen::Vector3d lc_dot_target = Eigen::Vector3d::Zero();
  Eigen::Vector3d zmpMeasured = Eigen::Vector3d::Zero();
  Eigen::Vector2d supportMin = Eigen::Vector2d::Zero();
  Eigen::Vector2d supportMax = Eigen::Vector2d::Zero();
  std::string torsoBodyName_ = "";
  std::string leftFootLinkName_ = "";
  std::string rightFootLinkName_ = "";

  std::string rightHandName_ = "";
  std::string leftHandName_ = "";

  mc_filter::LowPass<sva::ForceVecd> leftHandDisturbanceFilter_;
  mc_filter::LowPass<sva::ForceVecd> rightHandDisturbanceFilter_;

  std::vector<mc_rbdyn::Plane> planes_;

  Eigen::VectorXd Traj_ant;

  double eta()
  {
    return std::sqrt(std::abs(mc_rtc::constants::gravity.z()) / controller_config_.stab_config.comHeight);
  }

  mc_rtc::clock::time_point t_clock;
  mc_rtc::clock::time_point t_test;

  bool User_Foot_Contact = true; // Both user feets are on the ground

  double x = 0.4;
  double y = 0.1;
  double z = 30; // Coordinate for a specified footstep position

  Eigen::Vector3d p_com_logged_ = Eigen::Vector3d::Zero();
  bool active = true; // MPC stabilization on or not
  bool UseRealRobot = true; // To use the real robots data
  bool UseMPCState = false;
  bool UseStepRecovery = false;
  bool IncreaseUpdate = false; // Call the MPC at controller sampling during double support
  bool Stop = true; // If true, the robot is at stop or the robot is about to stop at next step;
  bool Robot_Walking = false; // If false, the robot is not moving;
  std::mutex compute_trajectory_once_mtx;
  std::condition_variable compute_trajectory_once;
  std::atomic<bool> WalkingTrajectory_Computing = false;
  bool emergencyFlag = false; // Stop controller run loop
  bool AutoFootstepPlacement = true; // To enable the Autofootstep placement MPC
  bool UseAngularMomentum = false;
  bool UsePendulumSolver = true;
  bool Tds_by_ratio = true;
  bool force_contact_safety_ = true;
  bool updateAdmittance = false;
  bool velocityControl = true; // If false walk is done throught reference pose control

  double LeftFootRatio = 0.5;
  double maxRatioDelta = 0.2;
  double PrevLeftFootRatio = 0.5;
  double Ratio_target = 0.5; // A left foot ratio that set a zmp target when Stop
  double t = 0; // General timing in a step
  double t_lift = 0; // Time when the swing foot contact has been removed
  double t_contact = 0; // Time when foot hit the ground
  double mpc_thread_process_time = 0.0;
  double ControllerLoopTime = 0;
  double T_Steps = 1.1;
  double prevStepTiming = 0;
  double K_feedback = 1;

  StabilizerState stabilizer_state_ = StabilizerState::None;

  Eigen::Vector3d target_force_ = Eigen::Vector3d::Zero();

  //target wrench at the CoM
  sva::ForceVecd target_wrench_ = sva::ForceVecd::Zero();

  //measured foot wrench at the CoM
  sva::ForceVecd measured_wrench_ = sva::ForceVecd::Zero();

  double maxStiffTimeThreshold_ = 3; // Time after which hand task gain reach max [s]
  double linearStiffTimeThreshold_ = 1.5; // Time after which hand task gain switch from min to gradually reach max [s]
  double minStiffness_ = 10;
  double maxStiffness_ = 10;

  int N_Steps = 0;
  int N_Steps_Desired = -1;
  int N_Steps_Desired_std = -1;
  int N_Steps_Desired_recovery = 2;
  sva::PTransformd target_pose_ = sva::PTransformd::Identity(); // Reference pose for the footsteps plan generation

  double landing_time = 0; //time landing task is active

  double t_stop = 0;
  int count_stop = 0;
  // Measured by averaging the force sensor during the swing phase
  double vertical_force_offset_ = 0;
  size_t vertical_force_measure_cnt_ = 0;

  Eigen::Vector3d swingFootAcc = Eigen::Vector3d::Zero();
  Eigen::Vector3d swingFootVel = Eigen::Vector3d::Zero();
  sva::PTransformd X_0_SwingFootInitial = sva::PTransformd::Identity(); // Swing Foot Pose Before Swinging
  sva::PTransformd X_0_SwingFootInitial_real = sva::PTransformd::Identity();

  Eigen::Vector3d swing_foot_initial_pose =
      Eigen::Vector3d::Zero(); // Previous Swing Foot pose at the time of the MPC computation

  double SwingFootInitialAngle = 0.0;
  double input_tds = 0.25; // Double Step Time duration
  int count = 0; // Controller iterations
  double t_k = 0;
  double controller_timestep;
  int countStart = 0; // Controller Itaration at the time Start;
  int Index = 0; // Index of the target CoM CoMd ZMP in the vector returned by the MPC
  bool swing_foot_contact = true;
  bool doubleSupport_state = true;
  bool StepRecoveryState = false;
  bool autoStart = false;

  bool Use_w = true;
  Eigen::Vector3d w_ = Eigen::Vector3d::Zero();
  double kappa_ = 1;
  Eigen::Vector3d w_inf_ = Eigen::Vector3d::Zero();
  double kappa_inf_ = 1;
  double eta2_cstr = 0.0;
  Eigen::Vector3d Ldot_offset = Eigen::Vector3d::Zero(); // offset due to angular momentum
  Eigen::Vector3d Ldot = Eigen::Vector3d::Zero(); // current angular momentum
  std::string Tail = "Anticipative"; // Velocity tail, either "Periodic" Or "Truncated"

  Eigen::Vector3d SupportFootPose; // Initial  Foot Support at the time of computation

  Eigen::Matrix3d R_body_world_Step = Eigen::Matrix3d::Identity(); // Orientation of floating base updated at each steps

  std::string leftFootName_ = "LeftFootCenter";
  std::string rightFootName_ = "RightFootCenter";

  std::string supportFootName = "RightFootCenter";
  std::string swingFootName = "LeftFootCenter";

  ISMPC_Solver MPCSolver;
  FootTrajectory swingFootTrajectory;

  mc_filter::LowPass<sva::ForceVecd> filter_left_hand_wrench_;
  mc_filter::LowPass<sva::ForceVecd> filter_right_hand_wrench_;
  mc_filter::LowPass<Eigen::Vector3d> filter_gamma_;
  mc_filter::ExponentialMovingAverage<Eigen::Vector3d> zmp_vel_;

  bool FeetUp = false;

  mc_rtc::Configuration config_;

  double Vx_i = 0;
  double Vy_i = 0;
  double Omega_i = 0;
  Eigen::Vector3d reference_velocity = Eigen::Vector3d::Zero();

  Eigen::Vector3d staticPose = Eigen::Vector3d::Zero();

  sva::PTransformd ReferenceFrame_Origin_Offset = sva::PTransformd::Identity();

  sva::PTransformd leftFootTransformZero = sva::PTransformd::Identity();
  sva::PTransformd rightFootTransformZero = sva::PTransformd::Identity();

  sva::PTransformd X_0_leftFoot = sva::PTransformd::Identity();
  Eigen::Matrix3d R_0_leftFoot = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R_leftFoot_0 = Eigen::Matrix3d::Identity();
  Eigen::Vector3d T_leftFoot_0 = Eigen::Vector3d::Zero();

  sva::PTransformd X_0_rightFoot = sva::PTransformd::Identity();
  Eigen::Matrix3d R_0_rightFoot = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R_rightFoot_0 = Eigen::Matrix3d::Identity();
  Eigen::Vector3d T_rightFoot_0 = Eigen::Vector3d::Zero();

  sva::PTransformd X_0_support = sva::PTransformd::Identity();
  sva::PTransformd X_0_swing = sva::PTransformd::Identity();
  sva::PTransformd X_0_support_flat = sva::PTransformd::Identity();
  sva::PTransformd X_0_support_real = sva::PTransformd::Identity();
  sva::PTransformd X_support_0 = sva::PTransformd::Identity();
  sva::PTransformd X_support_0_real = sva::PTransformd::Identity();
  Eigen::Matrix3d R_0_support = Eigen::Matrix3d::Identity();
  Eigen::Vector3d T_0_support = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_0_support_real = Eigen::Matrix3d::Identity();
  Eigen::Vector3d T_0_support_real = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_support_0 = Eigen::Matrix3d::Identity();
  Eigen::Vector3d T_support_0 = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_support_0_real = Eigen::Matrix3d::Identity();
  Eigen::Vector3d T_support_0_real = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_swing_0 = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R_0_swing = Eigen::Matrix3d::Identity();
  Eigen::Vector3d T_swing_0 = Eigen::Vector3d::Zero();
  sva::PTransformd X_0_SwingFootTarget = sva::PTransformd::Identity();

  Eigen::Matrix3d floatingbaseWorldOri = Eigen::Matrix3d::Identity();
  Eigen::Vector3d floatingbaseWorldPos = Eigen::Vector3d::Zero();
  sva::PTransformd X_world_floatingbase = sva::PTransformd::Identity();
  Eigen::Vector3d floatingbaseWorldRPY = Eigen::Vector3d::Zero();

  double comAccZ = 0;
  mc_filter::LowPass<Eigen::Vector3d> filter_comAccZ;

  double currentLeftLeg = 0;
  double currentRightLeg = 0;

  unsigned int leftShoulderIndex = 0;
  unsigned int leftLegIndex = 0;
  unsigned int rightShoulderIndex = 0;
  unsigned int rightLegIndex = 0;

  std::vector<std::string> SwingFootJoints;

  Eigen::Vector6d footcontact_dof = Eigen::Vector6d::Zero();

  int kfoot = 0; // indx of the target step in the step plan

  // For Joystick {
  double maxVelX = 0.15;
  double minVelX = -0.15;
  double vRefX = 0;
  double vRefY = 0;
  double omegaRef = 0;
  double PrevVrefX = 0;
  bool joystickConnected = true;
  // For Joystick }

  std::vector<Eigen::Vector3d> SupPolygon;

  bool debugMode = false;
  bool tickerMode = false; //true if not in physical sim
  Eigen::Vector3d debugCoM = Eigen::Vector3d::Zero();
  Eigen::Vector3d debugZMP = Eigen::Vector3d::Zero();
  double debugTk = 0;
  bool debugDblSupp = true;
  bool debugStop = false;
};
