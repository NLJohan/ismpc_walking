#include "../include/ismpc_walking/Walking_controller.h"
#include <mc_control/Configuration.h>

#ifdef __linux__

#  include <sched.h>

void reset_affinity()
{
  cpu_set_t cpu_set;
  CPU_ZERO(&cpu_set);
  for(unsigned int i = 0; i < std::thread::hardware_concurrency(); ++i)
  {
    CPU_SET(i, &cpu_set);
  }
  int result = sched_setaffinity(0, sizeof(cpu_set_t), &cpu_set);
  if(result != 0)
  {
    perror("sched_setaffinity");
  }
}

#else

void reset_affinity() {}

#endif

Walking_controller::~Walking_controller()
{
  MPC_thread_on = false;
  if(walkingTrajectoryThread.joinable())
  {
    compute_trajectory_once.notify_all();
    walkingTrajectoryThread.join();
  }
}

Walking_controller::Walking_controller(mc_rbdyn::RobotModulePtr rm,
                                       double dt,
                                       const mc_rtc::Configuration & config,
                                       mc_control::ControllerParameters params)
: mc_control::fsm::Controller(rm, dt, config, params), filter_left_hand_wrench_(0.005, 0),
  filter_right_hand_wrench_(0.005, 0), filter_gamma_(0.005, 0), zmp_vel_(0.005, 0.05, {0., 0., 0.}),
  leftHandDisturbanceFilter_(dt, 0), rightHandDisturbanceFilter_(dt, 0), filter_comAccZ(dt, 0)
{

  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration stabiConfig(robot().module().defaultLIPMStabilizerConfiguration());
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration stabiConfig_standing(
      robot().module().defaultLIPMStabilizerConfiguration());
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration stabiConfig_sg_supp(
      robot().module().defaultLIPMStabilizerConfiguration());
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration stabiConfig_dbl_supp(
      robot().module().defaultLIPMStabilizerConfiguration());
  if(config.has("stabilizer"))
  {
    const auto & s_config = config("stabilizer");

    stabiConfig_standing.load(s_config);
    stabiConfig_sg_supp.load(s_config);
    stabiConfig_dbl_supp.load(s_config);

    if(config.has("stabilizer_sgsupp"))
    {
      const auto & s_config_sg = config("stabilizer_sgsupp");
      stabiConfig_sg_supp.load(s_config_sg);
    }
    if(config.has("stabilizer_dblsupp"))
    {
      const auto & s_config_dbl = config("stabilizer_dblsupp");
      stabiConfig_dbl_supp.load(s_config_dbl);
    }
  }

  if(config.has("footsteps_planner"))
  {
    planner_config_ = config("footsteps_planner");
  }

  controller_config_.stab_config = stabiConfig_standing;
  controller_config_.stab_config_standing = stabiConfig_standing;
  controller_config_.stab_config_sg_supp = stabiConfig_sg_supp;
  controller_config_.stab_config_dbl_supp = stabiConfig_dbl_supp;
  controller_config_.controller_timestep = dt;

  MPCSolver = ISMPC_Solver(dt, controller_config_.delta, controller_config_.Tp, controller_config_.Tc);

  Configure(controller_config_);
  Configure(config);

  auto rConfig = config("walking_controller");
  rConfig("torsoBodyName", torsoBodyName_);
  rConfig("rightFootLink", rightFootLinkName_);
  rConfig("leftFootLink", leftFootLinkName_);
  rConfig("left_foot_surface", leftFootName_);
  rConfig("right_foot_surface", rightFootName_);
  rConfig("left_hand_surface", leftHandName_);
  rConfig("right_hand_surface", rightHandName_);

  mc_rtc::log::info(robots().envIndex());
  controller_timestep = dt;
  // config_.load(config);
  // static auto constraint = mc_solver::ConstraintSetLoader::load(solver(), config("collisions")[0]);

  datastore().make_call(
      "KinematicAnchorFrame::" + robot().name(), [this](const mc_rbdyn::Robot & robot)
      { return sva::interpolate(robot.surfacePose(leftFootName_), robot.surfacePose(rightFootName_), LeftFootRatio); });

  const auto oConfig = config("ObserverPipelines")("observers");
  for(auto conf : oConfig)
  {

    if(conf("type") == "KinematicInertial")
    {
      std::cout << (conf("config").dump()) << std::endl;
      const auto conf_obs = conf("config")("anchorFrame");
      conf_obs("maxAnchorFrameDiscontinuity", maxRatioDelta);
    }
  }

  // solver().addConstraintSet(*constraint);
  // solver().addConstraintSet(contactConstraint);
  // solver().addConstraintSet(kinematicsConstraint);
  // solver().addConstraintSet(dynamicsConstraint);

  footcontact_dof << 0, 0, 1, 0, 0, 0;
  addContact({robot().name(), "ground", rightFootName_, "AllGround", 0.7, footcontact_dof});
  addContact({robot().name(), "ground", leftFootName_, "AllGround", 0.7, footcontact_dof});

  leftSwingFootTask =
      std::make_shared<mc_tasks::SurfaceTransformTask>(leftFootName_, robots(), robots().robotIndex(), 10.0, 10.);
  leftSwingFootTask->name("swingFootTask_left");

  rightSwingFootTask =
      std::make_shared<mc_tasks::SurfaceTransformTask>(rightFootName_, robots(), robots().robotIndex(), 10.0, 10.);
  rightSwingFootTask->name("swingFootTask_right");

  const sva::ForceVecd landingAdmittance = sva::ForceVecd(Eigen::Vector3d{0.03, 0.03, 0}, Eigen::Vector3d{0, 0, 0.03});
  leftLandingTask = std::make_shared<mc_tasks::force::CoPTask>(leftFootName_, robots(), robots().robotIndex(), 1,
                                                               controller_config_.stab_config.contactWeight);
  leftLandingTask->name("landingTask_left");
  leftLandingTask->maxLinearVel(Eigen::Vector3d::Ones() * 10);
  leftLandingTask->maxAngularVel(Eigen::Vector3d::Ones() * 10);
  leftLandingTask->admittance(landingAdmittance);
  leftLandingTask->damping(controller_config_.stab_config.contactDamping);

  rightLandingTask = std::make_shared<mc_tasks::force::CoPTask>(rightFootName_, robots(), robots().robotIndex(), 1,
                                                                controller_config_.stab_config.contactWeight);
  rightLandingTask->name("landingTask_right");
  rightLandingTask->maxLinearVel(Eigen::Vector3d::Ones() * 10);
  rightLandingTask->maxAngularVel(Eigen::Vector3d::Ones() * 10);
  rightLandingTask->admittance(landingAdmittance);
  rightLandingTask->damping(controller_config_.stab_config.contactDamping);

  swingFootName = leftFootName_;
  supportFootName = rightFootName_;

  momentumTask = std::make_shared<mc_tasks::MomentumTask>(robots(), robot().robotIndex(), 10, 10);
  Eigen::Vector6d momentumTask_dof;
  momentumTask_dof << 1, 1, 1, 0, 0, 0;
  momentumTask->dimWeight(momentumTask_dof);

  comTask = std::make_shared<mc_tasks::CoMTask>(robots(), robot().robotIndex(), 10, 200);

  if(rConfig.has("momemtum_task_joints"))
  {
    momentumTask->selectActiveJoints(solver(), rConfig("momemtum_task_joints"));
  }

  stabTask = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(solver().robots(), solver().realRobots(),
                                                                         solver().robots().robotIndex(), solver().dt());

  mc_rtc::log::info("com stiff {}", controller_config_.stab_config.comStiffness);

  SupportFootPose = robot().surfacePose(supportFootName).translation();
  SupportFootPose.z() = 0;

  swing_foot_initial_pose = robot().surfacePose(swingFootName).translation();
  X_0_SwingFootInitial = swing_foot_initial_pose;

  staticPose =
      ((robot().surfacePose(leftFootName_).translation() + robot().surfacePose(rightFootName_).translation()) / 2);
  staticPose.z() = controller_config_.stab_config.comHeight;

  swingFootAcc.setZero();
  swingFootVel.setZero();

  filter_left_hand_wrench_ = mc_filter::LowPass<sva::ForceVecd>(solver().dt(), controller_config_.wrench_filter_cutoff);
  filter_right_hand_wrench_ =
      mc_filter::LowPass<sva::ForceVecd>(solver().dt(), controller_config_.wrench_filter_cutoff);
  filter_gamma_ = mc_filter::LowPass<Eigen::Vector3d>(solver().dt(), controller_config_.gamma_filter_cutoff);

  zmp_vel_ = mc_filter::ExponentialMovingAverage<Eigen::Vector3d>(solver().dt(), controller_config_.delta,
                                                                  Eigen::Vector3d::Zero());

  create_datastore();
  getTransformations();

  // Change to:
  autoStart = config("walking_controller")("auto_start")("activate");
  autoStartConfigured = autoStart;
  reference_velocity.setZero();

  MPCSolver.Allow_none(controller_config_.MPC_allow_None);

  solver().addTask(stabTask);
  solver().addTask(comTask);
  solver().addTask(leftSwingFootTask);
  solver().addTask(rightSwingFootTask);
  solver().addTask(momentumTask);
  updateTasks();
  // --- CoM target logging for mc_log_ui comparison with com_jvrc1_pos_z ---
  logger().addLogEntry("com_target_pos", this,
    [this]() -> const Eigen::Vector3d & { return p_com_logged_; });
  // --- CoM-z tracking diagnostics: compare against com_jvrc1_pos_z (or equivalent robot-state
  // CoM-z log entry, already provided by mc_rtc) to assess (a) MPC-thread pipeline delay
  // (com_height_ref_h0 vs com_height_ref_hidx) and (b) whole-body-tracking lag/filtering
  // (com_height_ref_hidx, i.e. what was actually commanded to comTask, vs the true measured
  // CoM-z). See MoveCoM() for where these are captured.
  logger().addLogEntry("com_height_ref_h0", this,
    [this]() -> const double & { return m_com_height_raw_h0_logged_; });
  logger().addLogEntry("com_height_ref_hidx", this,
    [this]() -> const double & { return m_com_height_raw_hidx_logged_; });
  // --- Pipeline-jitter diagnostics: see the detailed comment at the capture site in MoveCoM().
  // mpc_index: mpc_state_.Index (samples ahead of CoM_height[0] actually read out this cycle).
  // mpc_process_time_ms: raw solve-time measurement driving Index's computation.
  // x0_support_z: support-foot height offset added on top of CoM_height[Index] to form
  //   com_target_pos_z -- logged separately to isolate it from the raw height reference.
  // controller_timestep / mpc_delta: the two periods Index mixes units between (see comment in
  //   MoveCoM()); logged every cycle (not just once) in case either is ever reconfigured live.
  logger().addLogEntry("mpc_index", this, [this]() -> const int & { return m_mpc_index_logged_; });
  logger().addLogEntry("mpc_process_time_ms", this,
    [this]() -> const double & { return m_mpc_thread_process_time_logged_; });
  logger().addLogEntry("x0_support_z", this,
    [this]() -> const double & { return m_x0_support_z_logged_; });
  logger().addLogEntry("controller_timestep_diag", this,
    [this]() -> const double & { return controller_timestep; });
  logger().addLogEntry("mpc_delta_diag", this,
    [this]() -> const double & { return controller_config_.delta; });
  deactivate();
  mc_rtc::log::success("ismpc_walking controller init done ");
  if(autoStart)
  {
    activate();
    Stop = false;
    N_Steps_Desired_std = config("walking_controller")("auto_start")("steps");
    N_Steps_Desired = N_Steps_Desired_std;
    double t_step = config("walking_controller")("auto_start")("ts");
    ts(t_step);
    reference_velocity = config("walking_controller")("auto_start")("speed");
    controller_config_.Double_Step_Ratio = config("walking_controller")("auto_start")("double_support_ratio");
  }
}

bool Walking_controller::wait_for_mpc_thread()
{
  if(!MPC_thread_ready)
  {
    // if(!datastore().has("footstep_planner::configure"))
    // {
    //   mc_rtc::log::info("waiting for footsteps_planner plugin");
    //   return false;
    // }
    if(!MPC_thread_on)
    {
      mc_rtc::log::info("Start MPC thread");
      UpdateInitialVectors();
      UpdatePlanner_input();
      WalkingTrajectory_Computing = true;
      MPC_thread_on = true;
      walkingTrajectoryThread = std::thread(&Walking_controller::WalkingTrajectoryLoop, this);
#ifndef WIN32
      // Lower thread priority so that it has a lesser priority than the real time
      // thread
      auto th_handle = walkingTrajectoryThread.native_handle();
      int policy = 0;
      sched_param param{};
      pthread_getschedparam(th_handle, &policy, &param);
      param.sched_priority = 80;
      if(pthread_setschedparam(th_handle, SCHED_RR, &param) != 0)
      {
        mc_rtc::log::warning(
            "[{}] Failed to lower calibration thread priority. If you are running on a real-time system, "
            "this might cause latency to the real-time loop.");
      }
#endif

      compute_trajectory_once.notify_all();
    }
    else
    {
      if(!WalkingTrajectory_Computing)
      {
        MPC_thread_ready = true;
        mc_rtc::log::success("MPC thread on");
        add_ISMPC_Config_GUI();
        addToGUI();
        add_FootSteps_GUI();
        Stabilizer_GUI(controller_config_.stab_config_sg_supp, "single support");
        Stabilizer_GUI(controller_config_.stab_config_dbl_supp, "double support");
        Stabilizer_GUI(controller_config_.stab_config_standing, "standing");
        AddToLog();
      }
      else
      {
        mc_rtc::log::info("Waiting for first computation");
      }
    }
  }
  return MPC_thread_ready;
}

void Walking_controller::WalkingTrajectoryLoop()
{
  reset_affinity();
  do
  {
    WalkingTrajectory_Computing = true;
    ComputeWalkingTrajectory();
    WalkingTrajectory_Computing = false;
    std::unique_lock<std::mutex> lock(compute_trajectory_once_mtx);
    compute_trajectory_once.wait(lock);
  } while(MPC_thread_on);
}

void Walking_controller::ComputeWalkingTrajectory()
{
  mc_rtc::clock::time_point t_clock = mc_rtc::clock::now();

  // Cleared here, at the top of every solve, so ismpc_wants_stop always
  // reflects only THIS solve's assessment (set below, at the two
  // autonomous-stop sites) rather than latching true forever once ISMPC
  // has ever wanted to stop once. See its declaration in the header.
  ismpc_wants_stop = false;

  {
    std::lock_guard<std::mutex> lk_copy_state(mutex_mpc_);
    UpdateInitialVectors();
    UpdatePlanner_input();
    mpc_thread_state = mpc_state_;
    mpc_thread_state.stop = !Robot_Walking;
  }
  if(NewConfigState)
  {
    MPCSolver.configure(controller_config_);
    NewConfigState = false;
  }
  MPCSolver.AutoFootstepPlacement = AutoFootstepPlacement;
  MPCSolver.UsePendulumSolver = UsePendulumSolver;
  MPCSolver.UseAngularMomentumDot = UseAngularMomentum;

  datastore().assign<std::vector<sva::MotionVecd>>("footsteps_planner::input_vel", mpc_thread_state.input_v_);
  datastore().assign<std::vector<sva::PTransformd>>("footsteps_planner::input_ref_pose",
                                                    mpc_thread_state.input_ref_pose_);
  datastore().assign<std::string>("footsteps_planner::support_foot_name", mpc_thread_state.input_Support_FootName);
  datastore().assign<sva::PTransformd>("footsteps_planner::support_foot_pose", mpc_thread_state.X_0_SupportFoot);
  datastore().assign<std::vector<double>>("footsteps_planner::input_time_steps", mpc_thread_state.input_timesteps_);
  datastore().call("footsteps_planner::compute_plan");

  mpc_thread_state.planned_steps_ = datastore().get<std::vector<sva::PTransformd>>("footsteps_planner::output_steps");

  mpc_thread_state.planned_timesteps_ = datastore().get<std::vector<double>>("footsteps_planner::output_time_steps");

  double tds = controller_config_.Double_Step_Ratio * mpc_thread_state.planned_timesteps_[0];
  if(!Tds_by_ratio)
  {
    tds = mpc_thread_state.input_tds;
  }
  mpc_thread_state.tds = tds;
  int Steps = N_Steps;
  int Steps_Desired = N_Steps_Desired;

  if(Stop && !doubleSupport_state)
  {
    Steps_Desired = Steps + 1;
  }

  // std::string tail = Tail;
  // if(mpc_thread_state.Index > 10. * MPCSolver.delta_mpc() / MPCSolver.delta_control())
  // {

  //   tail = "None";
  //   mc_rtc::log::warning("[ISMPC] Approaching Control Horizon, Tail temporary switched to None");
  // }
  // mc_rtc::log::warning(
  //     "[ComputeWalkingTrajectory] Steps={} Steps_Desired={} Stop={} doubleSupport_state={} ref_vel=({},{},{}) "
  //     "com_pu=({},{},{}) v_c_k=({},{},{}) t_k={} count={} realRobotCom=({},{},{}) realRobotComVel=({},{},{}) "
  //     "ComBias=({},{},{}) eta_at_index={} Index={}",
  //     Steps, Steps_Desired, Stop, doubleSupport_state, reference_velocity.x(), reference_velocity.y(),
  //     reference_velocity.z(), mpc_thread_state.p_u.x(), mpc_thread_state.p_u.y(), mpc_thread_state.p_u.z(),
  //     mpc_thread_state.v_c_k.x(), mpc_thread_state.v_c_k.y(), mpc_thread_state.v_c_k.z(), t_k, count,
  //     realRobot().com().x(), realRobot().com().y(), realRobot().com().z(), realRobot().comVelocity().x(),
  //     realRobot().comVelocity().y(), realRobot().comVelocity().z(), mpc_thread_state.ComBias.x(),
  //     mpc_thread_state.ComBias.y(), mpc_thread_state.ComBias.z(),
  //     mpc_thread_state.getEta(static_cast<size_t>(mpc_thread_state.Index)), mpc_thread_state.Index);
  MPCSolver.init_MPC(mpc_thread_state, Tail, Steps_Desired, Steps);
  // MPCSolver.Puk(mpc_state_.p_u);

  if(Use_w)
  {

    MPCSolver.InfiniteDisturbance(w_inf_, kappa_inf_);
    if(!doubleSupport_state || (debugMode && !debugDblSupp))
    {
      MPCSolver.Disturbance(w_, kappa_, 0.1);
    }
  }

  MPCSolver.GetWalkingParameters(mpc_thread_state.stop);

  std::chrono::duration<double, std::milli> time_span = mc_rtc::clock::now() - t_clock;
  mpc_thread_process_time = time_span.count();

  if(MPCSolver.QPsucceeded())
  {
    std::lock_guard<std::mutex> lk_copy_state(mutex_mpc_);
    mpc_thread_state.optimal_tds = MPCSolver.Tds();
    mpc_thread_state.optimal_timesteps_ = MPCSolver.timesteps();
    mpc_thread_state.optimal_steps_ = MPCSolver.optimal_steps();
    mpc_thread_state.QPSuccess = true;
    mpc_thread_state.FeasibilityPolygonStandingSwitch = MPCSolver.feasibility_region_switched();
    mpc_thread_state.X_MPC = MPCSolver.X_MPC();
    mpc_thread_state.Y_MPC = MPCSolver.Y_MPC();
    mpc_thread_state.CoM_height = MPCSolver.CoM_height_vec();
    mpc_thread_state.CoM_height_vel = MPCSolver.CoM_height_vel_vec();
    mpc_thread_state.CoM_height_acc = MPCSolver.CoM_height_acc_vec();
    mpc_thread_state.CoM_height_fine = MPCSolver.CoM_height_fine_vec();
    mpc_thread_state.CoM_height_vel_fine = MPCSolver.CoM_height_vel_fine_vec();
    mpc_thread_state.CoM_height_acc_fine = MPCSolver.CoM_height_acc_fine_vec();
    mpc_thread_state.eta_vec = MPCSolver.eta_vec();
    mpc_thread_state.Index = 1 + static_cast<int>(mpc_thread_process_time * 1e-3 / controller_timestep);
    mpc_thread_state.SupPolygon = MPCSolver.get_polynome_support();
    mpc_thread_state.Traj_ant = MPCSolver.GetAfterTc_ZMP_trajectory();
    mpc_thread_state.Tail = MPCSolver.Tail() != "None";
    mpc_thread_state.stab_error = MPCSolver.stability_error();
    mpc_thread_state.Pu_max = MPCSolver.Puk_max().segment(0, 2);
    mpc_thread_state.Pu_min = MPCSolver.Puk_min().segment(0, 2);
    mpc_thread_state.mpc_u_ = MPCSolver.ZMP_vel();
    mpc_thread_state.initial_zmp_ = MPCSolver.Initial_ZMP();
    mpc_thread_state.delayed_zmp_ = MPCSolver.Delayed_ZMP();
    mpc_thread_state.standing_mode = MPCSolver.stop();
    mpc_thread_state.FeasibilityPolygon = MPCSolver.feasibility_region();
    mpc_thread_state.alpha = MPCSolver.support_state();
    mpc_thread_state.ref_zmp_ = MPCSolver.zmp_ref().segment(0, 2);
    mpc_thread_state.admittance_ref_ = MPCSolver.admittance_references();
    mpc_thread_state.QP_zmp = MPCSolver.QP_zmp();
    mpc_thread_state.QP_dcm = MPCSolver.QP_dcm();
    mpc_thread_state.eta = MPCSolver.eta();
    mpc_thread_state.mpc_Lc_dot_ = MPCSolver.Lc_dot();
    kfoot = 0;
    NewThreadState = true;

    if(std::abs(MPCSolver.stability_error().x()) > controller_config_.max_stability_error
       || std::abs(MPCSolver.stability_error().y()) > controller_config_.max_stability_error && !StepRecoveryState)
    {
      // Advisory only: the policy has full authority over Stop (see
      // policyWantsWalk/SetPolicyWantsWalk). This no longer forces Stop --
      // it only records ISMPC's own opinion for the policy to observe and
      // learn from via ismpcWantsStop().
      mc_rtc::log::error("MPC result is too far from stability condition (advisory, policy has final say)");
      ismpc_wants_stop = true;
    }
  }
  else
  {
    mpc_state_.Pu_max = MPCSolver.Puk_max().segment(0, 2);
    mpc_state_.Pu_min = MPCSolver.Puk_min().segment(0, 2);
    mpc_state_.SupPolygon = MPCSolver.get_polynome_support();
    mpc_state_.mpc_Lc_dot_.setZero();
    mpc_state_.QPSuccess = false;
    if(!StepRecoveryState)
    {
      // Advisory only -- see comment above.
      mc_rtc::log::error("MPC failed (advisory, policy has final say)");
      ismpc_wants_stop = true;
    }
  }
}

void Walking_controller::UpdatePlanner_input()
{
  mpc_state_.input_v_.clear();
  mpc_state_.input_ref_pose_.clear();

  Eigen::Vector3d step_velocity = reference_velocity;
  double step_time = T_Steps;

  if(StepRecoveryState)
  {
    step_velocity.setZero();
  }

  if(velocityControl)
  {
    for(int k = 0; k < static_cast<int>(std::round(controller_config_.Tp / controller_config_.delta)); k++)
    {
      mpc_state_.input_v_.push_back(sva::MotionVecd(Eigen::Vector3d{0, 0, step_velocity.z()},
                                                    Eigen::Vector3d{step_velocity.x(), step_velocity.y(), 0}));
    }
  }
  else
  {
    mpc_state_.input_ref_pose_.push_back(target_pose_);
  }

  mpc_state_.input_timesteps_ = {step_time};
  while(mpc_state_.input_timesteps_.back() <= controller_config_.Tp)
  {
    mpc_state_.input_timesteps_.push_back(
        static_cast<double>((static_cast<int>(mpc_state_.input_timesteps_.size()) + 1) * step_time));
  }

  mpc_state_.set_input_tds(input_tds);
  mpc_state_.input_Support_FootName = "LeftFoot";
  if(supportFootName == rightFootName_)
  {
    mpc_state_.input_Support_FootName = "RightFoot";
  }
  mpc_state_.X_0_SupportFoot =
      sva::PTransformd(sva::RotZ(mc_rbdyn::rpyFromMat(robot().surfacePose(supportFootName).rotation()).z()),
                       robot().surfacePose(supportFootName).translation());
  mpc_state_.X_0_Initial_SwingFoot =
      sva::PTransformd(sva::RotZ(mc_rbdyn::rpyFromMat(robot().surfacePose(swingFootName).rotation()).z()),
                       robot().surfacePose(swingFootName).translation());

  mpc_state_.X_0_SwingFoot = X_0_swing;
  // Robot_Walking = !(Stop && doubleSupport_state); // This is so that the policy cannot start on tick 0
  mpc_state_.stop = !Robot_Walking;
  if(debugMode)
  {
    mpc_state_.stop = debugStop;
  }
}

void Walking_controller::CheckStepRecovery()
{
  if(MPC_thread_ready)
  {
    Eigen::MatrixX2d normals = MPCSolver.standing_feasibility_polygone().normals();
    Eigen::VectorXd offset = MPCSolver.standing_feasibility_polygone().offsets();
    Eigen::Vector2d dcm =
        (realRobot().com() + (realRobot().comVelocity() / mpc_state_.getEta(0))).segment(0, 2) + stabTask->biasDCM();
    Eigen::VectorXd stability_check = normals * dcm - offset;
    bool ok = true;
    for(int i = 0; i < stability_check.rows(); i++)
    {
      if(stability_check[i] > 1e-3)
      {
        ok = false;
        mc_rtc::log::info("break on cstr {}\nstabi check\n{}", i, stability_check);
        break;
      }
    }
    normals = mpc_state_.FeasibilityPolygonStandingSwitch.normals();
    offset = mpc_state_.FeasibilityPolygonStandingSwitch.offsets();
    stability_check = normals * dcm - offset;
    bool ok_switch = true;
    for(int i = 0; i < stability_check.rows(); i++)
    {
      if(stability_check[i] > 1e-3)
      {
        ok_switch = false;
        mc_rtc::log::info("[Support foot switch] break on cstr {}\nstabi check\n{}", i, stability_check);
        break;
      }
    }
    if(!ok || !ok_switch)
    {
      mc_rtc::log::warning("Can't Stop, stepping");
      N_Steps_Desired = N_Steps_Desired_recovery;
      // if(!ok_switch)
      // {
      //   SwitchFootSupport_manual();
      //   return;
      // }

      // mc_rtc::log::info("p_u {} ; p_u max {}",stabTask->measuredDCM(),MPCSolver.Puk_max());
      // mc_rtc::log::info("p_u {} ; p_u min {}",stabTask->measuredDCM(),MPCSolver.Puk_min());

      sva::PTransformd ff = robot().posW();
      if((ff.rotation() * stabTask->measuredCoMd()).x() < 0 && !StepRecoveryState)
      {
        if((ff.rotation()
            * (robot().frame(supportFootName).position().translation()
               - robot().frame(swingFootName).position().translation()))
               .x()
           > 0)
        {
          SwitchFootSupport_manual();
        }
      }
      if((ff.rotation() * stabTask->measuredCoMd()).x() > 0 && !StepRecoveryState)
      {
        if((ff.rotation()
            * (robot().frame(supportFootName).position().translation()
               - robot().frame(swingFootName).position().translation()))
               .x()
           < 0)
        {
          SwitchFootSupport_manual();
        }
      }

      const Eigen::Vector2d t_supp_swing = (robot().frame(swingFootName).position().translation()
                                            - robot().frame(supportFootName).position().translation())
                                               .segment(0, 2);
      const double l = t_supp_swing.norm();
      const Eigen::Vector2d t_supp_dcm =
          stabTask->measuredDCM().segment(0, 2) - robot().frame(supportFootName).position().translation().segment(0, 2);
      const double d_proj = t_supp_dcm.dot(t_supp_swing.normalized()) / l;
      mc_rtc::log::info(d_proj);
      if(d_proj < 0.3
         && std::abs((ff.rotation()
                      * (robot().frame(supportFootName).position().translation()
                         - robot().frame(swingFootName).position().translation()))
                         .x())
                < 0.05)
      {
        SwitchFootSupport_manual();
      }

      StepRecoveryState = true;
      Stop = false;
    }
    else
    {
      StepRecoveryState = false;
    }
  }
}

bool Walking_controller::run()
{
  JoystickInputs();
  if(!wait_for_mpc_thread())
  {
    return mc_control::fsm::Controller::run();
  }

  std::chrono::duration<double, std::milli> time_span = mc_rtc::clock::now() - t_clock;
  ControllerLoopTime = time_span.count();
  t_clock = mc_rtc::clock::now();

  if(emergencyFlag) return false;

  t = (count - countStart) * controller_timestep;

  planes_.clear();

  getTransformations();

  {
    std::lock_guard<std::mutex> lk_copy_state(mutex_mpc_);
    if(NewThreadState)
    {
      mpc_state_ = mpc_thread_state;
      NewThreadState = false;
      updateAdmittance = true;
    }
    // std::cerr << "[RUN_DEBUG] X_MPC.size()=" << mpc_state_.X_MPC.size()
    //       << " Y_MPC.size()=" << mpc_state_.Y_MPC.size()
    //       << " Index=" << mpc_state_.Index
    //       << " QPSuccess=" << mpc_state_.QPSuccess
    //       << std::endl;
    MoveCoM();
    UpdateInitialVectors();
    UpdatePlanner_input();
    mpc_state_.Index += 1;
  }

  // Stop is no longer mutated here: it is set exclusively via
  // SetPolicyWantsWalk() (the bridge entry point), which the RL policy
  // calls every action period. See policyWantsWalk's declaration in the
  // header for the full design rationale.
  if(!(Stop && doubleSupport_state))
  {
    Robot_Walking = (mpc_state_.X_MPC.size() > 0); // this is so that the policy doenst make the robot walk on first post-reset tick (otherwise error_and_throw)
    if(t - t_k > controller_config_.delta || (doubleSupport_state && IncreaseUpdate))
    {
      t_k += (doubleSupport_state && IncreaseUpdate) ? controller_timestep : controller_config_.delta;
      compute_trajectory_once.notify_all();
    }
    MoveFeet(t);
  }
  else
  {
    if(active)
    {
      MoveFeet(0);
    }
    updateTasks();
    N_Steps = 0;
    N_Steps_Desired = N_Steps_Desired_std;
    t_stop = (count - count_stop) * controller_timestep;
    if(UseRealRobot && mpc_state_.standing_mode)
    {
      if(UseStepRecovery)
      {
        CheckStepRecovery();
      }
    }
    if(t_stop + controller_timestep >= controller_config_.delta || StepRecoveryState || IncreaseUpdate)
    {
      count_stop = count;

      compute_trajectory_once.notify_all();
    }
    // compute_trajectory_once.notify_all();

    t_k = -controller_config_.delta;
    kfoot = 0;
    countStart = count + 1;

    Robot_Walking = false;
  }


  auto configureStabilizer = [&](StabilizerState targetState,
                                 const mc_rbdyn::lipm_stabilizer::StabilizerConfiguration & configBase, double lambda,
                                 const std::string & logMsg)
  {
    stabilizer_state_ = targetState;

    mc_rbdyn::lipm_stabilizer::StabilizerConfiguration config = configBase;
    controller_config_.lambda_ = lambda;

    comTask->weight(config.comWeight);
    comTask->stiffness(config.comStiffness);
    comTask->selectActiveJoints(solver(), config.comActiveJoints);

    config.comWeight = 0.0;

    stabTask->configure(config);
    Configure(controller_config_);

    if(!logMsg.empty()) mc_rtc::log::info(logMsg);
  };

  if(active)
  {
    if(!doubleSupport_state && stabilizer_state_ != StabilizerState::SingleSupport)
    {
      configureStabilizer(StabilizerState::SingleSupport, controller_config_.stab_config_sg_supp,
                          controller_config_.lambda_sg_supp, "configure sg");
    }
    else if(Robot_Walking && doubleSupport_state && stabilizer_state_ != StabilizerState::DoubleSupport)
    {
      configureStabilizer(StabilizerState::DoubleSupport, controller_config_.stab_config_dbl_supp,
                          controller_config_.lambda_dbl_supp, "configure dbl");
    }
    else if(!Robot_Walking && stabilizer_active_)
    {
      if(tickerMode)
      {
        controller_config_.stab_config_standing.copAdmittance.setZero();
        controller_config_.stab_config_standing.dfAdmittance.setZero();
      }
      if(stabilizer_state_ != StabilizerState::Standing)
      {
        configureStabilizer(StabilizerState::Standing, controller_config_.stab_config_standing,
                            controller_config_.lambda_dbl_supp, "configure std");
      }
    }
  }
  controller_config_.stab_config = stabTask->config();

  count += 1;
  return mc_control::fsm::Controller::run();
}

void Walking_controller::MoveCoM()
{

  if(mpc_state_.Index + 1 >= mpc_state_.X_MPC.size())
  {

    if(!Robot_Walking)
    {
      if(active)
      {
        mc_rtc::log::error("Control Horizon reached");
        deactivate();
      }
    }
    else
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("Control Horizon reached");
    }
  }

  Eigen::Vector3d p_com(mpc_state_.Get_CoM_planarTarget(mpc_state_.Index));
  // Time-Varying Fix: p_com.z() now comes from mpc_state_.Get_CoM_planarTarget's z component
  // (which reads mpc_state_.CoM_height[Index], populated from the solver's time-varying
  // CoM_height vector) instead of being hardcoded to the constant comHeight. The support-foot
  // and swing-foot z offsets are preserved, now added on top of the time-varying value.

  // p_com.z() += X_0_support.translation().z();
  if(!doubleSupport_state && swing_foot_contact)
  {
    p_com.z() = mpc_state_.Get_CoM_planarTarget(mpc_state_.Index).z() + robot().surfacePose(swingFootName).translation().z();
  }
  Eigen::Vector3d Vc(mpc_state_.Get_CoMVel_planarTarget(mpc_state_.Index));
  // CoM-height feedforward: Vc.z() now comes from Get_CoMVel_planarTarget's
  // z-component (mpc_state_.CoM_height_vel[Index], the solver's analytic
  // zc_dot for the current sine reference) instead of being hardcoded to 0.
  // Falls back to 0 automatically (via Get_CoMVel_planarTarget's own bounds
  // check) for signal cases/cold-start conditions where CoM_height_vel is
  // empty, so this is safe even before the first populated solve.
  zmpTarget = mpc_state_.Get_ZMP_planarTarget(mpc_state_.Index);

  lc_dot_target = mpc_state_.get_Lc_dot(0);

  const int n = static_cast<int>(controller_config_.delta / controller_timestep);

  // mc_rtc::log::info("//Index : {}, z_y {}",mpc_state_.Index,zmpTarget.y());

  // Time-Varying Fix: eta applicable to this control sample is the one at the current horizon
  // index, not the stale/global mpc_state_.eta. Use the same Index as p_com/Vc/zmpTarget above.
  const double eta_now = mpc_state_.getEta(static_cast<size_t>(mpc_state_.Index));

  Eigen::Vector3d deltaLc = Eigen::Vector3d::Zero(); // offset to the acc ref to account for Lcd;
  if(UseAngularMomentum)
  {
    Eigen::Vector6d momentumTask_stiff = Eigen::Vector6d::Zero();
    Eigen::Vector6d momentumTask_dof;
    momentumTask_dof << 1, 1, 1, 0, 0, 0;
    momentumTask->dimWeight(momentumTask_dof);
    momentumTask_stiff << 0, 0, 5, 0, 0, 0;
    momentumTask->stiffness(momentumTask_stiff);

    momentumTask->weight(controller_config_.momentumTaskWeight);
    const auto target = sva::ForceVecd(lc_dot_target, Eigen::Vector3d::Zero());
    momentumTask->refAccel(target.vector());

    deltaLc << -lc_dot_target.y(), lc_dot_target.x(), 0.;
    // Time-Varying Fix: use the time-varying CoM height at the current index, not the constant
    // controller_config_.stab_config.comHeight, for consistency with the variable-height model.
    deltaLc /= (robot().mass() * p_com.z());
  }
  else
  {
    momentumTask->weight(0);
  }

  Eigen::Vector3d acc_com = std::pow(eta_now, 2) * (mpc_state_.p_c_k - zmpTarget) + deltaLc;
  // CoM-height feedforward: acc_com.z() now comes from the solver's analytic
  // zc_ddot for the current sine reference (mpc_state_.CoM_height_acc[Index],
  // the SAME value already used internally to compute eta_now via
  // ISMPC_Solver's m_eta[i] = sqrt((zc_ddot + g) / CoM_height[i]) -- now also
  // exposed here as an explicit feedforward term instead of being discarded
  // after that eta computation. Unlike Vc.z() above, this does not replace a
  // LIPM-model-derived x/y quantity: acc_com's x/y come from the pendulum
  // model (eta_now^2 * (p_c_k - zmpTarget)); z was previously just an
  // unconditional 0 with no equivalent physical derivation, so this is a
  // genuinely new feedforward term, not a substitution of one signal for
  // another. Falls back to 0 (via Get_CoMHeightAccel_target's own bounds
  // check) for signal cases/cold-start conditions where CoM_height_acc is
  // empty.
  acc_com.z() = mpc_state_.Get_CoMHeightAccel_target(static_cast<size_t>(mpc_state_.Index));
  admittanceTarget = mpc_state_.delayed_zmp_ + mpc_state_.get_u(0);
  admittanceTarget.z() = 0;

  if(doubleSupport_state && updateAdmittance && mpc_state_.get_tds() - t_k > 0
     && mpc_state_.zmp_references().size() != 0)
  {
    size_t n_indx = static_cast<int>((mpc_state_.get_tds() - t_k) / controller_config_.delta);
    n_indx = std::max(std::min(n_indx, size_t(20)), size_t(1));
    const size_t indx_start = static_cast<size_t>(mpc_state_.Index);
    std::vector<Eigen::Vector2d> zmp_ref;

    const size_t n = static_cast<size_t>(controller_config_.delta / controller_timestep);
    for(size_t i = 1; i < n_indx + 1; i++)
    {
      zmp_ref.push_back(mpc_state_.Get_ZMP_planarTarget(indx_start + i * n).segment(0, 2));
    }

    stabTask->horizonReference(zmp_ref, controller_config_.delta);
    updateAdmittance = false;
  }

  Eigen::Vector3d acc_wrench = std::pow(eta_now, 2) * (mpc_state_.p_c_k - admittanceTarget) + deltaLc;
  acc_wrench.z() = 0;

  target_force_ = robot().mass() * (acc_com + mc_rtc::constants::gravity);
  const sva::PTransformd X_z_c = sva::PTransformd(mpc_state_.p_c_k) * sva::PTransformd(mpc_state_.p_z_k).inv();
  target_wrench_ = X_z_c.dualMul(sva::ForceVecd{Eigen::Vector3d::Zero(), target_force_});

  // stabTask->target(p_com, Vc, acc_wrench, zmpTarget,Eigen::Vector3d::Zero(),lc_dot_target);

  stabTask->target(p_com, Vc, acc_wrench, zmpTarget);
  // stabTask->target(p_com, Vc, acc_wrench, admittanceTarget);
  // stabTask->target(p_com, Vc, acc_com, zmpTarget);
  if(!active || debugMode)
  {
    p_com.segment(0, 2) = sva::interpolate(robot().surfacePose(leftFootName_), robot().surfacePose(rightFootName_), 0.5)
                              .translation()
                              .segment(0, 2);
    p_com.z() = controller_config_.stab_config.comHeight + X_0_support.translation().z();
    Vc.setZero();
    acc_com.setZero();
    if(!active)
    {
      if(!Stop)
      {
        Stop = true;
        mc_rtc::log::warning("[Walking Controller] MPC control is off, cannot walk");
      }
      lc_dot_target.setZero();
    }
  }
  p_com_logged_ = p_com;

  // --- CoM-z tracking diagnostics (see logger().addLogEntry calls in the constructor) ---
  // Gated by the SAME (active && !debugMode) condition as p_com_logged_/comTask->com() just
  // above -- these must never be read/logged from a stale or not-yet-populated mpc_state_
  // (e.g. before the first real MPC solve, or while debugMode/!active is holding p_com at the
  // fixed-stance fallback value). An earlier version of this instrumentation captured these
  // BEFORE this guard, unconditionally every tick, which produced large spurious pre-activation
  // oscillations with no physical meaning (an artifact of reading mpc_state_.Index/CoM_height
  // before they had a legitimate solve to draw from) -- fixed here by moving capture to this
  // gated location and simply holding the last value while inactive, rather than freeze at a
  // stale/garbage value.
  //
  // m_com_height_raw_h0_logged_: CoM_height[0], the raw value the solver computed for "now" at
  //   the START of the CURRENT MPC horizon (init_MPC's t_i = m_t_global sample), i.e. what was
  //   fed into the Riccati/eta computation for the present control cycle. This is the cleanest
  //   "ground truth reference" signal, free of the Index pipeline offset below.
  // m_com_height_raw_hidx_logged_: CoM_height[Index], the value actually READ OUT and applied to
  //   p_com.z() this control cycle (mpc_state_.Index accounts for the MPC-thread processing-time
  //   pipeline delay, see mpc_thread_state.Index in the MPC thread). Comparing this against
  //   CoM_height[0] isolates the pipeline-delay contribution from any later whole-body-tracking
  //   lag; comparing it against the true measured CoM-z (already logged by mc_rtc's own robot
  //   state, e.g. com_<robotName>_pos_z) isolates the whole-body-tracking contribution.
  // m_mpc_index_logged_ / m_mpc_thread_process_time_logged_: see the units-mismatch note in the
  //   constructor's addLogEntry block for mpc_index/mpc_process_time_ms.
  // m_x0_support_z_logged_: support-foot height offset added on top of CoM_height[Index] to form
  //   com_target_pos_z's z-component -- logged separately to confirm/refute that this offset
  //   (not a bug) explains why com_target_pos_z can sit above com_height_ref_hidx.
  if(active && !debugMode && !mpc_state_.CoM_height.empty())
  {
    m_com_height_raw_h0_logged_ = mpc_state_.CoM_height.front();
    const size_t idx_clamped =
        std::min(static_cast<size_t>(std::max(mpc_state_.Index, 0)), mpc_state_.CoM_height.size() - 1);
    m_com_height_raw_hidx_logged_ = mpc_state_.CoM_height[idx_clamped];

    m_mpc_index_logged_ = mpc_state_.Index;
    m_mpc_thread_process_time_logged_ = mpc_thread_process_time;
    m_x0_support_z_logged_ = X_0_support.translation().z();
  }

  p_com_logged_ = p_com;
  Vc_logged_ = Vc;
  acc_com_logged_ = acc_com;

  comTask->com(p_com);
  comTask->refVel(Vc);
  comTask->refAccel(acc_com);

  mc_tasks::lipm_stabilizer::ContactState supportFoot = supportFootName == leftFootName_
                                                            ? mc_tasks::lipm_stabilizer::ContactState::Left
                                                            : mc_tasks::lipm_stabilizer::ContactState::Right;

  stabTask->supportFoot(supportFoot);
}

void Walking_controller::UpdateInitialVectors()
{
  mpc_state_.t_k = t_k;
  mpc_state_.t_lift = t_lift;
  mpc_state_.doubleSupport = doubleSupport_state;
  mpc_state_.t = static_cast<double>(count) * controller_timestep;
  mpc_state_.X_0_Step_Target = X_0_SwingFootTarget;

  Eigen::Vector3d filteredNetForce = stabTask->measuredFilteredNetForces();
  mpc_state_.input_mass = filteredNetForce.z() / mc_rtc::constants::GRAVITY;

  if(debugMode)
  {
    debugCoM.z() = controller_config_.stab_config.comHeight;
    debugZMP.z() = 0;
    mpc_state_.v_c_k = Eigen::Vector3d::Zero();
    mpc_state_.p_c_k = debugCoM;
    mpc_state_.p_z_k = debugZMP;
    mpc_state_.p_u = mpc_state_.p_c_k + mpc_state_.v_c_k / mpc_state_.getEta(static_cast<size_t>(mpc_state_.Index));
    mpc_state_.t_k = debugTk;
    mpc_state_.doubleSupport = debugDblSupp;
    return;
  }

  if(UseMPCState && mpc_state_.X_MPC.size() != 0)
  {
    mpc_state_.p_c_k = mpc_state_.Get_CoM_planarTarget(mpc_state_.Index);
    mpc_state_.v_c_k = mpc_state_.Get_CoMVel_planarTarget(mpc_state_.Index);
    mpc_state_.p_z_k = mpc_state_.Get_ZMP_planarTarget(mpc_state_.Index);
    mpc_state_.p_u = mpc_state_.p_c_k + mpc_state_.v_c_k / mpc_state_.getEta(static_cast<size_t>(mpc_state_.Index));
  }
  else
  {
    mpc_state_.p_c_k = robot().com();
    mpc_state_.v_c_k = robot().comVelocity();
    mpc_state_.p_u = mpc_state_.p_c_k + mpc_state_.v_c_k / mpc_state_.getEta(static_cast<size_t>(mpc_state_.Index));
  }
  if(UseRealRobot)
  {

    sva::PTransformd zmp_frame = robot().surfacePose(supportFootName);
    sva::ForceVecd measured_net_wrench = robot().netWrench({"LeftFootForceSensor"});
    if(supportFootName == "RightFootCenter")
    {
      measured_net_wrench = robot().netWrench({"RightFootForceSensor"});
    }
    if(doubleSupport_state)
    {
      measured_net_wrench = robot().netWrench({"RightFootForceSensor", "LeftFootForceSensor"});
      zmp_frame = sva::interpolate(robot().surfacePose(supportFootName), robot().surfacePose(swingFootName), 0.5);
    }
    Eigen::Vector3d zmp_vel = mpc_state_.p_z_k;
    robot().zmp(mpc_state_.p_z_k, measured_net_wrench, zmp_frame);
    zmp_vel = (mpc_state_.p_z_k - zmp_vel) / controller_timestep;
    zmp_vel_.append(zmp_vel);

    mpc_state_.v_c_k = realRobot().comVelocity();
    mpc_state_.ComBias.segment(0, 2) = stabTask->biasDCM();
    // Time-Varying Fix: realRobot().com() carries the robot's actual measured z, which is
    // physically meaningful and must NOT be overwritten with the constant comHeight below.
    mpc_state_.p_c_k = realRobot().com() + mpc_state_.ComBias;
    const sva::PTransformd X_0_c = sva::PTransformd(mpc_state_.p_c_k);
    measured_wrench_ = X_0_c.dualMul(measured_net_wrench);

    mpc_state_.p_u = mpc_state_.p_c_k + mpc_state_.v_c_k / mpc_state_.getEta(static_cast<size_t>(mpc_state_.Index));
    if(controller_config_.stab_config.dcmBias.withDCMFilter)
    {
      mpc_state_.p_u.segment(0, 2) = -stabTask->filteredDCM();
      mpc_state_.p_c_k = mpc_state_.p_u - (mpc_state_.v_c_k / stabTask->omega());
    }
    mpc_state_.Lck = rbd::computeCentroidalMomentum(realRobot().mb(), realRobot().mbc(), mpc_state_.p_c_k).moment();
    Ldot = rbd::computeCentroidalMomentumDot(realRobot().mb(), realRobot().mbc(), mpc_state_.p_c_k, mpc_state_.v_c_k)
               .moment();

    // mpc_state_.Lck = rbd::computeCentroidalMomentum(robot().mb(), robot().mbc(), robot().com()).moment();
    // Ldot =
    //     rbd::computeCentroidalMomentumDot(robot().mb(), robot().mbc(), robot().com(), robot().comVelocity()).moment();
  }

  if(mpc_state_.X_MPC.size() != 0 && !UseRealRobot)
  {

    mpc_state_.p_z_k = mpc_state_.Get_ZMP_planarTarget(mpc_state_.Index);
  }

  if(!debugMode && UseRealRobot)
  {
    ComputePerturbances(w_, kappa_, w_inf_, kappa_inf_);
    stabTask->setExternalWrenches(
        {leftHandName_, rightHandName_},
        {robot().frame(leftHandName_).forceSensor().wrench(), robot().frame(rightHandName_).forceSensor().wrench()},
        {sva::MotionVecd(Eigen::Vector6d::Ones()), sva::MotionVecd(Eigen::Vector6d::Ones())});
  }

  // eta2_cstr = (mc_rtc::constants::GRAVITY/controller_config_.stab_config.comHeight);

  // Time-Varying Fix: p_c_k.z() used to be unconditionally hardcoded to the constant comHeight here,
  // which silently overwrote both the time-varying MPC target (from Get_CoM_planarTarget, branch above)
  // and the actual measured robot CoM z (from realRobot().com(), UseRealRobot branch above).
  // We now only fall back to the constant comHeight when no MPC trajectory exists yet (cold start /
  // not walking) and we are not using the real robot's measured z.
  if(!UseRealRobot && !(UseMPCState && mpc_state_.X_MPC.size() != 0))
  {
    mpc_state_.p_c_k.z() = controller_config_.stab_config.comHeight;
  }
  mpc_state_.v_c_k.z() = 0;
  mpc_state_.p_z_k.z() = 0;

  mpc_state_.Uk.setZero();
  if(mpc_state_.X_MPC.size() != 0)
  {
    mpc_state_.Uk = stabTask->distribZMP();
  }
}

void Walking_controller::reset(const mc_control::ControllerResetData & reset_data)
{
  // mc_rtc::log::warning(
  //     "[reset] ENTER Robot_Walking={} active={} Stop={} t_k={} count={} ref_vel=({},{},{}) N_Steps={}",
  //     Robot_Walking, active, Stop, t_k, count, reference_velocity.x(), reference_velocity.y(),
  //     reference_velocity.z(), N_Steps);

  // DIAGNOSTIC: read MPCSolver's state exactly as inherited from the previous
  // episode, before anything below (including ResetEpisodeState() further
  // down) touches it. Confirmed via this print: NextOptimalTs/m_timestamp
  // are already NaN at this point on episodes that go on to spam "ZMP
  // cannot be computed" -- the corruption originates in the PREVIOUS
  // episode's terminal ticks, not in anything reset() itself does or fails
  // to do at kinematic-state level. Kept post-fix as a regression check:
  // this should never print NextOptimalTs=nan again once ResetEpisodeState()
  // is wired in below and working.
  {
    // const auto ts = MPCSolver.timesteps();
    // mc_rtc::log::warning("[reset][solver_inherited] NextOptimalTs={} m_feas_res={} timesteps_empty={} "
    //                       "timesteps_front={}",
    //                       MPCSolver.PeekNextOptimalTs(), MPCSolver.PeekFeasRes(), ts.empty(),
    //                       ts.empty() ? 0.0 : ts.front());
  }
  // FULL STATE DUMP for episode-boundary proof: tag includes `count` (this episode's
  // pre-reset tick count, i.e. how long the PREVIOUS episode ran) so consecutive
  // dumps can be paired up in the log and diffed directly, rather than inferred.
  // Called here, at reset() ENTRY, BEFORE ResetEpisodeState() below runs -- this
  // captures exactly what MPCSolver inherited from the end of the previous episode.
  // MPCSolver.DumpState("reset_enter_prevCount" + std::to_string(count));
  // mpc_state_.DumpState("reset_enter_prevCount" + std::to_string(count) + "_mpcstate");
  // mpc_thread_state.DumpState("reset_enter_prevCount" + std::to_string(count) + "_mpcthreadstate");

  mc_control::fsm::Controller::reset(reset_data);

  bool observerResetOk = resetObserverPipelines();
  bool observerRunOk = runObserverPipelines();

  stabTask->reset();
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration config_stab = controller_config_.stab_config;
  config_stab.comWeight = 0;
  stabTask->configure(config_stab);

  comTask->reset();
  leftSwingFootTask->reset();
  rightSwingFootTask->reset();
  leftLandingTask->reset();
  rightLandingTask->reset();
  momentumTask->reset();

  swingFootTask.reset();
  SupportFootTask.reset();
  supportFootName = rightFootName_;
  swingFootName = leftFootName_;

  mpc_state_.input_v_.clear();
  mpc_state_.input_timesteps_.clear();
  mpc_state_.input_ref_pose_.clear();

  // NEW: reset walking-phase bookkeeping. These are Walking_controller members
  // (NOT part of mpc_state_, so ClearSolveState() below does not touch them) and
  // drive both the local phase clock t = (count - countStart) * controller_timestep
  // and mpc_state_.t_k, which UpdatePlanner_input() feeds to the pendulum
  // feasibility/timing solver as a timing horizon. Leaving these stale after a
  // reset means the freshly-restarted MPC thread solves a timing problem
  // relative to an old episode's clock, producing exactly the "broken cstr" /
  // huge offset-delta QP failures seen right after reset.
  count = 0;
  countStart = 0;
  t = 0;
  t_k = -controller_config_.delta;
  count_stop = 0;
  t_stop = 0;
  kfoot = 0;
  mpc_state_.Index = 0;
  mpc_thread_state.Index = 0;

  SupportFootPose = robot().surfacePose(supportFootName).translation();
  SupportFootPose.z() = 0;

  mpc_state_.p_c_k = robot().com();
  mpc_state_.p_c_k.z() = controller_config_.stab_config.comHeight;
  mpc_state_.p_z_k = robot().surfacePose(swingFootName).translation();
  mpc_state_.p_z_k.z() = 0;
  mpc_state_.p_u = mpc_state_.p_c_k;
  mpc_state_.v_c_k = robot().comVelocity();

  // NEW: clear all solve-derived horizon state on both copies, so
  // UpdateInitialVectors()/MoveCoM() cannot read a stale trajectory from
  // the previous episode before the MPC thread produces a fresh one.
  mpc_state_.ClearSolveState();
  mpc_thread_state.ClearSolveState();
  // NEW: MPCSolver (an ISMPC_Solver) is a single member of Walking_controller
  // constructed once for the whole controller lifetime -- it is never
  // reconstructed per-episode, so without this call it silently inherits
  // NextOptimalTs/m_timestamp/m_feas_res/m_Tds from the end of the PREVIOUS
  // episode's last GetWalkingParameters() call, including a possible NaN.
  // Confirmed via [reset][solver_inherited] prints above: this is the
  // direct, sole cause of the permanent "ZMP cannot be computed" spam --
  // once NextOptimalTs is NaN, (NextOptimalTs - m_tk) > 0.1 is false forever
  // (IEEE-754 comparisons against NaN are always false), so the feasibility
  // solver is never called again and the same NaN m_timestamp[0] is reused
  // every tick for the rest of the episode. This must run BEFORE the
  // p_c_k/p_z_k/p_u/v_c_k re-apply below, order doesn't matter relative to
  // it, but must be present every reset.
  MPCSolver.ResetEpisodeState();
  // Second dump, same tag family, immediately after ResetEpisodeState(): proves
  // (rather than assumes) that the reset actually took effect this call, in the
  // same log, right next to the pre-reset "inherited" dump above. NOTE: count is
  // already 0 by this point (see count = 0 above) -- this tag intentionally does
  // NOT include prevCount, to avoid implying it's still meaningful here.
  // MPCSolver.DumpState("reset_after_ResetEpisodeState");
  // Re-apply the fresh p_c_k/p_z_k/p_u/v_c_k set just above, since
  // ClearSolveState() zeroes them along with everything else.
  mpc_state_.p_c_k = robot().com();
  mpc_state_.p_c_k.z() = controller_config_.stab_config.comHeight;
  mpc_state_.p_z_k = robot().surfacePose(swingFootName).translation();
  mpc_state_.p_z_k.z() = 0;
  mpc_state_.p_u = mpc_state_.p_c_k;
  mpc_state_.v_c_k = robot().comVelocity();

  filter_left_hand_wrench_ = mc_filter::LowPass<sva::ForceVecd>(solver().dt(), controller_config_.wrench_filter_cutoff);
  filter_right_hand_wrench_ =
      mc_filter::LowPass<sva::ForceVecd>(solver().dt(), controller_config_.wrench_filter_cutoff);
  filter_gamma_ = mc_filter::LowPass<Eigen::Vector3d>(solver().dt(), controller_config_.gamma_filter_cutoff);

  swing_foot_initial_pose = robot().surfacePose(swingFootName).translation();
  X_0_SwingFootInitial = swing_foot_initial_pose;
  updateTasks();

  addContact({robot().name(), "ground", rightFootName_, "AllGround", 0.7, footcontact_dof});
  addContact({robot().name(), "ground", leftFootName_, "AllGround", 0.7, footcontact_dof});

  MPC_thread_on = false;
  MPC_thread_ready = false;
  if(walkingTrajectoryThread.joinable())
  {
    compute_trajectory_once.notify_all();
    walkingTrajectoryThread.join();
  }

  // NEW: activate()/deactivate() are both gated on `if(!Robot_Walking)` (see
  // their definitions) -- they are no-ops whenever Robot_Walking is true.
  // Robot_Walking is only ever written inside run() (never reset here), so
  // if the previous episode terminated mid-stride (the common case: falls
  // happen while walking, not while standing), it is still true at this
  // point and the activate()/deactivate() calls below would silently do
  // nothing, leaving `active` stuck at whatever it was at termination --
  // e.g. permanently false if MoveCoM()'s "Control Horizon reached" path
  // had called deactivate() before the episode ended. Explicitly clear it
  // here so activate()/deactivate() actually take effect on every reset,
  // matching the intent of the autoStartConfigured re-arm below.
  Robot_Walking = false;

  // NEW: reference_velocity is a Walking_controller member, set at
  // construction (setZero(), then possibly overwritten by auto_start.speed)
  // but never touched by reset() -- so after the first episode it holds
  // whatever the last set_reference_velocity() bridge call wrote, i.e. the
  // PREVIOUS episode's last-sampled twist command, not zero and not the new
  // episode's freshly-resampled one (the Python side only pushes the new
  // twist from apply_actions(), which runs strictly after this reset() call
  // returns). UpdatePlanner_input() feeds this straight into step_velocity,
  // which seeds the footstep-planner/pendulum-solver's velocity horizon on
  // the very first post-reset solve -- combined with the CoM/support state
  // that was JUST reset to the fresh nominal pose, a large stale velocity
  // here is a footstep target wildly inconsistent with the actual reset
  // state, which is consistent with the large multi-constraint QP breaks
  // seen immediately after reset. Zero it here, matching the constructor's
  // own initialization; N_Steps is reset alongside it for the same reason
  // (see Steps/Steps_Desired in ComputeWalkingTrajectory(), also never
  // reset elsewhere).
  reference_velocity.setZero();
  N_Steps = 0;

  if(autoStartConfigured)
  {
    activate();
    // Policy has full, unconditional authority over walking from tick 0 --
    // no hardcoded settle window. Default to not walking (Stop=true) purely
    // as a safe starting point until the policy's first SetPolicyWantsWalk()
    // call; the policy may request walking immediately if it chooses to.
    // Walking_controller stays `active` (stabilizer/CoM tracking engaged,
    // MoveCoM() still runs every tick) regardless -- this only concerns
    // MoveFeet()/stepping, gated behind `!(Stop && doubleSupport_state)`.
    policyWantsWalk = false;
    Stop = true;
    N_Steps_Desired = N_Steps_Desired_std;
  }
  else
  {
    deactivate();
    policyWantsWalk = false;
    Stop = true;
  }
  autoStart = false;

  // mc_rtc::log::warning(
  //     "[reset] EXIT  Robot_Walking={} active={} Stop={} t_k={} count={} ref_vel=({},{},{}) N_Steps={} "
  //     "autoStartConfigured={}",
  //     Robot_Walking, active, Stop, t_k, count, reference_velocity.x(), reference_velocity.y(),
  //     reference_velocity.z(), N_Steps, autoStartConfigured);
}