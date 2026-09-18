#include "../include/ismpc_walking/Walking_controller.h"

bool Walking_controller::MoveFeet(double t)
{

  prevStepTiming = 0;
  double nextTimeStep(0);

  nextTimeStep = mpc_state_.get_Ts(static_cast<size_t>(kfoot));

  // if(kfoot != 0)
  // {
  //   prevStepTiming = mpc_state_.get_Ts(kfoot-1);
  //   X_0_SwingFootInitial = mpc_state_.Get_CorrectedFootstep(kfoot-1) ;
  // }

  if(kfoot < mpc_state_.optimal_steps_.size())
  {
    X_0_SwingFootTarget = mpc_state_.Get_CorrectedFootstep(kfoot);
  }
  else
  {
    X_0_SwingFootTarget = mpc_state_.Get_PlannedFootstep(kfoot);
  }

  Eigen::Vector3d supportFootPose = robot().surfacePose(supportFootName).translation();
  supportFootPose.z() = mc_rbdyn::rpyFromMat(robot().surfacePose(supportFootName).rotation()).z();

  std::string swingFoot_sensorName = "RightFootForceSensor";
  std::string supportFoot_sensorName = "LeftFootForceSensor";
  if(swingFootName == leftFootName_)
  {
    swingFoot_sensorName = "LeftFootForceSensor";
    supportFoot_sensorName = "RightFootForceSensor";
  }
  const auto & sensor = robot().forceSensor(swingFoot_sensorName);
  const auto & sensor_support = robot().forceSensor(supportFoot_sensorName);

  if(doubleSupport_state)
  {
    t_lift = prevStepTiming + mpc_state_.get_tds();
    if(t >= prevStepTiming + mpc_state_.get_tds())
    {

      mc_rtc::log::success("lifting " + swingFootName);
      solver().addTask(swingFootTask);
      Eigen::Vector3d supp_pose= realRobot().surfacePose(supportFootName).translation();
      Eigen::Vector3d swing_pose = realRobot().surfacePose(swingFootName).translation();

      double supp_yaw;
      if(std::abs((supp_pose - swing_pose).z()) < 0.01)
      {
        supp_pose.z() = 0;
      }
      // supp_pose.z() = 0.0;
      supp_yaw = mc_rbdyn::rpyFromMat(robot().surfacePose(supportFootName).rotation()).z();
      if(supportFootName == leftFootName_)
      {
        stabTask->setContacts(
            {{mc_tasks::lipm_stabilizer::ContactState::Left, sva::PTransformd(sva::RotZ(supp_yaw), supp_pose)}});
      }
      else
      {
        stabTask->setContacts(
            {{mc_tasks::lipm_stabilizer::ContactState::Right, sva::PTransformd(sva::RotZ(supp_yaw), supp_pose)}});
      }

      removeContact({robot().name(), "ground", swingFootName, "AllGround", 0.7, footcontact_dof});


      t_lift = t;

      doubleSupport_state = false;
      swing_foot_contact = false;
      // Recompute MPC once contact is released
      //{
      t_k += t - t_k;
      compute_trajectory_once.notify_all();
      //}
    }
  }

  const double offset = 0;
  const double singleSupportDuration = (nextTimeStep - t_lift) + offset;
  const double height_off = X_0_support.translation().z();
  swingFootTrajectory.setZOffset(controller_config_.foot_landing_offset + height_off);

  swingFootTrajectory.getSwingFootTrajectory(X_0_SwingFootTarget, X_0_SwingFootInitial, t,
                                             controller_config_.footStepHeight, singleSupportDuration, t_lift,
                                             controller_timestep);

  swingFootVel = swingFootTrajectory.GetVelocity().linear();
  swingFootAcc = swingFootTrajectory.GetAccel().linear();
  sva::PTransformd X_0_FootTask_Target = swingFootTrajectory.GetTrajectory();

  sva::MotionVecd V_0_FootTask_Target = swingFootTrajectory.GetVelocity();
  sva::MotionVecd A_0_FootTask_Target = swingFootTrajectory.GetAccel();

  double swing_yaw = mc_rbdyn::rpyFromMat(X_0_swing.rotation()).z();

  swingFootTask->target(X_0_FootTask_Target);
  swingFootTask->refVelB(sva::PTransformd(sva::RotZ(swing_yaw)) * V_0_FootTask_Target);
  swingFootTask->refAccel(
      sva::MotionVecd(Eigen::Vector3d::Zero(), sva::RotZ(swing_yaw) * A_0_FootTask_Target.linear()));
  // swingFootTask->refAccel(sva::PTransformd(X_0_swing.rotation())
  // *sva::MotionVecd(Eigen::Vector3d::Zero(),A_0_FootTask_Target.linear()));

  if(!doubleSupport_state)
  {
    if(controller_config_.footStepHeight - X_0_FootTask_Target.translation().z() < 0.005)
    {
      double vertical_offset_measure = swingFootTask->frame().wrench().force().z();
      vertical_force_offset_ =
          (vertical_force_offset_ * static_cast<double>(vertical_force_measure_cnt_) + vertical_offset_measure)
          / (static_cast<double>(vertical_force_measure_cnt_) + 1);
      vertical_force_measure_cnt_++;
    }

    double step_time = t - t_lift;
    double swing_foot_height = robot().surfacePose(swingFootName).translation().z();
    double support_foot_height = robot().surfacePose(supportFootName).translation().z();
    double foot_diff_height =
        (realRobot().surfacePose(swingFootName).translation() - realRobot().surfacePose(supportFootName).translation())
            .z();

    bool TouchDown = (robot().frame(swingFootName).wrench().force().z() - vertical_force_offset_
                      > controller_config_.impact_threshold);
    // TouchDown = false;

    if(((step_time > 0.2 && TouchDown) || step_time >= singleSupportDuration) && !swing_foot_contact)

    {

      t_contact = t;
      solver().removeTask(swingFootTask);

      solver().addTask(landingTask);
      landingTask->targetPose(landingTask->surfacePose());
      landingTask->admittance(sva::ForceVecd(Eigen::Vector3d{0.03, 0.03, 0}, Eigen::Vector3d{0, 0, 0.01}));
      landingTask->targetCoP(Eigen::Vector2d::Zero());
      landingTask->targetForce(Eigen::Vector3d{0, 0, 50});

      swing_foot_contact = true;
      mc_rtc::log::info("height : {} ", swing_foot_height);
      mc_rtc::log::info("touchdown : {} ", TouchDown);
      landing_time = 0;
      if(!TouchDown && !tickerMode)
      {
        mc_rtc::log::warning("[Walking Controller] Contact without forces measured, stoping");
        landing_time = controller_config_.landing_time;
        // Stop = true;
      }
      // mc_rtc::log::info("Locking " + swingFootName + "at t : " + std::to_string(t));
      mc_rtc::log::info("T_contact - T_steps : {}", t - nextTimeStep);

      mc_rtc::log::success("Locked " + swingFootName);

      if((X_0_SwingFootTarget.translation() - robot().surfacePose(swingFootName).translation()).norm() > 0.25
         && !StepRecoveryState)
      {
        mc_rtc::log::error("Contact occured too far from reference, stop triggered");
        Stop = true;
      }
      // if( std::abs( mc_rbdyn::rpyFromMat(realRobot().surfacePose(swingFootName).rotation()).x() -
      // mc_rbdyn::rpyFromMat(robot().surfacePose(swingFootName).rotation()).x() ) >
      // controller_config_.safety_roll_error_ )
      // {
      //   mc_rtc::log::error("Robot is about to fall, stoping");
      //   Stop = true;
      // }

      // prevStepTiming = nextTimeStep;
      // if(kfoot + 1 < mpc_state_.planned_steps().size())
      // {
      //   kfoot += 1;
      // }
    }

    if(swing_foot_contact && !doubleSupport_state && ( t - t_contact >= landing_time || robot().frame(swingFootName).wrench().force().z() > 100 ) )
    {
      solver().removeTask(landingTask);
      mc_rtc::log::info("Landing over {}, Touchdown 2 {}",t - t_contact, robot().frame(swingFootName).wrench().force().z() > 100);
      Eigen::Vector3d supp_pose;
      double supp_yaw;
      supp_pose = realRobot().surfacePose(supportFootName).translation();
      // supp_pose.z() = 0.0;
      supp_yaw = mc_rbdyn::rpyFromMat(robot().surfacePose(supportFootName).rotation()).z();
      Eigen::Vector3d swing_pose;
      double swing_yaw;
      swing_pose = realRobot().surfacePose(swingFootName).translation();
      // swing_pose.z() = 0.0;
      swing_yaw = mc_rbdyn::rpyFromMat(robot().surfacePose(swingFootName).rotation()).z();
      if(supportFootName == leftFootName_)
      {
        stabTask->setContacts(
            {{mc_tasks::lipm_stabilizer::ContactState::Left, sva::PTransformd(sva::RotZ(supp_yaw), supp_pose)},
             {mc_tasks::lipm_stabilizer::ContactState::Right, sva::PTransformd(sva::RotZ(swing_yaw), swing_pose)}});
      }
      else
      {
        stabTask->setContacts(
            {{mc_tasks::lipm_stabilizer::ContactState::Left, sva::PTransformd(sva::RotZ(swing_yaw), swing_pose)},
             {mc_tasks::lipm_stabilizer::ContactState::Right, sva::PTransformd(sva::RotZ(supp_yaw), supp_pose)}});
      }
      addContact({robot().name(), "ground", swingFootName, "AllGround", 0.7, footcontact_dof});

      switchFootSupport();
      updateTasks();

      N_Steps += 1;

      doubleSupport_state = true;
      if(N_Steps == N_Steps_Desired)
      {
        Stop = true;
      }
      t_k = -controller_config_.delta;
      countStart = count + 1;
    }
  }

  if(doubleSupport_state)
  {

    swing_foot_initial_pose = robot().surfacePose(swingFootName).translation();
    SwingFootInitialAngle = mc_rbdyn::rpyFromMat(robot().surfacePose(swingFootName).rotation()).z();

    X_0_SwingFootInitial =
        sva::PTransformd(sva::RotZ(mc_rbdyn::rpyFromMat(robot().surfacePose(swingFootName).rotation()).z()),
                         robot().surfacePose(swingFootName).translation());

    X_0_SwingFootInitial = robot().surfacePose(swingFootName);

    // if(vertical_force_measure_cnt_ > 0)
    // {
    //   mc_rbdyn::ForceSensor & sensor = robot().sensor<mc_rbdyn::ForceSensor>(robot().frame(supportFootName).forceSensor().name());
    //   auto calib = sensor.calib();
    //   // FIXME Maybe X_frame_sensor * Eigen::Vector3d(0, 0, offset)
    //   calib.offset.force().z() += vertical_force_offset_;
    //   sensor.loadCalibrator(calib);
    // }

    vertical_force_offset_ = 0;
    vertical_force_measure_cnt_ = 0;

    // stabTask->setExternalWrenches({},{},{});
    filter_gamma_.reset(Eigen::Vector3d::Zero());
    solver().removeTask(leftSwingFootTask);
    solver().removeTask(rightSwingFootTask);
  }

  return 0;
}

void Walking_controller::updateTasks()
{
  if(supportFootName == leftFootName_)
  {
    swingFootTask = rightSwingFootTask;
    landingTask = rightLandingTask;
  }
  else
  {
    swingFootTask = leftSwingFootTask;
    landingTask = leftLandingTask;
  }
  Eigen::MatrixXd dimW(Eigen::VectorXd::Zero(6));
  dimW(5) = 1;
  swingFootTask->weight(controller_config_.SwingFootWeight);
  swingFootTask->dimWeight(Eigen::VectorXd::Ones(6));
  swingFootTask->stiffness(controller_config_.SwingFootStiffness);
}

void Walking_controller::switchFootSupport()
{
  {
    std::lock_guard<std::mutex> lk(mutex_mpc_);
    if(supportFootName == leftFootName_)
    {
      supportFootName = rightFootName_;
      swingFootName = leftFootName_;
    }
    else
    {
      supportFootName = leftFootName_;
      swingFootName = rightFootName_;
    }
  }

  mc_tasks::lipm_stabilizer::ContactState supportFoot = supportFootName == leftFootName_
                                                            ? mc_tasks::lipm_stabilizer::ContactState::Left
                                                            : mc_tasks::lipm_stabilizer::ContactState::Right;
  stabTask->supportFoot(supportFoot);
}
