#include "../include/ismpc_walking/Walking_controller.h"

inline double floorn(double x, int n)
{
  return floor(pow(10, n) * x) / pow(10, n);
}

// FIXME This should be part of mc_rtc
inline void AddStabilizerConfigToGUI(mc_rtc::gui::StateBuilder & gui,
                                     std::vector<std::string> category,
                                     mc_rbdyn::lipm_stabilizer::StabilizerConfiguration & c_)
{
  category.push_back("Main");
  gui.addElement(
      category,
      mc_rtc::gui::NumberInput(
          "CoM weight", [&c_]() { return c_.comWeight; }, [&c_](double d) { c_.comWeight = d; }),
      mc_rtc::gui::ArrayInput(
          "CoM stiffness", {"x", "y", "z"}, [&c_]() -> Eigen::Vector3d { return c_.comStiffness; },
          [&c_](const Eigen::Vector3d & d) { c_.comStiffness = d; }),
      mc_rtc::gui::ArrayInput(
          "Foot admittance", {"CoPx", "CoPy"}, [&c_]() -> Eigen::Vector2d { return c_.copAdmittance; },
          [&c_](const Eigen::Vector2d & a) { c_.copAdmittance = a; }),
      mc_rtc::gui::ArrayInput(
          "Foot CoP lambda", {"CoPx", "CoPy", "Fz"}, [&c_]() -> Eigen::Vector3d { return c_.copFzLambda; },
          [&c_](const Eigen::Vector3d & a) { c_.copFzLambda = a; }),
      mc_rtc::gui::NumberInput(
          "CoM height", [&c_]() { return c_.comHeight; }, [&c_](double d) { c_.comHeight = d; }),
      // mc_rtc::gui::ArrayInput(
      //     "CoP Lambda Support Foot", {"CoPx", "CoPy"},
      //     [&c_]() -> Eigen::Vector2d {
      //       return c_.lambdaCoPSupportFoot;
      //     },
      //     [&c_](const Eigen::Vector2d & a) { c_.lambdaCoPSupportFoot = a; }),
      // mc_rtc::gui::ArrayInput(
      //     "CoP slope", {"CoPx", "CoPy"},
      //     [&c_]() -> Eigen::Vector2d {
      //       return c_.copSlope;
      //     },
      //     [&c_](const Eigen::Vector2d & a) { c_.copSlope = a; }),
      mc_rtc::gui::NumberInput(
          "Admittance Delay", [&c_]() { return c_.delayCoP; }, [&c_](double d) { c_.delayCoP = d; }),
      // mc_rtc::gui::ArrayInput(
      //     "Foot force difference", {"Admittance", "Damping"},
      //     [&c_]() -> Eigen::Vector2d {
      //       return {c_.dfzAdmittance, c_.dfzDamping};
      //     },
      //     [&c_](const Eigen::Vector2d & a) {
      //       c_.dfzAdmittance = a(0);
      //       c_.dfzDamping = a(1);
      //     }),
      mc_rtc::gui::ArrayInput(
          "Foot force difference Admittance", {"Fx", "Fy", "Fz"},
          [&c_]() -> Eigen::Vector3d { return c_.dfAdmittance; },
          [&c_](const Eigen::Vector3d & a) { c_.dfAdmittance = a; }),
      // mc_rtc::gui::ArrayInput(
      //     "Support Foot force difference Admittance", {"Fx", "Fy", "Fz"},
      //     [&c_]() -> Eigen::Vector3d { return c_.dfAdmittanceSupportFoot; },
      //     [&c_](const Eigen::Vector3d & a) { c_.dfAdmittanceSupportFoot = a; }),
      mc_rtc::gui::ArrayInput(
          "Foot force difference Damping", {"Fx", "Fy", "Fz"}, [&c_]() -> Eigen::Vector3d { return c_.dfDamping; },
          [&c_](const Eigen::Vector3d & a) { c_.dfDamping = a; }),
      mc_rtc::gui::NumberInput(
          "CoMd Error gain", [&c_]() { return c_.comdErrorGain; }, [&c_](double a) { c_.comdErrorGain = a; }),
      mc_rtc::gui::NumberInput(
          "ZMPd gain", [&c_]() { return c_.zmpdGain; }, [&c_](double a) { c_.zmpdGain = a; }),
      mc_rtc::gui::ArrayInput(
          "DCM filters", {"Integrator T [s]", "Derivator T [s]"},
          [&c_]() -> Eigen::Vector2d {
            return {c_.dcmDerivatorTimeConstant, c_.dcmIntegratorTimeConstant};
          },
          [&c_](const Eigen::Vector2d & T) {
            c_.dcmDerivatorTimeConstant = T(0);
            c_.dcmIntegratorTimeConstant = T(1);
          }),
      mc_rtc::gui::NumberInput(
          "Torso pitch [rad]", [&c_]() { return c_.torsoPitch; }, [&c_](double pitch) { c_.torsoPitch = pitch; }));
  category.pop_back();
  category.push_back("Advanced");
  gui.addElement(
      category,
      mc_rtc::gui::Checkbox(
          "Constrain CoP", [&c_]() { return c_.constrainCoP; }, [&c_]() { c_.constrainCoP = !c_.constrainCoP; }),
      mc_rtc::gui::NumberInput(
          "Admittance Velocity Filter [0-1]", [&c_]() { return c_.copVelFilterGain; },
          [&c_](double gain) { c_.copVelFilterGain = gain; }),
      mc_rtc::gui::ArrayInput(
          "Vertical drift compensation", {"frequency", "stiffness"},
          [&c_]() -> Eigen::Vector2d {
            return {c_.vdcFrequency, c_.vdcStiffness};
          },
          [&c_](const Eigen::Vector2d & v) {
            c_.vdcFrequency = v(0);
            c_.vdcStiffness = v(1);
          }),
      mc_rtc::gui::NumberInput(
          "Torso pitch [rad]", [&c_]() { return c_.torsoPitch; },
          [&c_](double pitch) {
            c_.torsoPitch = pitch;
            ;
          }),
      mc_rtc::gui::ArrayInput(
          "Admittance damping", {"wx", "wy", "wz", "vx", "vy", "vz"},
          [&c_]() -> Eigen::Vector6d { return c_.contactDamping.vector(); },
          [&c_](const Eigen::Vector6d & d) { c_.contactDamping = sva::MotionVecd(d); }));

  category.push_back("DCM Bias");
  gui.addElement(category, mc_rtc::gui::ElementsStacking::Horizontal,
                 mc_rtc::gui::Checkbox(
                     "Enabled", [&c_]() { return c_.dcmBias.withDCMBias; },
                     [&c_]() { c_.dcmBias.withDCMBias = !c_.dcmBias.withDCMBias; }),
                 mc_rtc::gui::Checkbox(
                     "Correct CoM Pos", [&c_]() { return c_.dcmBias.correctCoMPos; },
                     [&c_]() { c_.dcmBias.correctCoMPos = !c_.dcmBias.correctCoMPos; }),
                 mc_rtc::gui::Checkbox(
                     "Use Filtered DCM", [&c_]() { return c_.dcmBias.withDCMFilter; },
                     [&c_]() { c_.dcmBias.withDCMFilter = !c_.dcmBias.withDCMFilter; }));
  gui.addElement(category,
                 mc_rtc::gui::NumberInput(
                     "dcmMeasureErrorStd", [&c_]() { return c_.dcmBias.dcmMeasureErrorStd; },
                     [&c_](double v) { c_.dcmBias.dcmMeasureErrorStd = v; }),
                 mc_rtc::gui::NumberInput(
                     "zmpMeasureErrorStd", [&c_]() { return c_.dcmBias.zmpMeasureErrorStd; },
                     [&c_](double v) { c_.dcmBias.zmpMeasureErrorStd = v; }),
                 mc_rtc::gui::NumberInput(
                     "driftPerSecondStd", [&c_]() { return c_.dcmBias.biasDriftPerSecondStd; },
                     [&c_](double v) { c_.dcmBias.biasDriftPerSecondStd = v; }),
                 mc_rtc::gui::ArrayInput(
                     "Bias Limit [m]", {"sagital", "lateral"},
                     [&c_]() -> const Eigen::Vector2d & { return c_.dcmBias.biasLimit; },
                     [&c_](const Eigen::Vector2d & v) { c_.dcmBias.biasLimit = v; }),
                 mc_rtc::gui::ArrayInput(
                     "CoM bias Limit [m]", {"sagital", "lateral"},
                     [&c_]() -> const Eigen::Vector2d & { return c_.dcmBias.comBiasLimit; },
                     [&c_](const Eigen::Vector2d & v) { c_.dcmBias.comBiasLimit = v; }));
  category.pop_back();
  category.push_back("Ext Wrench");
  gui.addElement(
      category,
      mc_rtc::gui::Checkbox(
          "addExpectedCoMOffset", [&c_]() { return c_.extWrench.addExpectedCoMOffset; },
          [&c_]() { c_.extWrench.addExpectedCoMOffset = !c_.extWrench.addExpectedCoMOffset; }),
      mc_rtc::gui::Checkbox(
          "subtractMeasuredValue", [&c_]() { return c_.extWrench.subtractMeasuredValue; },
          [&c_]() { c_.extWrench.subtractMeasuredValue = !c_.extWrench.subtractMeasuredValue; }),
      mc_rtc::gui::Checkbox(
          "modifyCoMErr", [&c_]() { return c_.extWrench.modifyCoMErr; },
          [&c_]() { c_.extWrench.modifyCoMErr = !c_.extWrench.modifyCoMErr; }),
      mc_rtc::gui::Checkbox(
          "modifyZMPErr", [&c_]() { return c_.extWrench.modifyZMPErr; },
          [&c_]() { c_.extWrench.modifyZMPErr = !c_.extWrench.modifyZMPErr; }),
      mc_rtc::gui::Checkbox(
          "modifyZMPErrD", [&c_]() { return c_.extWrench.modifyZMPErrD; },
          [&c_]() { c_.extWrench.modifyZMPErrD = !c_.extWrench.modifyZMPErrD; }),
      mc_rtc::gui::Checkbox(
          "excludeFromDCMBiasEst", [&c_]() { return c_.extWrench.excludeFromDCMBiasEst; },
          [&c_]() { c_.extWrench.excludeFromDCMBiasEst = !c_.extWrench.excludeFromDCMBiasEst; }),
      mc_rtc::gui::NumberInput(
          "Limit of comOffsetErrCoM", [&c_]() { return c_.extWrench.comOffsetErrCoMLimit; },
          [&c_](double a) { c_.extWrench.comOffsetErrCoMLimit = a; }),
      mc_rtc::gui::NumberInput(
          "Limit of comOffsetErrZMP", [&c_]() { return c_.extWrench.comOffsetErrZMPLimit; },
          [&c_](double a) { c_.extWrench.comOffsetErrZMPLimit = a; }),
      mc_rtc::gui::NumberInput(
          "Cutoff period of extWrenchSumLowPass", [&c_]() { return c_.extWrench.extWrenchSumLowPassCutoffPeriod; },
          [&c_](double a) { c_.extWrench.extWrenchSumLowPassCutoffPeriod = a; }),
      mc_rtc::gui::NumberInput(
          "Cutoff period of comOffsetLowPass", [&c_]() { return c_.extWrench.comOffsetLowPassCutoffPeriod; },
          [&c_](double a) { c_.extWrench.comOffsetLowPassCutoffPeriod = a; }),
      mc_rtc::gui::NumberInput(
          "Cutoff period of comOffsetLowPassCoM", [&c_]() { return c_.extWrench.comOffsetLowPassCoMCutoffPeriod; },
          [&c_](double a) { c_.extWrench.comOffsetLowPassCoMCutoffPeriod = a; }),
      mc_rtc::gui::NumberInput(
          "Time constant of comOffsetDerivator", [&c_]() { return c_.extWrench.comOffsetDerivatorTimeConstant; },
          [&c_](double a) { c_.extWrench.comOffsetDerivatorTimeConstant = a; }));
}

void Walking_controller::addToGUI()
{

  gui()->addElement(
      {"Walking", "Main"},

      mc_rtc::gui::Button("Start Move",
                          [this]() {
                            if(stabilizer_active_)
                            {
                              Stop = false;
                              compute_trajectory_once.notify_all();
                            }
                          }),
      mc_rtc::gui::Button("Stop", [this]() { Stop = true; }),
      mc_rtc::gui::Checkbox(
          "Active", [this]() { return active; },
          [this]() {
            if(active)
            {
              deactivate();
            }
            else
            {
              activate();
            }
          }),
      mc_rtc::gui::ComboInput(
          "Velocity Tail", {"Periodic", "Truncated", "Anticipative", "None"}, [this]() { return Tail; },
          [this](const std::string str) {
            Tail = str;
            compute_trajectory_once.notify_all();
          }),
      mc_rtc::gui::Label("Velocity Tail used", [this]() { return this->mpc_state_.Tail; }),
      mc_rtc::gui::Label("Timing", [this]() { return t; }),
      mc_rtc::gui::Label("Double support duration", [this]() { return this->mpc_state_.get_tds(); }),
      mc_rtc::gui::Label("Next Step Timing ", [this]() { return this->mpc_state_.get_Ts(0); }),
      mc_rtc::gui::Checkbox(
          "Automatic FootStep Placement", [this]() { return AutoFootstepPlacement; },
          [this]() { AutoFootstepPlacement = !AutoFootstepPlacement; }),
      mc_rtc::gui::Checkbox(
          "Use Angular Momentum", [this]() { return UseAngularMomentum; },
          [this]() { UseAngularMomentum = !UseAngularMomentum; }),
      mc_rtc::gui::Checkbox(
          "Timing/Steps optimization", [this]() { return UsePendulumSolver; },
          [this]() { UsePendulumSolver = !UsePendulumSolver; }),
      mc_rtc::gui::Checkbox(
          "Real Robot Data feedback", [this]() { return UseRealRobot; },
          [this]() {
            UseRealRobot = !UseRealRobot;
            if(UseMPCState) UseMPCState = false;
            compute_trajectory_once.notify_all();
          }),
      mc_rtc::gui::Checkbox(
          "Robot Step Recovery", [this]() { return UseStepRecovery; },
          [this]() { UseStepRecovery = !UseStepRecovery; }),
      mc_rtc::gui::NumberInput(
          "lambda sg support", [this]() -> double { return controller_config_.lambda_sg_supp; },
          [this](const double n) { controller_config_.lambda_sg_supp = n; }),
      mc_rtc::gui::NumberInput(
          "lambda dbl support", [this]() -> double { return controller_config_.lambda_dbl_supp; },
          [this](const double n) { controller_config_.lambda_dbl_supp = n; }),
      mc_rtc::gui::NumberInput(
          "zmp delay", [this]() -> double { return controller_config_.zmp_delay; },
          [this](const double n) { controller_config_.zmp_delay = n; }),
      mc_rtc::gui::Checkbox(
          "MPC state feedback", [this]() { return UseMPCState; },
          [this]() {
            UseMPCState = !UseMPCState;
            if(UseRealRobot) UseRealRobot = false;
            compute_trajectory_once.notify_all();
          }),
      mc_rtc::gui::Checkbox(
          "Disturbance", [this]() { return Use_w; }, [this]() { Use_w = !Use_w; }),
      mc_rtc::gui::ArrayInput(
          "Input Disturbance", [this]() { return w_; }, [this](const Eigen::Vector3d & in) { w_ = in; }),
      mc_rtc::gui::NumberInput(
          "Input Kappa", [this]() -> double { return kappa_; }, [this](double w) { kappa_ = w; }),
      mc_rtc::gui::ArrayInput(
          "Input Disturbance Infinity", [this]() { return w_inf_; },
          [this](const Eigen::Vector3d & in) { w_inf_ = in; }),
      mc_rtc::gui::NumberInput(
          "Input Kappa Infinity", [this]() -> double { return kappa_inf_; }, [this](double w) { kappa_inf_ = w; }),
      // mc_rtc::gui::Checkbox(
      //     "Force Contact Safety", [this]() { return force_contact_safety_; },
      //     [this]() { force_contact_safety_ = !force_contact_safety_; }),
      mc_rtc::gui::Checkbox(
          "Ticker Mode", [this]() { return tickerMode; },
          [this]() { tickerMode = !tickerMode; }),
      mc_rtc::gui::Checkbox(
          "Increase MPC update rate (Double Support)", [this]() { return IncreaseUpdate; },
          [this]() { IncreaseUpdate = !IncreaseUpdate; }),
      // mc_rtc::gui::Checkbox("ZMP_Corr", [this](){return ZMP_correction;}, [this](){ZMP_correction =
      // !ZMP_correction;}),
      mc_rtc::gui::Label("Stab Error (m)", [this]() { return this->mpc_state_.stab_error; }),
      mc_rtc::gui::Label("MPC Processing Time (ms)", [this]() { return this->mpc_thread_process_time; }),
      mc_rtc::gui::Label("Run Loop Processing Time (ms)", [this]() { return this->ControllerLoopTime; }),
      // mc_rtc::gui::Label("ZMP box range x",[this](){return this->MPCSolver.ZMP_dx;}),
      // mc_rtc::gui::Label("ZMP box range y",[this](){return this->MPCSolver.ZMP_dy;})
      mc_rtc::gui::ComboInput(
        "CoM Height Signal", {"RL (default)", "Sine", "PerStepCosine", "Step"},
        [this]() { return ToString(MPCSolver.TestSignal()); },
        [this](const std::string & str) { MPCSolver.TestSignal(FromString(str)); }),
      mc_rtc::gui::Form(
        "CoM Height: Sine",
        [this](const mc_rtc::Configuration & data) {
        MPCSolver.SetSineParams(data("CoM avg height"), data("Amplitude"), data("Period"));
        },
        mc_rtc::gui::FormNumberInput("CoM avg height", false, [this]() { return MPCSolver.CoMHeightAvg(); }),
        mc_rtc::gui::FormNumberInput("Amplitude", false, [this]() { return MPCSolver.CoMHeightAmplitude(); }),
        mc_rtc::gui::FormNumberInput("Period", false, [this]() { return MPCSolver.CoMHeightPeriod(); })),
      mc_rtc::gui::Form(
        "CoM Height: PerStepCosine",
        [this](const mc_rtc::Configuration & data) {
        MPCSolver.SetPerStepCosineParams(data("CoM avg height"), data("Amplitude"));
        },
        mc_rtc::gui::FormNumberInput("CoM avg height", false, [this]() { return MPCSolver.CoMHeightAvg(); }),
        mc_rtc::gui::FormNumberInput("Amplitude", false, [this]() { return MPCSolver.CoMHeightAmplitude(); })),
      mc_rtc::gui::Form(
        "CoM Height: Step",
        [this](const mc_rtc::Configuration & data) {
        double now = MPCSolver.GlobalTime();
        double delay = data("Timing (s from now)");
        MPCSolver.SetStepParams(data("Height before"), data("Amplitude"), now + delay);
        },
        mc_rtc::gui::FormNumberInput("Height before", false, [this]() { return MPCSolver.CoMHeightAvg(); }),
        mc_rtc::gui::FormNumberInput("Timing (s from now)", false, 1.0),
        mc_rtc::gui::FormNumberInput("Amplitude", false, [this]() { return MPCSolver.CoMHeightAmplitude(); }))
    );

  gui()->addElement(
      {"Walking", "Footsteps Parameters"},
      mc_rtc::gui::Checkbox(
          "Velocity Control Mode", [this]() { return velocityControl; },
          [this]() { velocityControl = !velocityControl; }),
      mc_rtc::gui::ArrayInput(
          "Reference velocity", {"x", "y", "omega"}, [this]() -> const Eigen::Vector3d & { return reference_velocity; },
          [this](const Eigen::Vector3d & vel) { reference_velocity = vel; }),
      mc_rtc::gui::NumberInput(
          "Ts", [this]() -> double { return ts(); }, [this](const double t) { T_Steps = t; }),
      mc_rtc::gui::NumberInput(
          "Tds/Ts", [this]() -> double { return controller_config_.Double_Step_Ratio; },
          [this](const double t) { controller_config_.Double_Step_Ratio = t; }),
      mc_rtc::gui::NumberInput(
          "Desired Steps", [this]() -> int { return N_Steps_Desired; },
          [this](const double n) { N_Steps_Desired_std = static_cast<int>(n); }),
      mc_rtc::gui::Label("Steps Done", [this]() -> int { return N_Steps; }),
      mc_rtc::gui::Transform(
          "Reference pose", [this]() { return target_pose_; },
          [this](const sva::PTransformd & in) {
            target_pose_ = sva::PTransformd(
                mc_rbdyn::rpyToMat(Eigen::Vector3d{0, 0, mc_rbdyn::rpyFromMat(in.rotation()).z()}), in.translation());
          }),
      mc_rtc::gui::Button("Compute Trajectory", [this]() { compute_trajectory_once.notify_all(); }));

  gui()->addElement({"Walking", "Visualization", "Feasibility"},
                    mc_rtc::gui::Polygon("Feasibility Region", mc_rtc::gui::Color(1., 0., 1.),
                                         [this]() -> std::vector<Eigen::Vector3d> {
                                           std::vector<Eigen::Vector3d> output = mpc_state_.FeasibilityPolygon;
                                           for(auto & p : output)
                                           {
                                             p.z() = mpc_state_.getPck().z();
                                           }
                                           return output;
                                         }),
                    mc_rtc::gui::Polygon("Feasibility Region Switch", mc_rtc::gui::Color(1., 0., 0.5),
                                         [this]() -> std::vector<Eigen::Vector3d> {
                                           std::vector<Eigen::Vector3d> output =
                                               mpc_state_.FeasibilityPolygonStandingSwitch.Get_Polygone_Corners();
                                           for(auto & p : output)
                                           {
                                             p.z() = mpc_state_.getPck().z();
                                           }
                                           return output;
                                         })

  );

  gui()->addElement(
      {"Walking", "Visualization", "Trajectories"},
      //     mc_rtc::gui::Trajectory(
      //         "RealCoMTrajectory",
      //         mc_rtc::gui::LineConfig(mc_rtc::gui::Color(1., 0., 0.), 0.01, mc_rtc::gui::LineStyle::Dotted),
      //         [this]() {
      //           if(UseRealRobot)
      //           {
      //             return realRobot().com();
      //           }
      //           else
      //           {
      //             return robot().com();
      //           }
      //         }),
      // mc_rtc::gui::Trajectory(
      //     "SwingFoot Trajectory",
      //     mc_rtc::gui::LineConfig(mc_rtc::gui::Color(0., 1., 0.), 0.01, mc_rtc::gui::LineStyle::Solid),
      //     [this]() { return robot().surfacePose(swingFootName).translation(); }),
      // mc_rtc::gui::Trajectory(
      //     "ZMPMeasured", mc_rtc::gui::LineConfig(mc_rtc::gui::Color(0.5, 1., 0.), 0.01,
      //     mc_rtc::gui::LineStyle::Solid), [this]() { return stabTask->measuredZMP(); }),
      // mc_rtc::gui::Trajectory(
      //     "p_z_k", mc_rtc::gui::LineConfig(mc_rtc::gui::Color(0., 1., 0.), 0.01, mc_rtc::gui::LineStyle::Solid),
      //     [this]() {
      //       return mpc_state_.p_z_k;
      //       ;
      //     }),

      mc_rtc::gui::Force(
          "Target angular momentum",
          [this]() -> sva::ForceVecd { return sva::ForceVecd(mpc_state_.get_Lc_dot(0), Eigen::Vector3d::Zero()); },
          [this]() -> const sva::PTransformd & { return robot().posW(); }),

      mc_rtc::gui::Trajectory(
          "Predicted ZMP Trajectory",
          mc_rtc::gui::LineConfig(mc_rtc::gui::Color(0., 0., 1.), 0.02, mc_rtc::gui::LineStyle::Dotted),
          [this]() -> std::vector<Eigen::Vector3d> {
            std::vector<Eigen::Vector3d> Output;
            for(size_t k = 0; k < mpc_state_.X_MPC.size(); k++)
            {
              Output.push_back(mpc_state_.Get_ZMP_planarTarget(k));
            }
            return Output;
          }),
      // mc_rtc::gui::Trajectory("ZMP Ref Trajectory",
      // mc_rtc::gui::LineConfig(mc_rtc::gui::Color(0., 1., 1.),0.02,mc_rtc::gui::LineStyle::Dotted),
      //                         [this]() -> std::vector<Eigen::Vector3d> {
      //                           std::vector<Eigen::Vector3d> ref_traj = this->mpc_state_.zmp_ref_traj();
      //                           return ref_traj;
      //                         }),

      mc_rtc::gui::Trajectory(
          "ZMP QP Trajectory",
          mc_rtc::gui::LineConfig(mc_rtc::gui::Color(0., 1., 0.6), 0.02, mc_rtc::gui::LineStyle::Dotted),
          [this]() -> std::vector<Eigen::Vector3d> {
            std::vector<Eigen::Vector3d> ref_traj = mpc_state_.QP_zmp;
            return ref_traj;
          }),

      mc_rtc::gui::Trajectory(
          "Admittance ref Trajectory",
          mc_rtc::gui::LineConfig(mc_rtc::gui::Color(0., 1., 0.6), 0.02, mc_rtc::gui::LineStyle::Dotted),
          [this]() -> const std::vector<Eigen::Vector3d> & { return mpc_state_.admittance_ref_; }),

      mc_rtc::gui::Trajectory(
          "DCM ref Trajectory",
          mc_rtc::gui::LineConfig(mc_rtc::gui::Color(0., 1., 1), 0.02, mc_rtc::gui::LineStyle::Dotted),
          [this]() -> std::vector<Eigen::Vector3d> {
            std::vector<Eigen::Vector3d> output;
            std::vector<Eigen::Vector2d> traj = MPCSolver.dcmRefTrajectory();

            for(auto & p : traj)
            {
              output.push_back(Eigen::Vector3d{p.x(), p.y(), mpc_state_.p_c_k.z()});
            }
            return output;
          }),

      mc_rtc::gui::Trajectory("Predicted CoM Trajectory", mc_rtc::gui::Color(1., 0., 0.),
                              [this]() -> std::vector<Eigen::Vector3d> {
                                std::vector<Eigen::Vector3d> Output;
                                for(size_t k = 0; k < mpc_state_.X_MPC.size(); k++)
                                {
                                  Output.push_back(mpc_state_.Get_CoM_planarTarget(k)
                                                   + Eigen::Vector3d{0, 0, controller_config_.stab_config.comHeight});
                                }
                                return Output;
                              }),

      mc_rtc::gui::Trajectory("Predicted DCM Trajectory", mc_rtc::gui::Color(0., 1., 0.),
                              [this]() -> std::vector<Eigen::Vector3d> {
                                std::vector<Eigen::Vector3d> Output;
                                for(size_t k = 0; k < mpc_state_.X_MPC.size(); k++)
                                {
                                  Output.push_back(mpc_state_.Get_CoM_planarTarget(k)
                                                   + mpc_state_.Get_CoMVel_planarTarget(k) / mpc_state_.eta
                                                   + Eigen::Vector3d{0, 0, controller_config_.stab_config.comHeight});
                                }
                                return Output;
                              }),

      mc_rtc::gui::Trajectory(
          "DCM QP Trajectory",
          mc_rtc::gui::LineConfig(mc_rtc::gui::Color(0., 1., 0.), 0.02, mc_rtc::gui::LineStyle::Dotted),
          [this]() -> std::vector<Eigen::Vector3d> {
            std::vector<Eigen::Vector3d> ref_traj = mpc_state_.QP_dcm;
            return ref_traj;
          }),

      // mc_rtc::gui::Trajectory("Anticipative Trajectory", mc_rtc::gui::Color(1., 0., 1.),
      //                         [this]() -> std::vector<Eigen::Vector3d> {
      //                           std::vector<Eigen::Vector3d> Output;
      //                           Eigen::VectorXd Traj = MPCSolver.GetAfterTc_ZMP_trajectory();
      //                           int n = (int)(Traj.size() / 2);
      //                           for(int k = 0; k < n; k++)
      //                           {
      //                             Output.push_back(Eigen::Vector3d{Traj(k), Traj(k + n), 0});
      //                           }
      //                           return Output;
      //                         }),

      // mc_rtc::gui::Polygon(
      //     "AllPoly", mc_rtc::gui::Color(1., 0.3, 0.),
      //     [this]() -> const std::vector<std::vector<Eigen::Vector3d>> & { return this->MPCSolver.get_allpolys(); }),
      mc_rtc::gui::Polygon("SupportPolygon", mc_rtc::gui::Color(1., 1., 0.),
                           [this]() -> const std::vector<Eigen::Vector3d> & { return mpc_state_.get_SupPolygon(); }));

  gui()->addElement(
      {"Walking", "Plots"}, mc_rtc::gui::ElementsStacking::Horizontal,
      mc_rtc::gui::Button("Plot ZMP Tracking (x)",
                          [this]() {
                            gui()->addPlot(
                                "ZMP Model Tracking (x)",
                                mc_rtc::gui::plot::X(
                                    "t", [this]() { return static_cast<double>(count) * controller_timestep; }),
                                mc_rtc::gui::plot::Y(
                                    "u", [this]() { return admittanceTarget.x(); }, mc_rtc::gui::Color::Red),
                                mc_rtc::gui::plot::Y(
                                    "mpc zmp ref", [this]() { return mpc_state_.ref_zmp_.x(); },
                                    mc_rtc::gui::Color::Green, mc_rtc::gui::plot::Style::Solid),
                                mc_rtc::gui::plot::Y(
                                    "zmp_mes", [this]() { return mpc_state_.getPzk().x(); }, mc_rtc::gui::Color::Blue,
                                    mc_rtc::gui::plot::Style::Dashed),
                                mc_rtc::gui::plot::Y(
                                    "dcm_mes", [this]() { return mpc_state_.getPuk().x(); },
                                    mc_rtc::gui::Color::Magenta, mc_rtc::gui::plot::Style::Solid),
                                mc_rtc::gui::plot::Y(
                                    "zmp modelled", [this]() { return zmpTarget.x(); }, mc_rtc::gui::Color::Blue));
                          }),
      mc_rtc::gui::Button("Stop ZMP (x)", [this]() { gui()->removePlot("ZMP Model Tracking (x)"); }));

  gui()->addElement(
      {"Walking", "Plots"}, mc_rtc::gui::ElementsStacking::Horizontal,
      mc_rtc::gui::Button("Plot ZMP Tracking (y)",
                          [this]() {
                            gui()->addPlot(
                                "ZMP Model Tracking (y)",
                                mc_rtc::gui::plot::X(
                                    "t", [this]() { return static_cast<double>(count) * controller_timestep; }),
                                mc_rtc::gui::plot::Y(
                                    "u", [this]() { return admittanceTarget.y(); }, mc_rtc::gui::Color::Red),
                                mc_rtc::gui::plot::Y(
                                    "mpc zmp ref", [this]() { return mpc_state_.ref_zmp_.y(); },
                                    mc_rtc::gui::Color::Green, mc_rtc::gui::plot::Style::Solid),
                                mc_rtc::gui::plot::Y(
                                    "zmp_mes", [this]() { return mpc_state_.getPzk().y(); }, mc_rtc::gui::Color::Blue,
                                    mc_rtc::gui::plot::Style::Dashed),
                                mc_rtc::gui::plot::Y(
                                    "dcm_mes", [this]() { return mpc_state_.getPuk().y(); },
                                    mc_rtc::gui::Color::Magenta, mc_rtc::gui::plot::Style::Solid),
                                mc_rtc::gui::plot::Y(
                                    "zmp modelled", [this]() { return zmpTarget.y(); }, mc_rtc::gui::Color::Blue));
                          }),
      mc_rtc::gui::Button("Stop ZMP (y)", [this]() { gui()->removePlot("ZMP Model Tracking (y)"); })

  );
  constexpr double ARROW_HEAD_DIAM = 0.015;
  constexpr double ARROW_HEAD_LEN = 0.05;
  constexpr double ARROW_SHAFT_DIAM = 0.06;
  constexpr double FORCE_SCALE = 0.0015;
  mc_rtc::gui::ArrowConfig pendulumArrowConfig;
  pendulumArrowConfig.color = mc_rtc::gui::Color::Yellow;
  pendulumArrowConfig.end_point_scale = 0.02;
  pendulumArrowConfig.head_diam = .1 * ARROW_HEAD_DIAM;
  pendulumArrowConfig.head_len = .1 * ARROW_HEAD_LEN;
  pendulumArrowConfig.scale = 1.;
  pendulumArrowConfig.shaft_diam = .1 * ARROW_SHAFT_DIAM;
  pendulumArrowConfig.start_point_scale = 0.02;
  gui()->addElement(
      {"Walking", "Debug"},
      mc_rtc::gui::Checkbox(
          "Active", [this]() -> bool { return debugMode; },
          [this]() {
            if(!robot_walking())
            {
              debugMode = !debugMode;
              deactivate();
            }
          }),
      mc_rtc::gui::Point3D(
          "CoM", [this]() -> const Eigen::Vector3d & { return debugCoM; },
          [this](const Eigen::Vector3d & p) { debugCoM = p; }),
      mc_rtc::gui::Point3D(
          "ZMP", [this]() -> const Eigen::Vector3d & { return debugZMP; },
          [this](const Eigen::Vector3d & p) { debugZMP = p; }),
      mc_rtc::gui::Arrow(
          "Pendulum", pendulumArrowConfig, [this]() -> Eigen::Vector3d { return debugZMP; },
          [this]() -> Eigen::Vector3d { return debugCoM; }),
      mc_rtc::gui::NumberInput(
          "t_k", [this]() -> const double { return debugTk; }, [this](const double p) { debugTk = p; }),
      mc_rtc::gui::Checkbox(
          "Stop", [this]() -> bool { return debugStop; }, [this]() { debugStop = !debugStop; }),
      mc_rtc::gui::Checkbox(
          "Double Support", [this]() -> bool { return debugDblSupp; }, [this]() { debugDblSupp = !debugDblSupp; }));
}

void Walking_controller::add_ISMPC_Config_GUI()
{
  gui()
      ->addElement(
          {"Walking", "ISMPC Configuration"},
          mc_rtc::gui::Form(
              "Configure", [this](const mc_rtc::Configuration & conf) { reconfigure(conf); },
              mc_rtc::gui::FormArrayInput("QP Weight (zmp_vel ; step ; zmp traj ; stab ; Ld ; dcm ; dcm_vel)", false,
                                          [this]() -> std::array<double, 7> {
                                            return {controller_config_.Beta_zmp_vel,  controller_config_.Beta_step,
                                                    controller_config_.Beta_zmp_traj, controller_config_.Beta_stab,
                                                    controller_config_.Beta_Ld,       controller_config_.Beta_dcm,
                                                    controller_config_.Beta_dcm_vel};
                                          }),
              mc_rtc::gui::FormArrayInput(
                  "Beta dcm static (pos , vel)", false,
                  [this]() -> std::array<double, 2> {
                    return {controller_config_.Beta_dcm_static, controller_config_.Beta_dcm_vel_static};
                  }),
              mc_rtc::gui::FormNumberInput("Beta zmp static (pos)", false,
                                           [this]() { return controller_config_.Beta_zmp_traj_static; }),
              mc_rtc::gui::FormNumberInput("Tc", false, [this]() { return controller_config_.Tc; }),
              mc_rtc::gui::FormNumberInput("delta", false, [this]() { return controller_config_.delta; }),
              mc_rtc::gui::FormArrayInput("step kinematics cstr", false,
                                          [this]() -> std::array<double, 2> {
                                            return {controller_config_.MPC_Footsteps_kin_Constraint_size.x(),
                                                    controller_config_.MPC_Footsteps_kin_Constraint_size.y()};
                                          }),
              mc_rtc::gui::FormArrayInput("zmp cstr square static", false,
                                          [this]() -> std::array<double, 2> {
                                            return {controller_config_.MPC_ZMP_cstr_square_static.x(),
                                                    controller_config_.MPC_ZMP_cstr_square_static.y()};
                                          }),
              mc_rtc::gui::FormArrayInput("zmp cstr square", false,
                                          [this]() -> std::array<double, 2> {
                                            return {controller_config_.MPC_ZMP_Constraint_size.x(),
                                                    controller_config_.MPC_ZMP_Constraint_size.y()};
                                          }),
              mc_rtc::gui::FormArrayInput(
                  "u cstr square", false,
                  [this]() -> std::array<double, 2> {
                    return {controller_config_.MPC_U_Constraint_size.x(), controller_config_.MPC_U_Constraint_size.y()};
                  }),
              mc_rtc::gui::FormArrayInput("zmp cstr square offset", false,
                                          [this]() -> std::array<double, 2> {
                                            return {controller_config_.MPC_ZMP_cstr_square_offset.x(),
                                                    controller_config_.MPC_ZMP_cstr_square_offset.y()};
                                          }),
              mc_rtc::gui::FormArrayInput("zmp ref offset", false,
                                          [this]() -> std::array<double, 2> {
                                            return {controller_config_.MPC_ZMP_ref_offset_sg_supp.x(),
                                                    controller_config_.MPC_ZMP_ref_offset_sg_supp.y()};
                                          }),
              mc_rtc::gui::FormArrayInput("zmp ref end step", false,
                                          [this]() -> std::array<double, 2> {
                                            return {controller_config_.MPC_ZMP_ref_offset_end_step.x(),
                                                    controller_config_.MPC_ZMP_ref_offset_end_step.y()};
                                          }),
              mc_rtc::gui::FormArrayInput("zmp ref start step", false,
                                          [this]() -> std::array<double, 2> {
                                            return {controller_config_.MPC_ZMP_ref_offset_start_step.x(),
                                                    controller_config_.MPC_ZMP_ref_offset_start_step.y()};
                                          }),
              mc_rtc::gui::FormNumberInput("feet distance", false,
                                           [this]() { return controller_config_.feet_ditance_; }),
              mc_rtc::gui::FormNumberInput("External disturbance cutoff period", false,
                                           [this]() { return controller_config_.external_disturbance_cutoff_period_; }),
              mc_rtc::gui::FormCheckbox("Include high frequency hand disturbance", false, [this]() {
                return controller_config_.with_external_disturbance_highpass_filter_;
              })));
}

void Walking_controller::add_FootSteps_GUI()
{

  auto footStepPolygon = [this](const sva::PTransformd & X_0_step, const sva::PTransformd & X_0_support,
                                std::string sup) {
    // double x = step.translation().x() ;
    // double y = step.translation().y() ;
    // double theta = mc_rbdyn::rpyFromMat(step.rotation()).z();
    // Eigen::Vector3d footPos;
    // footPos << x, y, 0;
    // Eigen::Matrix3d footOri = mc_rbdyn::rpyToMat(0, 0, theta);
    // sva::PTransformd X_support_foot(footOri, footPos);
    const auto & surface = robot().surface(sup);
    const auto & points = surface.points();
    std::vector<Eigen::Vector3d> polygon;
    for(const auto & point : points)
    {
      const auto & X_foot_point = surface.X_b_s().inv() * point;
      Eigen::Vector3d p = (X_foot_point * X_0_step).translation();
      polygon.push_back(p);
    }
    return polygon;
  };

  gui()->addElement(
      {"Walking", "Visualization", "FootStep"},
      mc_rtc::gui::Polygon("Planned Footsteps", mc_rtc::gui::Color(0., 1., 0.),
                           [this, footStepPolygon]() -> std::vector<std::vector<Eigen::Vector3d>> {
                             const std::vector<sva::PTransformd> & steps = mpc_state_.planned_steps();
                             //  mc_rtc::log::info("step {}",steps.size());
                             std::vector<std::vector<Eigen::Vector3d>> Output;
                             for(size_t k = 0; k < steps.size(); k++)
                             {
                               if(k % 2 == 1)
                               {
                                 Output.push_back(footStepPolygon(steps[k], this->X_0_support, this->supportFootName));
                               }
                               else
                               {
                                 Output.push_back(footStepPolygon(steps[k], this->X_0_support, this->swingFootName));
                               }
                             }
                             return Output;
                           }));
  gui()->addElement({"Walking", "Visualization", "FootStep"},
                    mc_rtc::gui::Polygon(
                        "Corrected Footsteps", mc_rtc::gui::Color(0., 0., 1.),
                        [this, footStepPolygon]() -> std::vector<std::vector<Eigen::Vector3d>> {
                          const std::vector<sva::PTransformd> & steps_opti = mpc_state_.optimal_steps();

                          std::vector<std::vector<Eigen::Vector3d>> Output;
                          //  mc_rtc::log::info("step opti {}",steps_opti.size());

                          for(size_t k = 0; k < steps_opti.size(); k++)
                          {

                            if(k % 2 == 1)
                            {

                              Output.push_back(
                                  footStepPolygon(steps_opti[k], this->X_0_support, this->supportFootName));
                            }
                            else
                            {
                              Output.push_back(footStepPolygon(steps_opti[k], this->X_0_support, this->swingFootName));
                            }
                          }
                          return Output;
                        }));
}

void Walking_controller::Stabilizer_GUI(mc_rbdyn::lipm_stabilizer::StabilizerConfiguration & config, std::string name)
{
  if(!gui()->hasElement({"Walking", "Stabilizer"}, "Activate"))
  {
    gui()->addElement({"Walking", "Stabilizer"},
                      mc_rtc::gui::Button("Activate",
                                          [this]() {
                                            stabTask->enable();
                                            stabilizer_active_ = true;
                                          }),
                      mc_rtc::gui::Button("Deactivate", [this]() {
                        if(!robot_walking())
                        {
                          stabTask->disable();
                          stabilizer_active_ = false;
                        }
                      }));
  }
  AddStabilizerConfigToGUI(*gui(), {"Walking", "Stabilizer", name}, config);
}
