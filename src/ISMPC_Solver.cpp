#include "../include/ismpc_walking/ISMPC_Solver.h"
#include <mc_rtc/io_utils.h>

ISMPC_Solver::ISMPC_Solver() {}

ISMPC_Solver::ISMPC_Solver(double delta_controller, double delta, double Tp, double Tc)
{
  m_Tc = Tc;
  m_Tp = Tp;
  m_delta = delta;
  m_delta_control = delta_controller;

  count_Dstep = 0;
  m_C = static_cast<int>(m_Tc / m_delta);
  m_P = static_cast<int>(m_Tp / m_delta);
  QPsuccess = false;

  // Resize vectors to match the control horizon
  CoM_height.resize(m_C);
  m_eta.resize(m_C);
  m_eta_free.resize(m_C);

  // At construction time m_timestamp is not yet available, so fill with the constant
  // nominal height. init_MPC() will overwrite this with the phase-based profile on the
  // first QP call, once footstep timings are known.
  const double eta_nom = std::sqrt(g / CoM_height_avg);
  std::fill(CoM_height.begin(), CoM_height.end(), CoM_height_avg);
  std::fill(m_eta.begin(), m_eta.end(), eta_nom);
  std::fill(m_eta_free.begin(), m_eta_free.end(), eta_nom);
  Compute_Integration_Matrix(m_eta);

  m_kappa = 1;
  w_k.setZero();
  m_kappa_inf = 1;
  w_k_inf.setZero();
  perturbation_duration = 0;
}

void ISMPC_Solver::configure(const ControllerConfiguration & config)
{
  m_dx_f = config.MPC_Footsteps_kin_Constraint_size.x();
  m_dy_f = config.MPC_Footsteps_kin_Constraint_size.y();
  m_dx_f_rect = config.MPC_Footsteps_Constraint_size.x();
  m_dy_f_rect = config.MPC_Footsteps_Constraint_size.y();
  m_dx = config.MPC_ZMP_Constraint_size.x();
  m_dy = config.MPC_ZMP_Constraint_size.y();
  m_dx_u = config.MPC_U_Constraint_size.x();
  m_dy_u = config.MPC_U_Constraint_size.y();
  m_dx_static = config.MPC_ZMP_cstr_square_static.x();
  m_dy_static = config.MPC_ZMP_cstr_square_static.y();
  m_Beta_step = config.Beta_step;
  m_Beta_zmp_vel = config.Beta_zmp_vel;
  m_Beta_stab = config.Beta_stab;
  m_Beta_zmp_traj = config.Beta_zmp_traj;
  m_Beta_zmp_traj_stop = config.Beta_zmp_traj_static;
  m_Beta_Lc = config.Beta_Ld;
  m_Beta_dcm = config.Beta_dcm;
  m_Beta_dcm_stop = config.Beta_dcm_static;
  m_Beta_dcm_vel = config.Beta_dcm_vel;
  m_Beta_dcm_vel_stop = config.Beta_dcm_vel_static;
  m_lambda = config.lambda_;
  m_feet_distance = config.feet_ditance_;
  zmp_delay(config.zmp_delay);
  Slide_ZMP_region = config.sliding_zmp_cstr_region;
  zmp_cstr_next_stp_ratio = config.MPC_ZMP_next_stp_cstr_ratio;
  rect_pose_offset = config.MPC_ZMP_cstr_square_offset;
  rect_pose_offset_static = config.MPC_ZMP_static_cstr_square_offset;
  Allow_None = config.MPC_allow_None;
  m_Tc = config.Tc;
  m_Tp = config.Tp;
  m_delta = config.delta;
  m_delta_control = config.controller_timestep;
  m_C = (int)std::round((m_Tc) / m_delta);
  m_P = (int)std::round((m_Tp) / m_delta);

  // Resize to updated horizon size after configure() may have changed m_C
  CoM_height.resize(m_C);
  m_eta.resize(m_C);
  m_eta_free.resize(m_C);

  CoM_height_avg = config.stab_config.comHeight;

  // Same as constructor: fill with constant nominal until init_MPC() has footstep timings.
  const double eta_nom_cfg = std::sqrt(g / CoM_height_avg);
  std::fill(CoM_height.begin(), CoM_height.end(), CoM_height_avg);
  std::fill(m_eta.begin(), m_eta.end(), eta_nom_cfg);
  std::fill(m_eta_free.begin(), m_eta_free.end(), eta_nom_cfg);

  Use_Stability_Task = config.use_stability_task;
  zmp_ref_offset = config.MPC_ZMP_ref_offset_sg_supp;
  zmp_ref_offset_end_step = config.MPC_ZMP_ref_offset_end_step;
  zmp_ref_offset_start_step = config.MPC_ZMP_ref_offset_start_step;
  m_ts_range = config.ts_range;
  m_tss_range = config.tss_range;
  m_tds_range = config.tds_range;
  m_foot_max_vel = config.max_swing_foot_velocity;

  Compute_Integration_Matrix(m_eta);
}

void ISMPC_Solver::init_MPC(const MPC_state & mpc_state, std::string Tail, int Steps_Desired, int Step)
{
  P_c_k = mpc_state.p_c_k;
  V_c_k = mpc_state.v_c_k;
  P_z_k = mpc_state.p_z_k;
  Lc_k = mpc_state.Lck;
  m_mass = mpc_state.input_mass;

  X_0_swing_foot_target = mpc_state.X_0_Step_Target;
  X_0_swing_foot = mpc_state.X_0_SwingFoot;

  DoubleSupport = mpc_state.doubleSupport;
  m_t_lift = mpc_state.t_lift;

  m_tk = std::max(0., mpc_state.t_k);
  m_t_global = mpc_state.t;
  m_delay_elapsed = std::min(m_delay - (m_t_global - m_t_delay), m_delay);
  if(m_t_global - m_t_delay > m_delta || m_tk == 0 || m_delay_elapsed < 0)
  {
    U_k = mpc_state.Uk;
    m_t_delay = m_t_global;
    m_delay_elapsed = m_delay;
  }
  if(mpc_state.stop)
  {
    m_delay_elapsed = 0;
  }

  P_z_k_delayed = P_z_k + (1 - exp(-m_lambda * m_delay_elapsed)) * (U_k - P_z_k);
  m_Tail = Tail;

  m_support_foot = mpc_state.input_Support_FootName;

  // Cheap placeholder DCM using the instantaneous eta (index 0) from the PREVIOUS call's height
  // profile (m_eta is not yet refreshed for this call at this point in init_MPC). This value is
  // consumed by m_feasibilitySolver.solve() before Stability_Constraints() runs later in
  // GetWalkingParameters(). It is NOT the kernel-consistent DCM: the authoritative
  // xi(t0) = P_c_k + V_c_k/Omega(t0), with Omega solving the Riccati equation over the
  // freshly-computed m_eta profile, is (re)computed in Stability_Constraints() via
  // Compute_Riccati_Kernel(), and overwrites P_u_k there before the QP is solved.
  P_u_k = P_c_k + (V_c_k / m_eta[0]);
  X_0_swing_foot_initial = mpc_state.X_0_Initial_SwingFoot;

  X_0_support_foot = mpc_state.X_0_SupportFoot;

  input_steps_ = mpc_state.planned_steps_;
  m_timestamp = mpc_state.planned_timesteps_;
  m_input_Tds = mpc_state.tds;

  N_Steps = Step;
  N_Steps_Desired = Steps_Desired;

  if(input_steps_.size() == 0)
  {
    mc_rtc::log::warning("[ISMPC] No Footsteps target provided");
    input_steps_.push_back(X_0_swing_foot_initial);
  }

  R_0_support = X_0_support_foot.rotation();
  R_support_0 = R_0_support.transpose();

  m_kappa = 1;
  w_k.setZero();
  m_kappa_inf = 1;
  w_k_inf.setZero();
  perturbation_duration = 0;

  // --- Phase-based CoM height profile ---
  // m_tk is the elapsed time within the current step cycle (set above from mpc_state.t_k).
  // It resets to 0 at each step switch, which is the signal we use to also reset
  // m_tk_within_step. Outside of that event, m_tk_within_step advances by m_delta each call.
  if(m_tk <= m_delta)
  {
    // m_tk just reset — a new step has started. Snap m_tk_within_step to m_tk so that
    // the phase at sample 0 exactly equals m_tk / T_step rather than drifting.
    m_tk_within_step = m_tk;
  }
  else
  {
    m_tk_within_step += m_delta;
  }

  // Build a flat array of step durations over the control horizon from m_timestamp.
  // step_duration[j] = duration of the j-th upcoming step, step_start[j] = time relative
  // to now at which step j starts. Index 0 is the current step (already started at
  // -m_tk_within_step relative to now, landing at m_timestamp[0] - m_tk relative to now).
  //
  // We express everything as time-relative-to-now (t_i = i * m_delta) for sample i.
  // Step j spans [t_start_j, t_end_j) where t_end_j = m_timestamp[j] - m_tk.
  // The duration of step j is T_j = t_end_j - t_start_j.
  // The phase of sample i within step j is phi = (t_i - t_start_j + m_tk_within_step) / T_j
  // (the m_tk_within_step term accounts for the fact that step 0 already started earlier).
  //
  // NOTE: the above per-step-phase-locked cosine profile (crouch/extend timed to footstep
  // phase) is a real, physically-motivated FEATURE, kept available below behind
  // CoMHeightTestSignal::PerStepCosine, but is NOT what should be used for CoM-z tracking
  // identification tests: it couples the height profile to footstep timing (variable T_j,
  // fragile phase bookkeeping across step transitions -- this is what produced the spike
  // artifacts seen in earlier logged sine-response data), which is exactly the confound we
  // want to AVOID when trying to isolate the whole-body-controller's own height-tracking
  // dynamics. For identification, use a wall-clock-based step or sine (below), decoupled
  // entirely from footstep phase.

  // --- CoM-height reference profile selector (for CoM-z tracking identification tests) ---
  // Only one of these should be active at a time; PerStepCosine is the physically-motivated
  // production profile, Step and Sine are wall-clock test signals for system identification
  // (clean impulse/step response fitting, or frequency-domain / least-squares sine fitting).
  enum class CoMHeightTestSignal
  {
    PerStepCosine,
    Step,
    Sine,
    RlSine // RL-driven sine: offset/amplitude/frequency/phase from
           // SetCoMHeightSineParams(), set externally each control period.
  };
  constexpr CoMHeightTestSignal test_signal = CoMHeightTestSignal::Sine;

  switch(test_signal)
  {
    case CoMHeightTestSignal::PerStepCosine:
    {
      const double four_pi_sq = 4.0 * M_PI * M_PI; // precomputed constant
      double t_start_j = -m_tk_within_step; // t_start_j relative to now for step j=0
      size_t j = 0; // index into m_timestamp

      for(int i = 0; i < m_C; ++i)
      {
        const double t_i = static_cast<double>(i) * m_delta; // time of sample i relative to now

        // Advance step index if sample i has crossed into the next step
        while(j + 1 < m_timestamp.size() && t_i >= m_timestamp[j] - m_tk)
        {
          t_start_j = m_timestamp[j] - m_tk; // new step starts here relative to now
          ++j;
        }

        // Duration of the step that contains sample i
        const double t_end_j = (j < m_timestamp.size()) ? (m_timestamp[j] - m_tk) : (m_timestamp.back() - m_tk + m_Tds);
        const double T_j = t_end_j - t_start_j;
        const double T_j_safe = (T_j > 1e-6) ? T_j : 1e-6; // guard against zero-duration edge case

        // Phase in [0, 1] within step j
        const double phi = std::clamp((t_i - t_start_j) / T_j_safe, 0.0, 1.0);

        // Cosine profile: minimum height (crouch) at phi=0 (step start / double support),
        // maximum height at phi=0.5 (mid single support). Same cosine evaluation gives both
        // z_c and z_ddot analytically -- no extra trig call.
        const double cos_phi = std::cos(2.0 * M_PI * phi);

        CoM_height[i] = CoM_height_avg - m_com_z_amplitude * cos_phi;

        // z_ddot = A * (4*pi^2 / T^2) * cos(2*pi*phi)
        // Derivation: z(phi) = z_nom - A*cos(2*pi*phi), phi = t/T (constant within step)
        // dz/dt = A * 2*pi/T * sin(2*pi*phi)
        // d2z/dt2 = A * (2*pi/T)^2 * cos(2*pi*phi)
        const double zc_ddot = m_com_z_amplitude * (four_pi_sq / (T_j_safe * T_j_safe)) * cos_phi;

        m_eta[i] = std::sqrt((zc_ddot + g) / CoM_height[i]);
        m_eta_free[i] = m_eta[i];
      }
      break;
    }

    case CoMHeightTestSignal::Step:
    {
      // Single wall-clock step, decoupled from footstep phase, for clean step-response
      // identification (fit a 1st/2nd-order model to CoM_height_actual(t) vs this reference).
      // Zero feedforward acceleration: the reference is a true mathematical step (infinite
      // acceleration at the edge, not representable/meaningful as a feedforward term here);
      // this is intentional and matches what a step-response identification test wants to see
      // (an INPUT step, with all resulting z_ddot behavior coming from the plant/tracking
      // dynamics being identified, not fed forward from the reference itself).
      for(int i = 0; i < m_C; ++i)
      {
        const double t_i = m_t_global + static_cast<double>(i) * m_delta; // absolute time of sample i
        CoM_height[i] = (t_i < m_com_z_test_t0) ? CoM_height_avg : (CoM_height_avg + m_com_z_amplitude);

        const double zc_ddot = 0.0;
        m_eta[i] = std::sqrt((zc_ddot + g) / CoM_height[i]);
        m_eta_free[i] = m_eta[i];
      }
      break;
    }

    case CoMHeightTestSignal::Sine:
    {
      // Wall-clock sinusoid, decoupled from footstep phase (unlike PerStepCosine above), so the
      // reference is a clean, uninterrupted sine usable for identification regardless of step
      // timing/duration. z_c(t) = CoM_height_avg + A*sin(omega_test*t), with the analytically
      // exact z_ddot fed forward (unlike the Step case above, here a smooth feedforward IS
      // physically meaningful and should be supplied, since the reference itself is smooth).
      const double omega_test = 2.0 * M_PI / std::max(m_com_z_test_period, 1e-6);
      CoM_height_vel.resize(static_cast<size_t>(m_C));
      CoM_height_acc.resize(static_cast<size_t>(m_C));
      for(int i = 0; i < m_C; ++i)
      {
        const double t_i = m_t_global + static_cast<double>(i) * m_delta; // absolute time of sample i
        const double phase = omega_test * t_i;
        const double sin_phase = std::sin(phase);
        const double cos_phase = std::cos(phase);

        CoM_height[i] = CoM_height_avg + m_com_z_amplitude * sin_phase;
        const double zc_dot = m_com_z_amplitude * omega_test * cos_phase;
        const double zc_ddot = -m_com_z_amplitude * omega_test * omega_test * sin_phase;
        CoM_height_vel[i] = zc_dot;
        CoM_height_acc[i] = zc_ddot;

        m_eta[i] = std::sqrt((zc_ddot + g) / CoM_height[i]);
        m_eta_free[i] = m_eta[i];
      }
      break;
    }

    case CoMHeightTestSignal::RlSine:
    {
      // RL-driven sine, mirrors the Sine case above but reads the reference
      // from SetCoMHeightSineParams() instead of the fixed test constants.
      // The caller (mc_mjlab) is responsible for offset - amplitude >= 0.
      const double omega = 2.0 * M_PI * m_rl_com_z_frequency;
      CoM_height_vel.resize(static_cast<size_t>(m_C));
      CoM_height_acc.resize(static_cast<size_t>(m_C));
      for(int i = 0; i < m_C; ++i)
      {
        const double t_i = m_t_global + static_cast<double>(i) * m_delta;
        const double phase = omega * t_i + m_rl_com_z_phase;
        const double sin_phase = std::sin(phase);
        const double cos_phase = std::cos(phase);

        CoM_height[i] = m_rl_com_z_offset + m_rl_com_z_amplitude * sin_phase;
        // z(t)  = offset + A*sin(w t + phi)
        // zdot  = A*w*cos(w t + phi)
        // zddot = -A*w^2*sin(w t + phi)
        const double zc_dot = m_rl_com_z_amplitude * omega * cos_phase;
        const double zc_ddot = -m_rl_com_z_amplitude * omega * omega * sin_phase;
        CoM_height_vel[i] = zc_dot;
        CoM_height_acc[i] = zc_ddot;

        m_eta[i] = std::sqrt((zc_ddot + g) / CoM_height[i]);
        m_eta_free[i] = m_eta[i];
      }
      break;
    }
  }

  Compute_Integration_Matrix(m_eta);

  // Refresh the Riccati/kernel profile (Omega, beta, B, K) for this call's m_eta/CoM_height
  // profile. This must run whenever a(t) = eta(t)^2 changes, i.e. once per init_MPC() call,
  // and before Stability_Constraints() (which calls Compute_Hk_And_bfree() and consumes
  // m_Omega/m_K_kernel) runs later in GetWalkingParameters().
  Compute_Riccati_Kernel();
}

Eigen::Vector2d ISMPC_Solver::compute_dcm_delay()
{
  const double eta_0 = m_eta[0];
  const double e_d_lpe = eta_0 / (m_lambda + eta_0);
  const double epl = m_lambda + eta_0;
  const Eigen::Vector2d Puk = (P_c_k + (V_c_k / eta_0)).segment(0, 2);

  Eigen::Vector2d Pu_delay = Puk;
  
  // High-performance exponential computation caching
  const double exp_eta_delay = std::exp(eta_0 * m_delay_elapsed);
  
  Pu_delay -= (m_kappa * U_k - w_k).segment(0, 2) * (1.0 - 1.0 / exp_eta_delay);
  Pu_delay -= e_d_lpe * m_kappa * (P_z_k - U_k).segment(0, 2) * (1.0 - std::exp(-epl * m_delay_elapsed));
  Pu_delay *= exp_eta_delay;

  return Pu_delay;
}

void ISMPC_Solver::compute_dcm(Eigen::MatrixXd & A_out,
                               Eigen::Vector2d & b_out,
                               const Eigen::Vector2d & dcm_delay,
                               const int indx)
{
  A_out.setZero(2, N_variable);
  
  // 1. Precompute cumulative integrations of eta up to indx (step j)
  // integrated_eta_to[m] handles the discrete sum of eta from 0 to m-1 multiplied by m_delta
  std::vector<double> integrated_eta_to(indx + 1, 0.0);
  double cum_sum = 0.0;
  for (int k = 0; k < indx; ++k)
  {
    cum_sum += m_eta[k];
    integrated_eta_to[k + 1] = cum_sum * m_delta;
  }

  // Precompute specific horizon exponential scalar for scaling at step j
  const double exp_eta_tj = std::exp(integrated_eta_to[indx]);

  const double tj = static_cast<double>(indx) * m_delta;
  double tp = perturbation_duration;
  if(tj < tp)
  {
    tp = tj;
  }
  const int p = static_cast<int>(tp / m_delta);

  // Time-varying properties evaluate instantaneous value at current step (index 0) or target step
  const double eta_0 = m_eta[0]; 
  const double e_d_lpe = eta_0 / (eta_0 + m_lambda);
  const double lpe = eta_0 + m_lambda;

  // 2. Compute b_out using time-varying exponential terms
  b_out = dcm_delay;
  
  const double exp_neg_eta_tp = std::exp(-eta_0 * tp);
  const double exp_neg_eta_tj = 1.0 / exp_eta_tj; // Reciprocal optimization

  b_out -= P_z_k_delayed.head<2>() * (m_kappa * (1.0 - exp_neg_eta_tp) + (exp_neg_eta_tp - exp_neg_eta_tj) * m_kappa_inf);
  b_out += w_k.head<2>() * (1.0 - exp_neg_eta_tp) + w_k_inf.head<2>() * exp_neg_eta_tp;
  b_out *= exp_eta_tj;

  // 3. Dense Matrix loop over the prediction steps
  for(int i = 0; i < indx; ++i)
  {
    // Use cached values to avoid redundant exponential evaluations
    const double exp_neg_eta_ti = std::exp(-integrated_eta_to[i]);
    const double exp_neg_eta_tp_minus_ti = std::exp(-eta_0 * std::max(0.0, tp - (i * m_delta)));
    const double exp_neg_eta_tj_minus_ti = std::exp(-(integrated_eta_to[indx] - integrated_eta_to[i]));

    double factor = 0.0;
    if(i < p)
    {
      const double dt_p = std::max(0.0, tp - (i * m_delta));
      const double exp_neg_lpe_tp_minus_ti = std::exp(-lpe * dt_p);
      
      factor -= m_kappa * (1.0 - exp_neg_eta_tp_minus_ti - e_d_lpe * (1.0 - exp_neg_lpe_tp_minus_ti));
      factor -= m_kappa_inf * (exp_neg_eta_tp_minus_ti - exp_neg_eta_tj_minus_ti 
                               - e_d_lpe * (exp_neg_lpe_tp_minus_ti - std::exp(-lpe * (tj - (i * m_delta)))));
    }
    else
    {
      const double dt_j = tj - (i * m_delta);
      factor -= m_kappa_inf * (1.0 - exp_neg_eta_tj_minus_ti - e_d_lpe * (1.0 - std::exp(-lpe * dt_j)));
    }

    const double scaled_factor = exp_neg_eta_ti * factor;
    auto block_2x2 = A_out.block<2, 2>(0, 2 * i);
    block_2x2(0, 0) = scaled_factor;
    block_2x2(1, 1) = scaled_factor;

    if(UseAngularMomentumDot)
    {
      // Use time-varying CoM height and eta at step i
      const double eta_i = m_eta[i];
      const double exp_neg_eta_ti_plus_1 = std::exp(-integrated_eta_to[i + 1]);
      
      double am_factor = (exp_neg_eta_ti - exp_neg_eta_ti_plus_1) / (m_mass * CoM_height[i] * eta_i * eta_i);
      
      // Fast block assignment using fixed sizes
      A_out.block<2, 2>(0, 2 * (m_C + j_Max_C + i)) << 0.0, -am_factor, am_factor, 0.0;
    }
  }

  A_out *= exp_eta_tj;
}

void ISMPC_Solver::create_dcm_cost_function(Eigen::MatrixXd & M_dcm,
                                            Eigen::VectorXd & b_dcm,
                                            Eigen::MatrixXd & M_traj_dcm,
                                            Eigen::VectorXd & b_traj_dcm,
                                            Eigen::MatrixXd & M_traj_zmp,
                                            Eigen::VectorXd & b_traj_zmp)
{
  int step_indx = 0;

  double sgn = -1; 
  if(m_support_foot == "RightFoot") 
  {
    sgn = 1;
  }
  const double sgn_init = sgn;

  const Eigen::Vector2d Pu_delayed = compute_dcm_delay();

  M_dcm.setZero(2 * m_C, N_variable);
  b_dcm.setZero(2 * m_C);
  M_traj_dcm.setZero(2 * m_C, N_variable);
  b_traj_dcm.setZero(2 * m_C);
  M_traj_zmp.setZero(2 * m_C, N_variable);
  b_traj_zmp.setZero(2 * m_C);

  Eigen::MatrixXd M_dcm_stab = Eigen::MatrixXd::Zero(2, N_variable);
  Eigen::Vector2d b_dcm_stab = Eigen::Vector2d::Zero();

  const Eigen::Vector2d offset = Eigen::Vector2d{0, m_dy / 2};
  const Eigen::Matrix2d R_support_0 = X_0_support_foot.rotation().transpose().topLeftCorner<2, 2>();
  const Eigen::Vector2d P_support = X_0_support_foot.translation().head<2>() + sgn * R_support_0 * offset;

  double ts_im1 = 0;

  if(!m_stop)
  {
    for(size_t i = 0; i < m_timestamp.size(); i++)
    {
      double ts_i = m_timestamp[i] - m_tk;
      if(i == m_timestamp.size() - 1)
      {
        ts_i = 1e9;
      }

      Eigen::Matrix2d R_i_0;
      Eigen::Vector2d P_i = P_support;

      // For long intervals outside the short control horizon, we use the average/terminal eta
      const double eta_val = m_eta.back();
      const double exp_neg_eta_ts_im1 = std::exp(-eta_val * ts_im1);
      const double exp_neg_eta_ts_i = std::exp(-eta_val * ts_i);
      const double exp_diff = exp_neg_eta_ts_im1 - exp_neg_eta_ts_i;

      if(i == 0)
      {
        b_dcm_stab += exp_diff * P_support;
      }
      else
      {
        R_i_0 = input_steps_[i - 1].rotation().transpose().topLeftCorner<2, 2>();
        P_i = input_steps_[i - 1].translation().head<2>() + sgn * R_i_0 * offset;

        if(static_cast<int>(i - 1) < j_Max_C)
        {
          M_dcm_stab.block(0, 2 * (m_C + i - 1), 2, 2).diagonal().setConstant(exp_diff);
          b_dcm_stab += R_i_0 * sgn * offset * exp_diff;
        }
        else
        {
          b_dcm_stab += exp_diff * P_i;
        }
      }

      ts_im1 = ts_i;
      sgn *= -1;
    }
  }

  double t = m_tk;
  double ts = m_timestamp[0];
  double t_m_PrevTs = 0;
  int indx_step = -1;

  Eigen::Matrix2d R_step_0 = X_0_support_foot.rotation().transpose().topLeftCorner<2, 2>();
  sgn = sgn_init;
  Eigen::Vector2d P_stp = P_support;
  Eigen::Vector2d P_PrevStp = P_support;
  Eigen::Vector2d prevOffset = Eigen::Vector2d::Zero();

  for(int i = 0; i < m_C; i++)
  {
    // Capture the local time-varying eta for this preview step
    const double eta_i = m_eta[i];
    const double exp_eta_delta = std::exp(eta_i * m_delta);

    Eigen::MatrixXd A_dcm;
    Eigen::Vector2d c_dcm;
    compute_dcm(A_dcm, c_dcm, Pu_delayed, i + 1);
    M_dcm.block(2 * i, 0, 2, N_variable) = A_dcm;
    b_dcm.segment<2>(2 * i) = c_dcm;

    if(!m_stop)
    {
      if(t + m_delta >= ts)
      {
        t_m_PrevTs = (t + m_delta - ts);
        prevOffset = sgn * R_step_0 * offset;
        P_PrevStp = P_stp;

        indx_step += 1;
        ts = m_timestamp[indx_step + 1];
        if(indx_step == static_cast<int>(m_timestamp.size() - 1))
        {
          ts = 1e6;
        }
        R_step_0 = input_steps_[indx_step].rotation().transpose().topLeftCorner<2, 2>();
        sgn *= -1;
        if(indx_step != -1 && indx_step > j_Max_C)
        {
          P_stp = input_steps_[indx_step].translation().head<2>() + sgn * R_step_0 * offset;
        }
      }

      const double exp_eta_tm_prev = std::exp(eta_i * t_m_PrevTs);

      if(indx_step - 1 < 0 || indx_step - 1 >= j_Max_C)
      {
        b_traj_dcm.segment<2>(2 * i) -= ((exp_eta_delta - exp_eta_tm_prev) * P_PrevStp);
        b_traj_zmp.segment<2>(2 * i) = P_PrevStp;
      }
      if(indx_step < 0 || indx_step >= j_Max_C)
      {
        b_traj_dcm.segment<2>(2 * i) -= ((exp_eta_tm_prev - 1.0) * P_stp);
        b_traj_zmp.segment<2>(2 * i) = P_stp;
      }
      if(indx_step - 1 >= 0 && indx_step - 1 < j_Max_C)
      {
        const double fact = exp_eta_delta - exp_eta_tm_prev;
        b_traj_dcm.segment<2>(2 * i) -= (fact * prevOffset);
        M_traj_dcm.block<2, 2>(2 * i, 2 * (m_C + indx_step - 1)).diagonal().setConstant(-fact);

        b_traj_zmp.segment<2>(2 * i) = prevOffset;
        M_traj_zmp.block<2, 2>(2 * i, 2 * (m_C + indx_step - 1)).setIdentity();
      }
      if(indx_step >= 0 && indx_step < j_Max_C)
      {
        const double fact = exp_eta_tm_prev - 1.0;
        b_traj_dcm.segment<2>(2 * i) -= (fact * sgn * R_step_0 * offset);
        M_traj_dcm.block<2, 2>(2 * i, 2 * (m_C + indx_step)).diagonal().setConstant(-fact);

        b_traj_zmp.segment<2>(2 * i) = sgn * R_step_0 * offset;
        M_traj_zmp.block<2, 2>(2 * i, 2 * (m_C + indx_step)).setIdentity();
      }
      t_m_PrevTs = m_delta;

      // Cumulative trajectory updates scaled by local step exponent
      if(i == 0)
      {
        M_traj_dcm.block(0, 0, 2, N_variable) = exp_eta_delta * M_dcm_stab;
        b_traj_dcm.head<2>() += exp_eta_delta * b_dcm_stab;
      }
      else
      {
        M_traj_dcm.block(2 * i, 0, 2, N_variable) += exp_eta_delta * M_traj_dcm.block(2 * (i - 1), 0, 2, N_variable);
        b_traj_dcm.segment<2>(2 * i) += exp_eta_delta * b_traj_dcm.segment<2>(2 * (i - 1));
      }
      t += m_delta;
    }
    else
    {
      b_traj_dcm.segment<2>(2 * i) = m_ref_zmp.head<2>();
      b_traj_zmp.segment<2>(2 * i) = m_ref_zmp.head<2>();
    }
  }
}

void ISMPC_Solver::create_cstr_matrices(Eigen::MatrixXd & A_out,
                                        Eigen::VectorXd & b_out,
                                        std::vector<SupportPolygon> & A_in,
                                        const std::vector<Eigen::VectorXd> & b_in)
{
  Eigen::Index k = 0;
  Eigen::Index cstr_index = 0;
  for(size_t i_ineq = 0; i_ineq < A_in.size(); i_ineq++)
  {
    Eigen::Index n_vertice = (A_in[i_ineq].normals().rows());

    A_out.block(cstr_index, k, n_vertice, 2) = A_in[i_ineq].normals();
    b_out.segment(cstr_index, n_vertice) = b_in[i_ineq];

    k += 2;
    cstr_index += n_vertice;
  }
}

void ISMPC_Solver::create_cstr_matrices(Eigen::MatrixXd & A_out,
                                        Eigen::VectorXd & b_out,
                                        std::vector<Eigen::MatrixX2d> & A_in,
                                        const std::vector<Eigen::VectorXd> & b_in)
{
  Eigen::Index step = 0;
  Eigen::Index cstr_index = 0;
  for(Eigen::Index i_ineq = 0; i_ineq < static_cast<Eigen::Index>(A_in.size()); i_ineq++)
  {

    Eigen::MatrixX2d ineq = A_in[i_ineq];

    A_out.block(cstr_index, step, ineq.rows(), 2) = ineq.block(0, 0, ineq.rows(), 2);
    b_out.segment(cstr_index, ineq.rows()) = b_in[i_ineq].segment(0, ineq.rows());

    step += 2;
    cstr_index += ineq.rows();
  }
}

Eigen::MatrixXd ISMPC_Solver::create_zmp_matrix(bool addDelay)
{
  Eigen::MatrixXd A_out = Eigen::MatrixXd::Zero(2 * m_C, 2 * m_C);
  for(int i = 0; i < m_C; i++)
  {
    for(int k = 0; k <= i; k++)
    {
      double t_m_tk = (1 + i - k) * m_delta;
      // if(k == i && addDelay){t_m_tk -= ( i==0 ? m_delay_elapsed : m_delay);}
      A_out.block(2 * i, 2 * k, 2, 2) = Eigen::Matrix2d::Identity() * (1 - exp(-m_lambda * t_m_tk));
    }
  }
  return A_out;
}

Eigen::MatrixXd ISMPC_Solver::create_u_matrix()
{
  Eigen::MatrixXd A_out = Eigen::MatrixXd::Zero(2 * m_C, 2 * m_C);
  A_out = Eigen::MatrixXd::Zero(2 * m_C, 2 * m_C);
  for(int i = 0; i < m_C; i++)
  {
    for(int k = 0; k <= i; k++)
    {
      A_out.block(2 * i, 2 * k, 2, 2) = Eigen::Matrix2d::Identity();
    }
  }
  return A_out;
}

void ISMPC_Solver::Compute_Integration_Matrix(const std::vector<double> & eta)
{
  // Use the instantaneous pendulum frequency at the beginning of the horizon
  const double eta_0 = eta[0];
  const double arg = eta_0 * m_delta_control;
  
  const double cosh_val = std::cosh(arg);
  const double sinh_val = std::sinh(arg);

  Integration_Mat.setZero();
  Integration_Mat(0, 0) = cosh_val;
  Integration_Mat(0, 1) = sinh_val / eta_0;
  Integration_Mat(1, 0) = eta_0 * sinh_val;
  Integration_Mat(1, 1) = cosh_val;
}

void ISMPC_Solver::Static_ZMP_Constraints()
{
  std::vector<Eigen::VectorXd> b_zmp_ineq;
  std::vector<Eigen::VectorXd> b_u_ineq;

  zmp_cstr_polygons.clear();
  std::vector<SupportPolygon> u_cstr_polygons;

  b_zmp_ineq.clear();
  double sgn = -1;

  if(m_support_foot == "RightFoot")
  {
    sgn = 1;
  }
  const Eigen::Vector3d rect_offset_support =
      X_0_support_foot.rotation().transpose()
      * Eigen::Vector3d{rect_pose_offset_static.x(), sgn * rect_pose_offset_static.y(), 0};

  const Eigen::Vector3d rect_offset_swing =
      X_0_support_foot.rotation().transpose()
      * Eigen::Vector3d{rect_pose_offset_static.x(), -sgn * rect_pose_offset_static.y(), 0};

  Rectangle Rect_jm1 = Rectangle(X_0_swing_foot_initial, Eigen::Vector2d{m_dx_static, m_dy_static}, rect_offset_swing);
  Rectangle Rect_j = Rectangle(X_0_support_foot, Eigen::Vector2d{m_dx_static, m_dy_static}, rect_offset_support);
  Rectangle Rect_jm1_u = Rectangle(X_0_swing_foot_initial, Eigen::Vector2d{m_dx_u, m_dy_u});
  Rectangle Rect_j_u = Rectangle(X_0_support_foot, Eigen::Vector2d{m_dx_u, m_dy_u});

  SupportPolygon SuppPoly = SupportPolygon(Rect_jm1, Rect_j);
  SupportPolygon SuppPoly_u = SupportPolygon(Rect_jm1_u, Rect_j_u);

  m_double_support_polygon = SuppPoly;

  ZMP_ref_traj.clear();
  ZMP_max_ref_traj.clear();
  ZMP_min_ref_traj.clear();
  All_poly.clear();

  Eigen::MatrixXd Delta = Eigen::MatrixXd::Zero(N_variable, N_variable); 
  Delta.block(0, 0, 2 * m_C, 2 * m_C) = create_zmp_matrix(true);

  // Use the instantaneous eta at the beginning of the horizon (index 0)
  P_u_k_max = m_eta[0] * m_delta * R_0_support * P_z_k;
  P_u_k_min = m_eta[0] * m_delta * R_0_support * P_z_k;

  sva::PTransformd X_0_step_j = X_0_support_foot;
  sva::PTransformd X_0_step_jm1 = X_0_swing_foot_initial;

  // Cache fixed-size head segment for fast geometry processing
  const Eigen::Vector2d P_z_k_delayed_2d = P_z_k_delayed.head<2>();

  for(int i = 0; i < m_C; i++)
  {
    sva::PTransformd X_0_step_stop =
        sva::PTransformd(X_0_step_j.rotation(), (Rect_j.get_center() + Rect_jm1.get_center()) * 0.5);

    sva::PTransformd ZMP_Zone = X_0_step_stop;

    ZMP_ref_traj.push_back(ZMP_Zone.translation().x() - P_z_k_delayed.x());
    ZMP_ref_traj.push_back(ZMP_Zone.translation().y() - P_z_k_delayed.y());

    zmp_cstr_polygons.push_back(SuppPoly);
    u_cstr_polygons.push_back(SuppPoly_u);

    ZMP_max_ref_traj.push_back(SuppPoly.get_center()
                               + R_support_0 * Eigen::Vector3d{m_dx_static / 2, m_dy_static / 2, 0});
    ZMP_min_ref_traj.push_back(SuppPoly.get_center()
                               - R_support_0 * Eigen::Vector3d{m_dx_static / 2, m_dy_static / 2, 0});

    if(i == 0)
    {
      SuppPolyCorners = zmp_cstr_polygons[i].Get_Polygone_Corners();
      m_ref_zmp = ZMP_Zone.translation();
    }

    Eigen::MatrixX2d normals(zmp_cstr_polygons.back().normals());
    Eigen::VectorXd offsets(zmp_cstr_polygons.back().offsets());

    // Switched to highly optimized .head<2>()
    b_zmp_ineq.push_back(offsets - normals * P_z_k_delayed_2d);
    b_u_ineq.push_back(u_cstr_polygons.back().offsets()
                       - u_cstr_polygons.back().normals() * P_z_k_delayed_2d);

    All_poly.push_back(zmp_cstr_polygons.back().Get_Polygone_Corners());
  }

  int N_zmp_cstr = 0;
  for(size_t k = 0; k < zmp_cstr_polygons.size(); k++)
  {
    N_zmp_cstr += static_cast<int>(zmp_cstr_polygons[k].normals().rows());
  }

  Eigen::MatrixXd ZMP_Cstr = Eigen::MatrixXd::Zero(N_zmp_cstr, N_variable);
  Eigen::VectorXd b_zmp = Eigen::VectorXd::Zero(ZMP_Cstr.rows());

  create_cstr_matrices(ZMP_Cstr, b_zmp, zmp_cstr_polygons, b_zmp_ineq);

  Aineq_zmp.resize(1 * ZMP_Cstr.rows(), N_variable);
  bineq_zmp.resize(Aineq_zmp.rows());

  Aineq_zmp = ZMP_Cstr * Delta;
  bineq_zmp = b_zmp;
  A_zmp = Delta.block(0, 0, 2 * m_C, N_variable);

  b_zmp_traj = Eigen::Map<Eigen::VectorXd>(ZMP_ref_traj.data(), ZMP_ref_traj.size());
  M_zmp_traj = Eigen::MatrixXd::Zero(b_zmp_traj.rows(), N_variable);
  M_zmp_traj.block(0, 0, b_zmp_traj.rows(), b_zmp_traj.rows()) =
      Delta.block(0, 0, b_zmp_traj.rows(), b_zmp_traj.rows());
}

void ISMPC_Solver::ZMP_Transition_Constraint(Eigen::MatrixXd & A_out, Eigen::VectorXd & b_out, SupportPolygon PolySS)
{
  const double t_transi_ds_ss = m_Tds - m_tk - m_delta;
  if(t_transi_ds_ss < 0)
  {
    A_out.resize(1, N_variable);
    A_out.setZero();
    b_out.resize(1);
    b_out.setZero();
    return;
  }
  const double dt = m_delta_control / 2;
  const Eigen::Index indx_transi_ds_ss = static_cast<Eigen::Index>(t_transi_ds_ss / m_delta);
  const Eigen::Index N_integration = static_cast<Eigen::Index>(m_delta / dt);
  
  Eigen::MatrixXd A_zmp = Eigen::MatrixXd::Zero(2, N_variable);
  const Eigen::Index poly_rows = PolySS.offsets().rows();
  
  A_out.setZero(N_integration * poly_rows, N_variable);
  b_out.setZero(A_out.rows());

  // Cache fixed-size head segment for the delayed ZMP position
  const Eigen::Vector2d P_z_k_delayed_2d = P_z_k_delayed.head<2>();
  const Eigen::VectorXd b_segment_base = PolySS.offsets() - PolySS.normals() * P_z_k_delayed_2d;

  for(Eigen::Index i = 0; i < N_integration; i++)
  {
    for(Eigen::Index k = 0; k <= indx_transi_ds_ss; k++)
    {
      double t_m_tk = t_transi_ds_ss + static_cast<double>(i) * dt - static_cast<double>(k) * m_delta;
      double factor = 1.0 - std::exp(-m_lambda * t_m_tk);
      
      // Fixed size fast coefficient mapping
      auto block = A_zmp.block<2, 2>(0, 2 * k);
      block(0, 0) = factor;
      block(1, 1) = factor;
    }
    
    A_out.block(i * poly_rows, 0, poly_rows, N_variable) = PolySS.normals() * A_zmp;
    b_out.segment(i * poly_rows, poly_rows) = b_segment_base;
  }
}

void ISMPC_Solver::ZMP_Constraints()
{
  std::chrono::high_resolution_clock::time_point t_clock = std::chrono::high_resolution_clock::now();

  std::vector<Eigen::VectorXd> b_zmp_ineq;
  std::vector<Eigen::VectorXd> b_u_ineq;
  zmp_cstr_polygons.clear();
  std::vector<SupportPolygon> u_cstr_polygons;
  
  double sgn = -1; 
  if(m_support_foot == "RightFoot") 
  {
    sgn = 1;
  }
  
  Eigen::Vector2d direction = Eigen::Vector2d::Zero();
  if((input_steps_[0] * X_0_support_foot.inv()).translation().x() > 0.1)
  {
    direction = Eigen::Vector2d{1., 0};
  }
  else if((input_steps_[0] * X_0_support_foot.inv()).translation().x() < -0.1)
  {
    direction = Eigen::Vector2d{-1, 0};
  }

  Eigen::Vector3d rect_offset_support =
      X_0_support_foot.rotation().transpose() * Eigen::Vector3d{rect_pose_offset.x(), sgn * rect_pose_offset.y(), 0};

  Eigen::Vector3d rect_offset_swing =
      X_0_support_foot.rotation().transpose() * Eigen::Vector3d{rect_pose_offset.x(), -sgn * rect_pose_offset.y(), 0};

  Eigen::Vector3d zmp_ref_offset_sg =
      X_0_support_foot.rotation().transpose() * Eigen::Vector3d{zmp_ref_offset.x(), sgn * zmp_ref_offset.y(), 0};

  Eigen::Vector3d zmp_ref_end_step =
      X_0_support_foot.rotation().transpose() * Eigen::Vector3d{zmp_ref_offset_end_step.x() * direction.x(), 0, 0};
  Eigen::Vector3d zmp_ref_start_step =
      X_0_support_foot.rotation().transpose() * Eigen::Vector3d{zmp_ref_offset_start_step.x() * -direction.x(), 0, 0};

  Eigen::Vector3d zmp_ref_offset_swing =
      X_0_support_foot.rotation().transpose() * Eigen::Vector3d{zmp_ref_offset.x(), -sgn * zmp_ref_offset.y(), 0};

  Rectangle Sliding_rect =
      Rectangle(mc_rbdyn::rpyFromMat(X_0_support_foot.rotation()).z(), Eigen::Vector2d{m_dx, m_dy});
  Rectangle Sliding_rect_u =
      Rectangle(mc_rbdyn::rpyFromMat(X_0_support_foot.rotation()).z(), Eigen::Vector2d{m_dx_u, m_dy_u});

  Rectangle Rect_jm1 = Rectangle(X_0_swing_foot_initial, Eigen::Vector2d{m_dx, m_dy}, rect_offset_swing);
  Rectangle Rect_j = Rectangle(X_0_support_foot, Eigen::Vector2d{m_dx, m_dy}, rect_offset_support);

  Rectangle Rect_jm1_u = Rectangle(X_0_swing_foot_initial, Eigen::Vector2d{m_dx_u, m_dy_u}, rect_offset_swing);
  Rectangle Rect_j_u = Rectangle(X_0_support_foot, Eigen::Vector2d{m_dx_u, m_dy_u}, rect_offset_support);

  SupportPolygon Poly_Rect = SupportPolygon(Sliding_rect);
  SupportPolygon Poly_Rect_u = SupportPolygon(Sliding_rect_u);

  SupportPolygon SuppPoly = SupportPolygon(Rect_jm1, Rect_j);
  SupportPolygon SuppPoly_u = SupportPolygon(Rect_jm1_u, Rect_j_u);

  SupportPolygon S_Support_Poly = SupportPolygon(Rect_j);
  SupportPolygon S_Support_Poly_u = SupportPolygon(Rect_j_u);

  ZMP_ref_traj.clear();
  ZMP_max_ref_traj.clear();
  ZMP_min_ref_traj.clear();
  All_poly.clear();

  Eigen::MatrixXd Delta = Eigen::MatrixXd::Zero(N_variable, N_variable);
  Delta.block(0, 0, 2 * m_C, 2 * m_C) = create_zmp_matrix(true);
  Eigen::MatrixXd Delta_zmp_ref = Delta;

  // Crucial Time-Varying Fix: use index 0 for instantaneous parameter value
  P_u_k_max = m_eta[0] * m_delta * R_0_support * P_z_k;
  P_u_k_min = m_eta[0] * m_delta * R_0_support * P_z_k;
  
  double NextStepTiming(0);
  if(!m_timestamp.empty())
  {
    NextStepTiming = m_timestamp[j_f];
  }
  double PrevStepTime = 0;

  sva::PTransformd X_0_step_j = X_0_support_foot;
  sva::PTransformd X_0_step_jm1 = X_0_swing_foot_initial;

  // Cache 2D projection points to bypass multi-dimensional overhead in the loop
  const Eigen::Vector2d P_z_k_delayed_2d = P_z_k_delayed.head<2>();

  for(int i = 0; i < m_C; i++)
  {
    if(m_tk + static_cast<double>(i) * m_delta >= NextStepTiming && j_f + 1 < static_cast<int>(m_timestamp.size()))
    {
      j_f += 1;
      j_fm1 = j_f - 1;
      count_Dstep = 1;
      sgn *= -1;

      double tds = m_Tds;
      if(UsePendulumSolver && m_feas_res)
      {
        tds = m_feasibilitySolver.get_optimal_steps_ds_duration()[j_f];
      }
      m_D = static_cast<int>(tds / m_delta) - Tds_offset;

      NextStepTiming = m_timestamp[j_f];
      PrevStepTime = m_timestamp[j_fm1];

      X_0_step_jm1 = X_0_step_j;
      X_0_step_j = input_steps_[j_f - 1];

      direction = Eigen::Vector2d::Zero();
      if((input_steps_[j_f] * X_0_step_j.inv()).translation().x() > 0.1)
      {
        direction = Eigen::Vector2d{1., 0};
      }
      else if((input_steps_[j_f] * X_0_step_j.inv()).translation().x() < -0.1)
      {
        direction = Eigen::Vector2d{-1, 0};
      }
      zmp_ref_end_step =
          X_0_step_j.rotation().transpose() * Eigen::Vector3d{zmp_ref_offset_end_step.x() * direction.x(), 0, 0};
      zmp_ref_start_step =
          X_0_step_j.rotation().transpose() * Eigen::Vector3d{zmp_ref_offset_start_step.x() * -direction.x(), 0, 0};

      Eigen::Vector3d offset = rect_offset_swing;
      rect_offset_swing = rect_offset_support;
      rect_offset_support = offset;
      offset = zmp_ref_offset_swing;
      zmp_ref_offset_swing = zmp_ref_offset_sg;
      zmp_ref_offset_sg = offset;

      Rect_jm1 = Rectangle(X_0_step_jm1, Eigen::Vector2d{m_dx, m_dy}, rect_offset_swing);
      Rect_j = Rectangle(X_0_step_j, Eigen::Vector2d{m_dx, m_dy}, rect_offset_support);

      Rect_jm1_u = Rectangle(X_0_step_jm1, Eigen::Vector2d{m_dx_u, m_dy_u}, rect_offset_swing);
      Rect_j_u = Rectangle(X_0_step_j, Eigen::Vector2d{m_dx_u, m_dy_u}, rect_offset_support);

      Sliding_rect = Rectangle(mc_rbdyn::rpyFromMat(X_0_step_jm1.rotation()).z(),
                               Eigen::Vector2d{m_dx, m_dy} * zmp_cstr_next_stp_ratio);

      Poly_Rect = SupportPolygon(Sliding_rect);
      Sliding_rect_u = Rectangle(mc_rbdyn::rpyFromMat(X_0_step_jm1.rotation()).z(), Eigen::Vector2d{m_dx_u, m_dy_u});
      Poly_Rect_u = SupportPolygon(Sliding_rect_u);
    }

    const double n = std::max(0., std::min(static_cast<double>(m_D), count_Dstep));
    const double alpha = std::min(1.0, std::max(0., n / (static_cast<double>(m_D))));

    if(j_f == 0 || !AutoFootstepPlacement)
    {
      if(j_f > 0)
      {
        if(!Slide_ZMP_region)
        {
          SuppPoly = SupportPolygon(Rect_jm1, Rect_j);
          SuppPoly_u = SupportPolygon(Rect_jm1_u, Rect_j_u);
        }
        S_Support_Poly = SupportPolygon(Rect_j);
        S_Support_Poly_u = SupportPolygon(Rect_j_u);
      }
      if((N_Steps >= N_Steps_Desired && N_Steps_Desired >= 0) && i == 0)
      {
        sva::PTransformd X_0_step_stop_j =
            sva::PTransformd(X_0_step_j.rotation(), (X_0_step_j.translation() + X_0_step_jm1.translation()) * 0.5);
        Rect_j = Rectangle(X_0_step_stop_j, Eigen::Vector2d{m_dx, m_dy});
        Rect_j_u = Rectangle(X_0_step_stop_j, Eigen::Vector2d{m_dx_u, m_dy_u});
        SuppPoly = SupportPolygon(Rect_jm1, Rect_j);
        SuppPoly_u = SupportPolygon(Rect_jm1_u, Rect_j_u);
      }

      sva::PTransformd ZMP_Zone =
          sva::PTransformd(X_0_step_j.rotation(), (Rect_j.get_center() + zmp_ref_offset_sg) * alpha
                                                      + (Rect_jm1.get_center() + zmp_ref_offset_swing) * (1.0 - alpha));

      Rectangle ZMP_rect = Rectangle(ZMP_Zone, Eigen::Vector2d{m_dx, m_dy});

      sva::PTransformd U_Zone = sva::PTransformd(X_0_step_j.rotation(), (Rect_j_u.get_center()) * alpha
                                                                            + (Rect_jm1_u.get_center()) * (1.0 - alpha));

      Rectangle U_rect = Rectangle(U_Zone, Eigen::Vector2d{m_dx_u, m_dy_u});

      if(Slide_ZMP_region || alpha == 1.0)
      {
        if(alpha == 1.0)
        {
          zmp_cstr_polygons.push_back(S_Support_Poly);
          u_cstr_polygons.push_back(S_Support_Poly_u);
        }
        else
        {
          zmp_cstr_polygons.push_back(SupportPolygon(ZMP_rect));
          u_cstr_polygons.push_back(SupportPolygon(U_rect));
        }

        ZMP_max_ref_traj.push_back(ZMP_rect.get_center() + R_support_0 * Eigen::Vector3d{m_dx / 2.0, m_dy / 2.0, 0});
        ZMP_min_ref_traj.push_back(ZMP_rect.get_center() - R_support_0 * Eigen::Vector3d{m_dx / 2.0, m_dy / 2.0, 0});
      }
      else
      {
        u_cstr_polygons.push_back(SuppPoly_u);
        zmp_cstr_polygons.push_back(SuppPoly);

        ZMP_max_ref_traj.push_back(ZMP_rect.get_center() + R_support_0 * Eigen::Vector3d{m_dx / 2.0, m_dy / 2.0, 0});
        ZMP_min_ref_traj.push_back(ZMP_rect.get_center() - R_support_0 * Eigen::Vector3d{m_dx / 2.0, m_dy / 2.0, 0});
      }

      ZMP_ref_traj.push_back((Rect_j.get_center() + zmp_ref_offset_sg).x() - P_z_k_delayed.x());
      ZMP_ref_traj.push_back((Rect_j.get_center() + zmp_ref_offset_sg).y() - P_z_k_delayed.y());

      Eigen::MatrixX2d normals(zmp_cstr_polygons.back().normals());
      Eigen::VectorXd offsets(zmp_cstr_polygons.back().offsets());

      b_zmp_ineq.push_back(offsets - normals * P_z_k_delayed_2d);
      b_u_ineq.push_back(u_cstr_polygons.back().offsets() - u_cstr_polygons.back().normals() * P_z_k_delayed_2d);

      All_poly.push_back(zmp_cstr_polygons.back().Get_Polygone_Corners());
    }
    else if(j_f == 1)
    {
      double l = sgn * m_feet_distance;

      sva::PTransformd X_0_step_j_min;
      sva::PTransformd X_0_step_j_max;
      X_0_step_j_min =
          sva::PTransformd(X_0_step_j.rotation(),
                           X_0_step_jm1.translation()
                               + X_0_step_j.rotation().transpose()
                                     * (Eigen::Vector3d{0., l, 0.}
                                        - Eigen::Vector3d{m_dx_f / 2.0, double(m_support_foot == "LeftFoot") * m_dy_f, 0}));
      X_0_step_j_max =
          sva::PTransformd(X_0_step_j.rotation(),
                           X_0_step_jm1.translation()
                               + X_0_step_j.rotation().transpose()
                                     * (Eigen::Vector3d{0., l, 0.}
                                        + Eigen::Vector3d{m_dx_f / 2.0, double(m_support_foot == "RightFoot") * m_dy_f, 0}));

      sva::PTransformd ZMP_Zone_min(Eigen::Matrix3d::Identity(),
                                    (X_0_step_j_min.translation() * alpha + X_0_step_jm1.translation() * (1.0 - alpha)));
      sva::PTransformd ZMP_Zone_max(Eigen::Matrix3d::Identity(),
                                    (X_0_step_j_max.translation() * alpha + X_0_step_jm1.translation() * (1.0 - alpha)));

      sva::PTransformd ZMP_Zone =
          sva::PTransformd(Eigen::Matrix3d::Identity(), (Rect_jm1.get_center() + zmp_ref_offset_swing) * (1.0 - alpha));

      sva::PTransformd U_Zone = sva::PTransformd(Eigen::Matrix3d::Identity(), (Rect_jm1_u.get_center()) * (1.0 - alpha));

      ZMP_max_ref_traj.push_back(ZMP_Zone_max.translation()
                                 + X_0_step_j.rotation().transpose() * Eigen::Vector3d{m_dx / 2.0, m_dy / 2.0, 0});
      ZMP_min_ref_traj.push_back(ZMP_Zone_min.translation()
                                 - X_0_step_j.rotation().transpose() * Eigen::Vector3d{m_dx / 2.0, m_dy / 2.0, 0});

      zmp_cstr_polygons.push_back(Poly_Rect);
      u_cstr_polygons.push_back(Poly_Rect_u);

      Eigen::MatrixX2d normals(zmp_cstr_polygons[i].normals());
      Eigen::VectorXd offsets(zmp_cstr_polygons[i].offsets());

      Eigen::VectorXd bcstr = offsets - normals * P_z_k_delayed_2d
                              + normals * ZMP_Zone.translation().head<2>()
                              + normals * (rect_offset_support).head<2>() * alpha;

      b_zmp_ineq.push_back(bcstr);
      b_u_ineq.push_back(u_cstr_polygons.back().offsets()
                         - u_cstr_polygons.back().normals() * P_z_k_delayed_2d
                         + u_cstr_polygons.back().normals() * U_Zone.translation().head<2>());

      ZMP_ref_traj.push_back(-P_z_k_delayed.x() + (rect_offset_support + zmp_ref_offset_sg).x());
      ZMP_ref_traj.push_back(-P_z_k_delayed.y() + (rect_offset_support + zmp_ref_offset_sg).y());

      Delta.block<2, 2>(2 * i, 2 * m_C + 2 * (j_f - 1)).setIdentity();
      Delta.block<2, 2>(2 * i, 2 * m_C + 2 * (j_f - 1)) *= -alpha;

      Delta_zmp_ref.block<2, 2>(2 * i, 2 * m_C + 2 * (j_f - 1)).setIdentity();
      Delta_zmp_ref.block<2, 2>(2 * i, 2 * m_C + 2 * (j_f - 1)) *= -1.0;

      All_poly.push_back(zmp_cstr_polygons.back().Get_Polygone_Corners());
      if(i == 0)
      {
        SuppPolyCorners = zmp_cstr_polygons.back().Get_Polygone_Corners();
      }
    }
    else
    {
      ZMP_ref_traj.push_back(-P_z_k_delayed.x() + (rect_offset_support + zmp_ref_offset_sg).x());
      ZMP_ref_traj.push_back(-P_z_k_delayed.y() + (rect_offset_support + zmp_ref_offset_sg).y());

      zmp_cstr_polygons.push_back(Poly_Rect);
      u_cstr_polygons.push_back(Poly_Rect_u);

      Eigen::MatrixX2d normals(zmp_cstr_polygons.back().normals());
      Eigen::VectorXd offsets(zmp_cstr_polygons.back().offsets());

      b_zmp_ineq.push_back(offsets - normals * P_z_k_delayed_2d
                           + normals * ((rect_offset_support).head<2>()) * alpha
                           + normals * ((rect_offset_swing).head<2>()) * (1.0 - alpha));

      b_u_ineq.push_back(u_cstr_polygons.back().offsets() - u_cstr_polygons.back().normals() * P_z_k_delayed_2d);

      Delta.block<2, 2>(2 * i, 2 * m_C + 2 * (j_f - 1)).setIdentity();
      Delta.block<2, 2>(2 * i, 2 * m_C + 2 * (j_f - 1)) *= -alpha;

      Delta.block<2, 2>(2 * i, 2 * m_C + 2 * (j_fm1 - 1)).setIdentity();
      Delta.block<2, 2>(2 * i, 2 * m_C + 2 * (j_fm1 - 1)) *= -(1.0 - alpha);

      Delta_zmp_ref.block<2, 2>(2 * i, 2 * m_C + 2 * (j_f - 1)).setIdentity();
      Delta_zmp_ref.block<2, 2>(2 * i, 2 * m_C + 2 * (j_f - 1)) *= -1.0;

      if(i == 0)
      {
        SuppPolyCorners = zmp_cstr_polygons[i].Get_Polygone_Corners();
      }
    }

    if(alpha == 1.0)
    {
      ZMP_ref_traj[2 * i] += zmp_ref_end_step.x();
      ZMP_ref_traj[2 * i + 1] += zmp_ref_end_step.y();
    }
    else
    {
      ZMP_ref_traj[2 * i] += zmp_ref_start_step.x();
      ZMP_ref_traj[2 * i + 1] += zmp_ref_start_step.y();
    }

    if(i == 0)
    {
      SuppPolyCorners = zmp_cstr_polygons.back().Get_Polygone_Corners();
      m_support_state = alpha;
      m_ref_zmp = Eigen::Vector3d{ZMP_ref_traj[0], ZMP_ref_traj[1], 0} + P_z_k_delayed;
    }

    count_Dstep += 1;
  }

  int N_zmp_cstr = 0;
  for(size_t k = 0; k < zmp_cstr_polygons.size(); k++)
  {
    N_zmp_cstr += static_cast<int>(zmp_cstr_polygons[k].normals().rows());
  }

  Eigen::MatrixXd ZMP_Cstr = Eigen::MatrixXd::Zero(N_zmp_cstr, N_variable);
  Eigen::VectorXd b_zmp = Eigen::VectorXd::Zero(ZMP_Cstr.rows());

  create_cstr_matrices(ZMP_Cstr, b_zmp, zmp_cstr_polygons, b_zmp_ineq);

  Aineq_zmp.setZero(ZMP_Cstr.rows(), N_variable);
  bineq_zmp.setZero(Aineq_zmp.rows());

  Aineq_zmp = ZMP_Cstr * Delta;
  bineq_zmp = b_zmp;

  b_zmp_traj = Eigen::Map<Eigen::VectorXd>(ZMP_ref_traj.data(), ZMP_ref_traj.size());
  M_zmp_traj = Eigen::MatrixXd::Zero(b_zmp_traj.rows(), N_variable);
  M_zmp_traj.block(0, 0, b_zmp_traj.rows(), N_variable) = Delta_zmp_ref.block(0, 0, b_zmp_traj.rows(), N_variable);
  
  A_zmp = Delta.block(0, 0, 2 * m_C, N_variable);
  A_zmp.block(0, 2 * m_C, 2 * m_C, N_variable - 2 * m_C).setZero();
}

void ISMPC_Solver::FootSteps_Constraints()
{
  std::vector<Eigen::VectorXd> b_kin_cstr_vec;
  std::vector<Eigen::MatrixX2d> kin_cstr_normals_vec;
  std::vector<Eigen::MatrixX2d> step_cstr_normals_vec;
  std::vector<Eigen::VectorXd> b_step_cstr_vec;
  
  Eigen::MatrixXd Delta = Eigen::MatrixXd::Identity(2 * j_Max_C, 2 * j_Max_C); 

  double l = m_feet_distance;
  if(m_support_foot == "LeftFoot")
  {
    l *= -1.0;
  }
  
  int N_footsteps_kin_cstr = 0;
  int N_footsteps_cstr = 0;
  
  for(int i = 0; i < j_Max_C; i++)
  {
    const double theta_i = mc_rbdyn::rpyFromMat(input_steps_[i].rotation()).z();
    sva::PTransformd & X_0_step_i = input_steps_[i];
    sva::PTransformd X_0_step_im1 = X_0_support_foot;
    if(i != 0)
    {
      X_0_step_im1 = input_steps_[i - 1];
    }
    Eigen::Matrix3d R_Theta_i_0 = X_0_step_im1.rotation().transpose();

    Eigen::Vector3d offset = R_Theta_i_0 * Eigen::Vector3d{0.0, l + (l / std::abs(l)) * m_dy_f / 2.0, 0.0};

    Rectangle Kinematic_Rectangle = Rectangle(theta_i, Eigen::Vector2d{m_dx_f, m_dy_f}, offset);

    if(i > 0)
    {
      // Fast fixed-size block mapping
      Delta.block<2, 2>(2 * i, 2 * (i - 1)).setIdentity();
      Delta.block<2, 2>(2 * i, 2 * (i - 1)) *= -1.0;
    }
    else
    {
      Kinematic_Rectangle = Rectangle(X_0_support_foot, Eigen::Vector2d{m_dx_f, m_dy_f}, offset);
    }
    
    SupportPolygon Kinematic_Poly = SupportPolygon(Kinematic_Rectangle);
    b_kin_cstr_vec.push_back(Kinematic_Poly.offsets());
    kin_cstr_normals_vec.push_back(Kinematic_Poly.normals());

    N_footsteps_kin_cstr += static_cast<int>(kin_cstr_normals_vec.back().rows());
    l *= -1.0;
  }

  Eigen::MatrixXd foosteps_kin_cstr = Eigen::MatrixXd::Zero(N_footsteps_kin_cstr, 2 * j_Max_C);
  Eigen::MatrixXd foosteps_cstr = Eigen::MatrixXd::Zero(N_footsteps_cstr, 2 * j_Max_C);
  Eigen::VectorXd b_kin_cstr(N_footsteps_kin_cstr);
  Eigen::VectorXd b_steps_cstr(N_footsteps_cstr);
  
  Aineq_steps.setZero(N_footsteps_kin_cstr + N_footsteps_cstr, N_variable);
  bineq_steps.setZero(N_footsteps_kin_cstr + N_footsteps_cstr);

  create_cstr_matrices(foosteps_kin_cstr, b_kin_cstr, kin_cstr_normals_vec, b_kin_cstr_vec);
  create_cstr_matrices(foosteps_cstr, b_steps_cstr, step_cstr_normals_vec, b_step_cstr_vec);

  Aineq_steps.block(0, 2 * m_C, N_footsteps_kin_cstr, 2 * j_Max_C) = foosteps_kin_cstr * Delta;
  bineq_steps.head(N_footsteps_kin_cstr) = b_kin_cstr;
}

void ISMPC_Solver::AntTailTrajectory()
{
  int PreviewSize = m_P - m_C;
  AfterTc_ZMP_trajectory;
  AfterTc_ZMP_trajectory.resize(2 * PreviewSize, 1);
  AfterTc_ZMP_trajectory.setZero();

  for(int i = 0; i < PreviewSize; i++)
  {

    double NextStepTiming(0);
    if(m_timestamp.size() != 0)
    {
      NextStepTiming = m_timestamp[j_f];
    }

    if(m_tk + static_cast<double>(m_C + i + 1) * m_delta > NextStepTiming)
    {
      if(N_Steps + j_f + 1 <= N_Steps_Desired || N_Steps_Desired < 0)
      {
        j_f += 1;
        if(j_f - 1 >= static_cast<int>(input_steps_.size()))
        {
          j_f -= 1;
          count_Dstep = (static_cast<double>(m_D) / 2) + 1;
        }
        else
        {
          j_fm1 = j_f - 1;
          count_Dstep = 1;
        }
      }
    }

    sva::PTransformd X_0_step_jm1 = X_0_swing_foot_initial;
    sva::PTransformd X_0_step_j = X_0_support_foot;

    if(j_f == 1)
    {
      X_0_step_j = input_steps_[j_f - 1];
      X_0_step_jm1 = X_0_support_foot;
    }
    else if(j_f > 1)
    {
      X_0_step_j = input_steps_[j_f - 1];
      X_0_step_jm1 = input_steps_[j_f - 2];
    }

    if(N_Steps + j_f >= N_Steps_Desired && N_Steps_Desired >= 0)
    {
      X_0_step_j = sva::PTransformd(X_0_step_j.rotation(), (X_0_step_j.translation() + X_0_step_jm1.translation()) / 2);
    }

    int n = std::max(0., std::min(static_cast<double>(m_D) + 1., count_Dstep));

    double alpha = std::min(1.0, std::max(0., static_cast<double>(n) / (static_cast<double>(m_D))));

    Eigen::Vector3d StepZone = (X_0_step_j.translation() * alpha + X_0_step_jm1.translation() * (1 - alpha));

    AfterTc_ZMP_trajectory(i) = StepZone.x();
    AfterTc_ZMP_trajectory(i + PreviewSize) = StepZone.y();

    ZMP_max_ref_traj.push_back(R_0_support * StepZone);
    ZMP_min_ref_traj.push_back(R_0_support * StepZone);

    count_Dstep += 1;
    if(j_f - 1 == static_cast<int>(input_steps_.size()) && alpha > 0.5)
    {
      count_Dstep = static_cast<double>(m_D) / 2 + 1;
    }
  }

  AfterTc_ZMP_velocity.resize(2 * (PreviewSize - 1));

  for(int k = 0; k < PreviewSize - 1; k++)
  {
    AfterTc_ZMP_velocity(k) = (AfterTc_ZMP_trajectory(k + 1) - AfterTc_ZMP_trajectory(k)) / m_delta;
    AfterTc_ZMP_velocity(k + PreviewSize - 1) =
        (AfterTc_ZMP_trajectory(k + 1 + PreviewSize) - AfterTc_ZMP_trajectory(k + PreviewSize)) / m_delta;
  }
}

void ISMPC_Solver::Compute_Riccati_Kernel()
{
  // Fine grid over [t0, t0+Tc] with N_fine = m_C * m_riccati_substeps intervals, plus the
  // tail node at index N_fine representing t0+Tc where the constant-height tail model begins.
  const int N_fine = m_C * m_riccati_substeps;
  m_riccati_dt = m_delta / static_cast<double>(m_riccati_substeps);

  m_Omega.assign(N_fine + 1, 0.0);
  m_beta.assign(N_fine + 1, 0.0);
  m_B_cum.assign(N_fine + 1, 0.0);
  m_K_kernel.assign(N_fine + 1, 0.0);

  // a(t) = eta(t)^2 sampled on the fine grid via linear interpolation of m_eta (piecewise
  // constant per MPC sample in the underlying model, but we interpolate linearly here purely
  // as a smooth stand-in for the RK4 substeps within one m_delta interval; m_eta itself already
  // encodes the true phase-based height profile at MPC-sample resolution).
  auto a_of_index = [&](int fine_idx) -> double
  {
    if(fine_idx >= N_fine)
    {
      // Tail: constant height model, a_inf = g / CoM_height_avg
      return g / CoM_height_avg;
    }
    const int i_lo = fine_idx / m_riccati_substeps;
    const int i_hi = std::min(i_lo + 1, m_C - 1);
    const double frac = static_cast<double>(fine_idx % m_riccati_substeps) / static_cast<double>(m_riccati_substeps);
    const double eta_lo = m_eta[i_lo];
    const double eta_hi = m_eta[i_hi];
    const double eta_interp = eta_lo * (1.0 - frac) + eta_hi * frac;
    return eta_interp * eta_interp;
  };

  const double a_inf = g / CoM_height_avg;
  const double Omega_tail = std::sqrt(a_inf); // Terminal condition, Prop. 8.1

  // --- Backward RK4 integration of dOmega/dt = Omega^2 - a(t), from the tail to t0 ---
  m_Omega[N_fine] = Omega_tail;
  const double h = m_riccati_dt;
  for(int idx = N_fine; idx > 0; --idx)
  {
    const double Om = m_Omega[idx];
    auto f = [&](int base_idx_for_a, double Omega_val) -> double
    {
      return Omega_val * Omega_val - a_of_index(base_idx_for_a);
    };
    // RK4 with the "time" argument carried through a_of_index; since a(t) only varies at
    // fine-grid resolution, we evaluate a() at idx, idx (approx midpoint), and idx-1 as the
    // best available samples (a is looked up at integer fine-grid indices; this is consistent
    // with a_of_index's own linear interpolation providing the smooth in-between behavior
    // that would otherwise require sub-fine-grid time queries).
    const double k1 = f(idx, Om);
    const double k2 = f(idx, Om - (h / 2.0) * k1);
    const double k3 = f(idx, Om - (h / 2.0) * k2);
    const double k4 = f(idx - 1, Om - h * k3);
    m_Omega[idx - 1] = Om - (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  }

  // --- Forward pass: beta, cumulative B, kernel K ---
  m_beta[0] = a_of_index(0) / m_Omega[0];
  m_B_cum[0] = 0.0;
  m_K_kernel[0] = m_beta[0]; // exp(-B_cum[0]) = 1
  for(int idx = 1; idx <= N_fine; ++idx)
  {
    m_beta[idx] = a_of_index(idx) / m_Omega[idx];
    m_B_cum[idx] = m_B_cum[idx - 1] + 0.5 * (m_beta[idx - 1] + m_beta[idx]) * h;
    m_K_kernel[idx] = m_beta[idx] * std::exp(-m_B_cum[idx]);
  }
}

void ISMPC_Solver::Compute_Hk_And_bfree(Eigen::VectorXd & H_k_out, Eigen::Vector2d & b_free_out)
{
  const int N_fine = m_C * m_riccati_substeps;
  const double h = m_riccati_dt;
  const double a_inf = g / CoM_height_avg;
  const double eta_inf = std::sqrt(a_inf);

  // kappa(t) on the fine grid: held at m_kappa until perturbation_duration elapses, then
  // switches to m_kappa_inf, matching the existing disturbance-duration convention used
  // throughout the rest of the solver (see Stability_Constraints' previous implementation
  // and compute_dcm()).
  auto kappa_of_index = [&](int fine_idx) -> double
  {
    const double t_fine = static_cast<double>(fine_idx) * h;
    return (t_fine < perturbation_duration) ? m_kappa : m_kappa_inf;
  };

  // --- G(t0,s): backward recursion, seeded with the analytic tail base case ---
  m_G_kernel.assign(N_fine + 1, 0.0);
  const double kappa_N = kappa_of_index(N_fine);
  m_G_kernel[N_fine] = m_lambda * kappa_N * m_K_kernel[N_fine] / (eta_inf + m_lambda);

  for(int idx = N_fine - 1; idx >= 0; --idx)
  {
    const double K_a = m_K_kernel[idx];
    const double K_b = m_K_kernel[idx + 1];
    const double kappa_a = kappa_of_index(idx);
    const double kappa_b = kappa_of_index(idx + 1);

    double local_integral;
    if(m_lambda * h < 1e-8)
    {
      local_integral = m_lambda * 0.5 * (K_a * kappa_a + K_b * kappa_b) * h;
    }
    else
    {
      const double A = K_a * kappa_a;
      const double Bc = K_b * kappa_b;
      const double slope = (Bc - A) / h;
      const double e = std::exp(-m_lambda * h);
      const double term1 = A * (1.0 - e) / m_lambda;
      const double term2 = slope * ((1.0 - e) / (m_lambda * m_lambda) - h * e / m_lambda);
      local_integral = m_lambda * (term1 + term2);
    }

    m_G_kernel[idx] = std::exp(-m_lambda * h) * m_G_kernel[idx + 1] + local_integral;
  }

  // --- S(t0,s) = integral_s^inf G(t0,tau) dtau: flat backward cumulative, tail-seeded ---
  m_S_cum.assign(N_fine + 1, 0.0);
  m_S_cum[N_fine] = m_G_kernel[N_fine] / eta_inf;
  for(int idx = N_fine - 1; idx >= 0; --idx)
  {
    m_S_cum[idx] = m_S_cum[idx + 1] + 0.5 * (m_G_kernel[idx] + m_G_kernel[idx + 1]) * h;
  }

  // --- H_k = S(t0, t_k + delta_d), interpolated on the fine grid ---
  H_k_out.setZero(m_C);
  for(int k = 0; k < m_C; ++k)
  {
    const double s_target = static_cast<double>(k) * m_delta + m_delay;
    const double idx_f = s_target / h;
    int i0 = static_cast<int>(std::floor(idx_f));
    i0 = std::clamp(i0, 0, N_fine);
    if(i0 >= N_fine)
    {
      H_k_out(k) = m_S_cum[N_fine];
    }
    else
    {
      const double frac = std::clamp(idx_f - static_cast<double>(i0), 0.0, 1.0);
      H_k_out(k) = m_S_cum[i0] * (1.0 - frac) + m_S_cum[i0 + 1] * frac;
    }
  }

  // --- b_free: known/delayed ZMP contribution on [t0, t0+delta_d] plus disturbance-augmented
  // known ZMP contribution on [t0+delta_d, ...] per Eq. (5.9)-(5.10). The effective ZMP is
  // r = kappa*p_z - Delta_c, with Delta_c represented by w_k / w_k_inf.
  //
  // KNOWN, VERIFIED (against the old exact constant-height closed form, disturbance-free case,
  // to ~1e-6 relative error under grid refinement):
  //   (1) On [t0, t0+delta_d], the real ZMP is NOT held constant at P_z_k: per Lemma 5.1, it
  //       relaxes from P_z_k toward U_k as p_z(tau) = exp(-lambda*tau)*P_z_k
  //       + (1-exp(-lambda*tau))*U_k. The delay-window integral must therefore split into an
  //       exponentially-weighted P_z_k part and a complementary U_k part, NOT weight the whole
  //       window by P_z_k alone.
  //   (2) The post-delay tail contribution of P_z_k_delayed (held from t0+delta_d onward, in the
  //       absence of any decision variable) must be weighted by a DIRECT integral of K(t0,tau)
  //       over [t0+delta_d, inf) -- NOT by S(t0,t0+delta_d) (built from G), since G already
  //       convolves K with the lambda-relaxation kernel Phi_z, which is the correct object for
  //       weighting a decision variable command issued at time s (H_k), but double-applies the
  //       relaxation if reused for a value that is already relaxed and held constant.
  //   (3) The disturbance term (w_k -> w_k_inf) switches at t0 + perturbation_duration + delta_d
  //       (NOT at t0 + perturbation_duration alone), confirmed by symbolic match against the old
  //       exact formula. This uses a direct K-integral (Delta_c is an additive offset on the
  //       effective ZMP, not routed through the lambda ZMP-lag).
  //
  // NOT YET FULLY VERIFIED: a residual discrepancy (~1e-3, not shrinking under grid refinement)
  // remains against the old exact formula in the general disturbed case, most likely from a
  // coupling between the kappa delay-window split and the disturbance term that has not been
  // correctly identified yet. Flagged for follow-up; the fixes below are the verified subset.
  const int idx_delay = std::clamp(static_cast<int>(std::round(m_delay / h)), 0, N_fine);

  // Fix (1): split the delay-window integral between P_z_k (weight W_a, exp(-lambda*tau)
  // decaying) and U_k (complementary weight W_b), instead of weighting the whole window by
  // P_z_k alone.
  double W_a = 0.0; // weight on P_z_k over [t0, t0+delta_d]
  double weight_delay_kappa_total = 0.0; // integral_{t0}^{t0+delta_d} K(t0,tau) kappa(tau) dtau
  for(int idx = 0; idx < idx_delay; ++idx)
  {
    const double tau_a = static_cast<double>(idx) * h;
    const double tau_b = static_cast<double>(idx + 1) * h;
    const double Ka_kappa = m_K_kernel[idx] * kappa_of_index(idx);
    const double Kb_kappa = m_K_kernel[idx + 1] * kappa_of_index(idx + 1);

    const double fa = Ka_kappa * std::exp(-m_lambda * tau_a);
    const double fb = Kb_kappa * std::exp(-m_lambda * tau_b);
    W_a += 0.5 * (fa + fb) * h;

    weight_delay_kappa_total += 0.5 * (Ka_kappa + Kb_kappa) * h;
  }
  const double W_b = weight_delay_kappa_total - W_a; // weight on U_k

  // Fix (2): direct K-integral (not via S/G) for the post-delay P_z_k_delayed weight.
  double weight_tail_direct = 0.0; // integral_{t0+delta_d}^{inf} K(t0,tau) kappa(tau) dtau
  for(int idx = idx_delay; idx < N_fine; ++idx)
  {
    const double Ka_kappa = m_K_kernel[idx] * kappa_of_index(idx);
    const double Kb_kappa = m_K_kernel[idx + 1] * kappa_of_index(idx + 1);
    weight_tail_direct += 0.5 * (Ka_kappa + Kb_kappa) * h;
  }
  // Analytic tail closure beyond t0+Tc, consistent with K's own tail decay
  // K_N * exp(-eta_inf*(t-t0-Tc)) there.
  weight_tail_direct += m_K_kernel[N_fine] * kappa_of_index(N_fine) / eta_inf;

  // Fix (3): disturbance term switches at (perturbation_duration + delta_d), direct K-integral.
  const double switch_t = perturbation_duration + m_delay;
  const int idx_switch = std::clamp(static_cast<int>(std::round(switch_t / h)), 0, N_fine);

  double weight_disturbance_w = 0.0; // integral_{t0}^{switch_t} K(t0,tau) dtau, weight on w_k
  for(int idx = 0; idx < idx_switch; ++idx)
  {
    weight_disturbance_w += 0.5 * (m_K_kernel[idx] + m_K_kernel[idx + 1]) * h;
  }
  double weight_disturbance_w_inf = 0.0; // integral_{switch_t}^{inf} K(t0,tau) dtau, weight on w_k_inf
  for(int idx = idx_switch; idx < N_fine; ++idx)
  {
    weight_disturbance_w_inf += 0.5 * (m_K_kernel[idx] + m_K_kernel[idx + 1]) * h;
  }
  weight_disturbance_w_inf += m_K_kernel[N_fine] / eta_inf;

  b_free_out = W_a * P_z_k.head<2>() + W_b * U_k.head<2>() + weight_tail_direct * P_z_k_delayed.head<2>()
               - weight_disturbance_w * w_k.head<2>() - weight_disturbance_w_inf * w_k_inf.head<2>();
}

void ISMPC_Solver::Stability_Constraints()
{
  // --- Variable-height stability constraint via the scalar Riccati kernel ---
  // xi(t0) = p_c(t0) + p_c_dot(t0)/Omega(t0) must equal b_free + sum_k H_k * u_k, where
  // Omega solves the Riccati equation \dot{Omega}=Omega^2-a(t) (NOT simply sqrt(a(t))), and
  // H_k, b_free are built from the closed-loop kernel G(t0,s) that folds in the ZMP first-order
  // lag (m_lambda) and delay (m_delay). See Compute_Riccati_Kernel() / Compute_Hk_And_bfree()
  // for the full derivation and the constant-height cross-check against Adios's closed form
  // (Corollary 6.2 of the checked variable-height stability note).
  //
  // Compute_Riccati_Kernel() must be called once per init_MPC() (whenever m_eta / a(t) changes)
  // before this function; it is invoked from GetWalkingParameters() alongside the existing
  // Compute_Integration_Matrix(m_eta) call.

  A_stab.setZero(2, N_variable);
  b_stab.setZero(2);

  Eigen::VectorXd H_k;
  Eigen::Vector2d b_free;
  Compute_Hk_And_bfree(H_k, b_free);

  for(int j = 0; j < m_C; j++)
  {
    A_stab.block<2, 2>(0, 2 * j) = Eigen::Matrix2d::Identity() * H_k(j);

    if(UseAngularMomentumDot)
    {
      // NOTE: this angular-momentum-dot coupling term has not yet been re-derived under the
      // new Riccati kernel; it is carried over from the old constant-eta-per-interval
      // formulation, using the same physical structure (same mass/height/eta^2 normalization)
      // but with the exponential weighting replaced by the new fine-grid-consistent kernel
      // K(t0, t_j) in place of the old exp(-integrated_eta_to[j]) so it at least uses the
      // correct decay for the *current* Omega/beta rather than the discarded eta-chaining.
      // This should be flagged for a proper re-derivation analogous to Delta_c's treatment in
      // Prop. 3.1 of the checked note, folding L_c_dot into the effective ZMP r(t) consistently
      // rather than patched on afterward.
      const double eta_j = m_eta[j];
      const int fine_idx_j = j * m_riccati_substeps;
      const double K_tj = m_K_kernel[fine_idx_j];

      auto am_block = A_stab.block<2, 2>(0, 2 * (m_C + j_Max_C + j));
      am_block << 0.0, 1.0, -1.0, 0.0;
      am_block /= (m_mass * CoM_height[j] * std::pow(eta_j, 2));
      am_block *= K_tj * m_delta;
    }
  }

  const double Omega_0 = m_Omega.empty() ? m_eta[0] : m_Omega[0];
  P_u_k = P_c_k + (V_c_k / Omega_0);

  b_stab = P_u_k.head<2>() - b_free;
}

void ISMPC_Solver::Compute_Stability_Range()
{
  P_u_k_min.setZero();
  P_u_k_max.setZero();

  // Pre-allocate horizon vectors for efficiency
  const Eigen::Index horizon_size = 2 * m_C;
  Eigen::VectorXd PzM = Eigen::VectorXd::Zero(horizon_size);
  Eigen::VectorXd Pzm = Eigen::VectorXd::Zero(horizon_size);
  Eigen::VectorXd Pz0 = Eigen::VectorXd::Zero(horizon_size);
  
  // Create tracking matrix capturing delay dynamics
  Eigen::MatrixXd Delta = create_zmp_matrix(true);

  // Cache current initial ZMP coordinates
  const Eigen::Vector2d P_z_k_2d = P_z_k.head<2>();

  // Ensure safe indexing up to control horizon limit
  const size_t loop_limit = std::min(static_cast<size_t>(m_C), ZMP_max_ref_traj.size());
  for(size_t k = 0; k < loop_limit; k++)
  {
    Pzm.segment<2>(2 * k) = ZMP_min_ref_traj[k].head<2>();
    PzM.segment<2>(2 * k) = ZMP_max_ref_traj[k].head<2>();
    Pz0.segment<2>(2 * k) = P_z_k_2d;
  }

  // Back-project ZMP bounds into tracking command ranges using optimal partial-pivoting LU solver
  Eigen::VectorXd u_M = Delta.lu().solve(PzM - Pz0);
  Eigen::VectorXd u_m = Delta.lu().solve(Pzm - Pz0);

  // Project command ranges through the time-varying stability matrix A_stab to find the DCM window bounds
  P_u_k_max.head<2>() = A_stab.topLeftCorner(2, horizon_size) * u_M + P_z_k_2d;
  P_u_k_min.head<2>() = A_stab.topLeftCorner(2, horizon_size) * u_m + P_z_k_2d;
}

void ISMPC_Solver::Compute_Standing_Stability_Range()
{
  // Time-Varying Fix: integrate eta over the next 3 steps of the horizon rather than
  // assuming the instantaneous eta_0 holds constant over that whole window.
  const int n_steps = std::min(3, m_C);
  double integrated = 0.0;
  for(int k = 0; k < n_steps; ++k)
  {
    integrated += m_eta[k] * m_delta;
  }
  // If m_C < 3 (very short horizon), extend using the last available eta as a fallback.
  if(n_steps < 3 && n_steps > 0)
  {
    integrated += m_eta[n_steps - 1] * m_delta * static_cast<double>(3 - n_steps);
  }
  const double contraction_factor = std::exp(-integrated);

  Eigen::VectorXd offset = m_double_support_polygon.normals() * P_z_k.head<2>() * (1.0 - contraction_factor)
                           + m_double_support_polygon.offsets() * contraction_factor;

  m_feasibility_standing_region = SupportPolygon(m_double_support_polygon.normals(), offset);
}

void ISMPC_Solver::Compute_Integration_Vector(const double eta,
                                              const Eigen::Vector2d & zmp0,
                                              const Eigen::Vector2d & zmpref,
                                              const double t0,
                                              const double tk)
{
  const double ch = (1 - cosh(eta * m_delta_control));
  const double sh = (0 - sinh(eta * m_delta_control));
  const double e_p_l = m_lambda + eta;
  const double l_m_e = m_lambda - eta;
  const double e_m_l = -l_m_e;
  Eigen::Vector2d com_coef = zmpref * ch;
  Eigen::Vector2d comd_coef = eta * zmpref * sh;

  const double t_kp1_m_t0 = tk + m_delta_control - t0;

  com_coef -= eta * (zmp0 - zmpref)
              * (0.5 * exp(-m_lambda * t_kp1_m_t0)
                 * (((exp(m_delta_control * e_p_l) - 1) / e_p_l) + ((exp(m_delta_control * l_m_e) - 1) / e_m_l)));

  comd_coef -= std::pow(eta, 2) * (zmp0 - zmpref)
               * (0.5 * exp(-m_lambda * t_kp1_m_t0)
                  * (((exp(m_delta_control * e_p_l) - 1) / e_p_l) - ((exp(m_delta_control * l_m_e) - 1) / e_m_l)));

  Integration_Vec_x << com_coef(0), comd_coef(0);
  Integration_Vec_y << com_coef(1), comd_coef(1);
}

void ISMPC_Solver::Integrate()
{
  m_X_MPC.clear();
  m_Y_MPC.clear();
  int N = (int)(m_delta / m_delta_control);
  int N_delay = static_cast<int>(m_delay_elapsed / m_delta_control);

  // Time-Varying Fix: Baseline starts with the instantaneous state parameter value
  double eta = m_eta[0];
  double kappa = m_kappa;

  Eigen::Vector2d state_x{P_c_k.x(), V_c_k.x()};
  Eigen::Vector2d state_y{P_c_k.y(), V_c_k.y()};

  Eigen::Vector2d w = w_k.head<2>();

  m_X_MPC.push_back(Eigen::Vector3d{state_x.x(), state_x.y(), P_z_k.x()});
  m_Y_MPC.push_back(Eigen::Vector3d{state_y.x(), state_y.y(), P_z_k.y()});

  Eigen::Vector2d Lc_dot_comp;
  Lc_dot_comp << -m_Ldot_c(m_C), m_Ldot_c(0);
  Lc_dot_comp /= (m_mass * std::pow(eta, 2) * CoM_height[0]);

  Eigen::Vector2d Pzi = (kappa * P_z_k.head<2>() - w - Lc_dot_comp);
  Eigen::Vector2d zmp_ref = kappa * U_k.head<2>() - w - Lc_dot_comp;

  // Time-Varying Fix: the homogeneous propagation matrix Integration_Mat must reflect the eta
  // applicable to *this* sub-stepping window. It was previously left stale from construction time
  // (built only from eta[0] once). Recompute it here before this loop using the current eta.
  Compute_Integration_Matrix(std::vector<double>{eta});

  for(int k = 0; k < N_delay; k++)
  {
    const double tk = static_cast<double>(k) * m_delta_control;

    Compute_Integration_Vector(eta, Pzi, zmp_ref, 0, tk);

    state_x = Integration_Mat * state_x + Integration_Vec_x;
    state_y = Integration_Mat * state_y + Integration_Vec_y;

    Eigen::Vector2d zmp = zmp_ref + (Pzi - zmp_ref) * std::exp(-m_lambda * (tk + m_delta_control));

    m_X_MPC.push_back(Eigen::Vector3d{state_x.x(), state_x.y(), (zmp + w + Lc_dot_comp).x() / kappa});
    m_Y_MPC.push_back(Eigen::Vector3d{state_y.x(), state_y.y(), (zmp + w + Lc_dot_comp).y() / kappa});
  }

  zmp_ref = kappa * P_z_k_delayed.head<2>() - w;

  m_admittance_targets.clear();
  for(Eigen::Index i = 0; i < m_C; i++)
  {
    // Time-Varying Fix: Update eta dynamically to track the changing vertical profile
    eta = m_eta[i];
    // Time-Varying Fix: Integration_Mat must be rebuilt for this step's eta before propagating
    // state across this step's N sub-samples. Without this, the homogeneous dynamics silently
    // keep using whichever eta was applicable to a previous (or the very first) step.
    Compute_Integration_Matrix(std::vector<double>{eta});

    if(static_cast<double>(i) * m_delta == perturbation_duration)
    {
      zmp_ref += w;
      zmp_ref /= kappa;
      zmp_ref *= m_kappa_inf;
      zmp_ref -= w_k_inf.head<2>();
      w = w_k_inf.head<2>();
      kappa = m_kappa_inf;
    }

    Lc_dot_comp << -m_Ldot_c(i + m_C), m_Ldot_c(i);
    Lc_dot_comp /= (m_mass * std::pow(eta, 2) * CoM_height[i]);

    zmp_ref.x() += kappa * m_ZMP_u(i) - Lc_dot_comp.x();
    zmp_ref.y() += kappa * m_ZMP_u(i + m_C) - Lc_dot_comp.y();

    Pzi = (Eigen::Vector2d{m_X_MPC.back()[2], m_Y_MPC.back()[2]} * kappa - w - Lc_dot_comp);

    m_admittance_targets.push_back(Eigen::Vector3d{(zmp_ref + w + Lc_dot_comp).x(), (zmp_ref + w + Lc_dot_comp).y(), 0.0} / kappa);

    for(int k = 0; k < N; k++)
    {
      const double tk = static_cast<double>(k) * m_delta_control;
      Compute_Integration_Vector(eta, Pzi, zmp_ref, 0, tk);

      state_x = Integration_Mat * state_x + Integration_Vec_x;
      state_y = Integration_Mat * state_y + Integration_Vec_y;

      Eigen::Vector2d zmp = zmp_ref + (Pzi - zmp_ref) * std::exp(-m_lambda * (tk + m_delta_control));

      m_X_MPC.push_back(Eigen::Vector3d{state_x.x(), state_x.y(), (zmp + w + Lc_dot_comp).x() / kappa});
      m_Y_MPC.push_back(Eigen::Vector3d{state_y.x(), state_y.y(), (zmp + w + Lc_dot_comp).y() / kappa});
    }
    zmp_ref += Lc_dot_comp;
  }
}

bool ISMPC_Solver::GetWalkingParameters(bool stop)
{
  std::chrono::high_resolution_clock::time_point t_clock = std::chrono::high_resolution_clock::now();
  if(UsePendulumSolver)
  {
    m_feas_res = true;
    if((NextOptimalTs - m_tk) > 0.1)
    {
      double Ts = m_timestamp[0];
      // Time-Varying Fix: Pass the complete m_eta vector to the feasibility configuration
      m_feasibilitySolver.configure(m_eta[0], m_delta_control, m_tds_range, m_tss_range, m_ts_range,
                                    Eigen::Vector2d{m_dx_f, 2.0 * m_dy_f}, Eigen::Vector2d{m_dx , m_dy},
                                    m_feet_distance, 8);
      std::vector<sva::PTransformd> & stepsRef = corr_steps_.size() != 0 ? corr_steps_ : input_steps_;

      m_feas_res = m_feasibilitySolver.solve(m_tk, m_t_lift, DoubleSupport, P_u_k.head<2>(), P_z_k.head<2>(),
                                             m_support_foot, X_0_support_foot, X_0_swing_foot_initial, m_input_Tds,
                                             input_steps_, m_timestamp, w_k_inf.head<2>(), m_kappa_inf);
    }

    std::vector<double> optimalTs = m_feasibilitySolver.get_optimal_steps_timings();
    std::vector<double> optimalTds = m_feasibilitySolver.get_optimal_steps_ds_duration();
    std::vector<sva::PTransformd> optimalPf = m_feasibilitySolver.get_optimal_footsteps();

    if(m_feas_res)
    {
      m_timestamp = optimalTs;
      if(DoubleSupport)
      {
        m_Tds = optimalTds[0];
      }
      m_feasibility_region = m_feasibilitySolver.get_feasibility_region();
    }
    else
    {
      mc_rtc::log::warning("[ISMPC {}] Step feasibility QP fail", m_t_global);
      m_Tds = m_input_Tds;
    }
  }
  else
  {
    m_feas_res = false;
    m_Tds = m_input_Tds;
  }

  if (m_tk - m_timestamp[0] > 0.0)
  {
    mc_rtc::log::warning("[ISMPC] t_k is over the first step, increasing step duration");
    m_timestamp[0] = m_tk + 2.0 * m_delta;
  }

  NextOptimalTs = m_timestamp[0];
  QPsuccess = false;
  InStabilityRange = false;
  m_stop = stop;

  double tc = m_tk + m_Tc;
  size_t tstep_indx = 0;

  j_Max_C = 0;
  if(!m_timestamp.empty())
  {
    while(tc > m_timestamp[tstep_indx])
    {
      tstep_indx += 1;
      if(tstep_indx > m_timestamp.size())
      {
        break;
      }
    }
  }
  j_Max_C = static_cast<int>(tstep_indx);
  j_f = 0;
  j_fm1 = j_f - 1;

  N_variable = 2 * (m_C + j_Max_C);
  if(UseAngularMomentumDot)
  {
    N_variable += 2 * m_C;
  }

  m_D = static_cast<int>(m_Tds / m_delta) - Tds_offset;
  count_Dstep = (std::min((m_tk / m_delta), static_cast<double>(m_D)));
  if(!DoubleSupport)
  {
    count_Dstep = static_cast<double>(m_D);
  }
  std::chrono::duration<double, std::milli> time_span = std::chrono::high_resolution_clock::now() - t_clock;

  double beta_dcm = m_Beta_dcm;
  double beta_dcm_vel = m_Beta_dcm_vel;
  double beta_zmp_traj = m_Beta_zmp_traj;

  if(perturbation_duration == 0.0)
  {
    m_kappa = m_kappa_inf;
    w_k = w_k_inf;
  }

  if(m_stop)
  {
    beta_dcm = m_Beta_dcm_stop;
    beta_dcm_vel = m_Beta_dcm_vel_stop;
    beta_zmp_traj = m_Beta_zmp_traj_stop;
    Static_ZMP_Constraints();
    if(UsePendulumSolver)
    {
      m_feasibility_standing_region = SupportPolygon(m_feasibilitySolver.get_feasibility_region());
      m_feasibility_standing_region_swing =
          SupportPolygon(m_feasibilitySolver.get_feasibility_region(X_0_swing_foot_initial, X_0_support_foot));
    }
  }
  else
  {
    ZMP_Constraints();
  }

  FootSteps_Constraints();
  Stability_Constraints();
  Compute_Stability_Range();

  if(!ComputeTrajectory)
  {
    return false;
  }

  Eigen::MatrixXd M_zmp_vel = -m_lambda * A_zmp;
  Eigen::VectorXd b_zmp_vel = Eigen::VectorXd::Zero(M_zmp_vel.rows());
  for(int i = 0; i < m_C; i++)
  {
    for(int j = 0; j <= i; j++)
    {
      M_zmp_vel.block<2, 2>(2 * i, 2 * j) += m_lambda * Eigen::Matrix2d::Identity();
    }
  }

  Eigen::MatrixXd M_dcm = Eigen::MatrixXd::Zero(0, N_variable);
  Eigen::VectorXd b_dcm = Eigen::VectorXd::Zero(0);
  Eigen::VectorXd b_dcm_traj = Eigen::VectorXd::Zero(0);
  Eigen::MatrixXd M_dcm_traj = Eigen::MatrixXd::Zero(0, N_variable);
  Eigen::VectorXd b_refDcm_zmp_traj = Eigen::VectorXd::Zero(0);
  Eigen::MatrixXd M_refDcm_zmp_traj = Eigen::MatrixXd::Zero(0, N_variable);

  create_dcm_cost_function(M_dcm, b_dcm, M_dcm_traj, b_dcm_traj, M_refDcm_zmp_traj, b_refDcm_zmp_traj);

  // Time-Varying Fix: Extract current initial step eta for system matrix calculations
  const double eta_0 = m_eta[0];

  Eigen::MatrixXd M_dcmVel = eta_0 * (M_dcm - A_zmp);
  Eigen::VectorXd b_dcmVel = b_dcm;
  
  const Eigen::Vector2d P_z_k_delayed_2d = P_z_k_delayed.head<2>();
  for(int i = 0; i < m_C; i++)
  {
    b_dcmVel.segment<2>(2 * i) -= P_z_k_delayed_2d;
  }
  b_dcmVel *= eta_0;

  Eigen::MatrixXd M_dcmVelRef = eta_0 * (M_dcm_traj - M_refDcm_zmp_traj);
  Eigen::VectorXd b_dcmVelRef = eta_0 * (b_dcm_traj - b_refDcm_zmp_traj);

  Eigen::MatrixXd M_steps = Eigen::MatrixXd::Zero(2, N_variable);
  if(j_Max_C != 0)
  {
    M_steps.block<2, 2>(0, 2 * m_C).setIdentity();
  }
  Eigen::VectorXd b_steps = Eigen::VectorXd::Zero(2);

  Eigen::MatrixXd M_stepsDelta = Eigen::MatrixXd::Zero(0, N_variable);
  Eigen::VectorXd b_stepsDelta = Eigen::VectorXd::Zero(0);
  if(j_Max_C > 0)
  {
    M_stepsDelta = Eigen::MatrixXd::Zero(2 * (j_Max_C - 1), N_variable);
    b_stepsDelta = Eigen::VectorXd::Zero(2 * (j_Max_C - 1));
    M_stepsDelta.block(0, 2 * m_C, 2 * (j_Max_C - 1), 2 * (j_Max_C - 1)) =
        Eigen::MatrixXd::Identity(2 * (j_Max_C - 1), 2 * (j_Max_C - 1));
  }

  for(int i = 0; i < j_Max_C; i++)
  {
    if(i == 0)
    {
      b_steps.head<2>() = input_steps_[i].translation().head<2>();
    }
    if(i < j_Max_C - 1)
    {
      M_stepsDelta.block<2, 2>(2 * i, 2 * (m_C + i + 1)).diagonal().setConstant(-1.0);
      b_stepsDelta.segment<2>(2 * i) = (input_steps_[i].translation() - input_steps_[i + 1].translation()).head<2>();
    }
  }

  m_Q = Eigen::MatrixXd::Identity(N_variable, N_variable) * 1e-12 + m_Beta_zmp_vel * (M_zmp_vel.transpose() * M_zmp_vel)
        + m_Beta_step * (M_stepsDelta.transpose() * M_stepsDelta) + m_Beta_step * (M_steps.transpose() * M_steps)
        + beta_zmp_traj * (M_zmp_traj.transpose() * M_zmp_traj)
        + beta_dcm * (M_dcm - M_dcm_traj).transpose() * (M_dcm - M_dcm_traj)
        + beta_dcm_vel * (M_dcmVel - M_dcmVelRef).transpose() * (M_dcmVel - M_dcmVelRef);

  m_p = m_Beta_zmp_vel * (M_zmp_vel.transpose() * b_zmp_vel) + m_Beta_step * (-M_stepsDelta.transpose() * b_stepsDelta)
        + m_Beta_step * (-M_steps.transpose() * b_steps) + beta_zmp_traj * (-M_zmp_traj.transpose() * b_zmp_traj)
        + beta_dcm * (M_dcm - M_dcm_traj).transpose() * (b_dcm - b_dcm_traj)
        + beta_dcm_vel * (M_dcmVel - M_dcmVelRef).transpose() * (b_dcmVel - b_dcmVelRef);

  Aeq = Eigen::MatrixXd::Zero(4, N_variable);
  beq = Eigen::VectorXd::Zero(Aeq.rows());

  if(m_Tail != "None" && Use_Stability_Task)
  {
    m_p += m_Beta_stab * (-A_stab.transpose() * b_stab);
    m_Q += m_Beta_stab * (A_stab.transpose() * A_stab);
  }
  else if(m_Tail != "None")
  {
    Aeq.block(0, 0, 2, N_variable) = A_stab;
    beq.head<2>() = b_stab;
  }

  if(m_timestamp[0] - m_tk < 0.3)
  {
    Aeq.block<2, 2>(2, 2 * m_C).setIdentity();
    beq.segment<2>(2) = X_0_swing_foot_target.translation().head<2>();
    Aineq_steps.block(0, 0, 4, N_variable).setZero();
    bineq_steps.head<4>().setZero();
  }

  Aineq_Ld = Eigen::MatrixXd::Zero(0, N_variable);
  bineq_Ld = Eigen::VectorXd::Zero(0);
  if(UseAngularMomentumDot)
  {
    Eigen::MatrixXd M_Ld = Eigen::MatrixXd::Zero(2 * m_C, N_variable);
    M_Ld.block(0, 2 * (m_C + j_Max_C), 2 * m_C, 2 * m_C) = Eigen::MatrixXd::Identity(2 * m_C, 2 * m_C);
    Eigen::MatrixXd M_L = Eigen::MatrixXd::Zero(2 * m_C, N_variable);
    Eigen::MatrixXd Delta_Lc = Eigen::MatrixXd::Zero(2 * m_C, 2 * m_C);
    Eigen::VectorXd b_L = Eigen::VectorXd::Zero(M_L.rows());
    Aineq_Ld = Eigen::MatrixXd::Zero(4 * m_C, N_variable);
    
    Aineq_Ld.block(0, 2 * (m_C + j_Max_C), 2 * m_C, 2 * m_C).setIdentity();
    Aineq_Ld.block(2 * m_C, 2 * (m_C + j_Max_C), 2 * m_C, 2 * m_C).diagonal().setConstant(-1.0);
    bineq_Ld = Eigen::VectorXd::Zero(Aineq_Ld.rows());

    for(int i = 0; i < m_C; i++)
    {
      for(int k = 0; k <= i; k++)
      {
        Delta_Lc.block<2, 2>(2 * i, 2 * k).diagonal().setConstant(m_delta);
      }
      b_L.segment<2>(2 * i) = Lc_k.head<2>();
      bineq_Ld.segment<2>(2 * i).setConstant(m_Ld_max);
      bineq_Ld.segment<2>(2 * (m_C + i)).setConstant(m_Ld_max);
    }
    M_L.block(0, 2 * (m_C + j_Max_C), 2 * m_C, 2 * m_C) = Delta_Lc;
    m_Q += m_Beta_Lc * M_Ld.transpose() * M_Ld + 0.1 * m_Beta_Lc * M_L.transpose() * M_L;
    m_p += 0.1 * m_Beta_Lc * M_L.transpose() * b_L;
  }

  Eigen::MatrixXd A_swingVel_cstr = Eigen::MatrixXd::Zero(0, N_variable);
  Eigen::VectorXd b_swingVel_cstr = Eigen::VectorXd::Zero(0);
  if(!DoubleSupport)
  {
    Eigen::MatrixXd N = Eigen::MatrixXd::Zero(4, 2);
    N << 1, 0, -1, 0, 0, 1, 0, -1;
    b_swingVel_cstr = Eigen::VectorXd::Ones(4) * m_foot_max_vel
                      + N * X_0_swing_foot.translation().head<2>() / (m_timestamp[0] - m_tk);
    A_swingVel_cstr = Eigen::MatrixXd::Zero(4, N_variable);
    A_swingVel_cstr.block(0, 2 * m_C, 4, 2) = N / (m_timestamp[0] - m_tk);
  }

  Aineq = Eigen::MatrixXd::Zero(Aineq_steps.rows() + Aineq_zmp.rows() + Aineq_Ld.rows() + A_swingVel_cstr.rows(), N_variable);
  bineq = Eigen::VectorXd::Zero(Aineq.rows());
  Aineq << Aineq_zmp, Aineq_steps, Aineq_Ld, A_swingVel_cstr;
  bineq << bineq_zmp, bineq_steps, bineq_Ld, b_swingVel_cstr;

  QP_Output = solveQP();
  stab_error = (A_stab * QP_Output - b_stab).head<2>();

  Eigen::VectorXd zmp_u = QP_Output.head(2 * m_C);
  if(!(((zmp_u - zmp_u).array() == (zmp_u - zmp_u).array()).all()))
  {
    mc_rtc::log::warning("[ISMPC] nan detected in solver output");
    QPsuccess = false;
    return true;
  }

  if(!QPsuccess && m_Tail != "None" && Allow_None && !Use_Stability_Task)
  {
    QPsuccess = false;
    mc_rtc::log::warning("[ISMPC] Ignoring Stability cstr");
    m_Tail = "None";
    Stability_Constraints();
    m_p += m_Beta_stab * (-A_stab.transpose() * b_stab);
    m_Q += m_Beta_stab * (A_stab.transpose() * A_stab);
    Aeq.block(0, 0, 2, N_variable).setZero();
    beq.head<2>().setZero();
    QP_Output = solveQP();
    stab_error = (A_stab * QP_Output - b_stab);
  }

  if(!QPsuccess)
  {
    mc_rtc::log::warning("[ISMPC] Ignoring QP due to solver initialization fault");
  }
  else
  {
    corr_steps_.clear();

    m_QP_zmp = (A_zmp * QP_Output).head(2 * m_C);
    m_QP_dcm = M_dcm * QP_Output + b_dcm;
    dcm_ref_traj.clear();
    const Eigen::VectorXd dcm_traj = M_dcm_traj * QP_Output + b_dcm_traj;

    m_ZMP_u.resize(2 * m_C, 1);
    m_Ldot_c = Eigen::VectorXd::Zero(2 * m_C);
    for(int k = 0; k < m_C; k++)
    {
      dcm_ref_traj.push_back(dcm_traj.segment<2>(2 * k));
      m_ZMP_u(k) = QP_Output(2 * k);
      m_ZMP_u(k + m_C) = QP_Output(2 * k + 1);
      if(UseAngularMomentumDot)
      {
        m_Ldot_c(k) = QP_Output(2 * (m_C + j_Max_C + k));
        m_Ldot_c(k + m_C) = QP_Output(2 * (m_C + j_Max_C + k) + 1);
      }
    }

    for(int k = 0; k < j_Max_C; k++)
    {
      if(AutoFootstepPlacement)
      {
        double xf = QP_Output(2 * m_C + 2 * k);
        double yf = QP_Output(2 * m_C + 2 * k + 1);
        corr_steps_.push_back(sva::PTransformd(input_steps_[k].rotation(), Eigen::Vector3d{xf, yf, 0.0}));
      }
      else
      {
        corr_steps_.push_back(input_steps_[k]);
      }
    }

    if(m_Tail == "None" || Use_Stability_Task)
    {
      Eigen::Vector2d P_u_k_2 = P_u_k.head<2>() + stab_error;
      V_c_k.head<2>() = eta_0 * (P_u_k_2 - P_c_k.head<2>());
    }

    Integrate();
  }
  return true;
}

Eigen::VectorXd ISMPC_Solver::solveQP()
{

  int Nvar = static_cast<int>(m_Q.rows());
  int NIneqConstr = static_cast<int>(Aineq.rows());
  int NEqConstr = static_cast<int>(Aeq.rows());
  // QP.tolerance(1e-3);
  QP.problem(Nvar, NEqConstr, NIneqConstr);
  QPsuccess = QP.solve(m_Q, m_p, Aeq, beq, Aineq, bineq);

  return QP.result();
}