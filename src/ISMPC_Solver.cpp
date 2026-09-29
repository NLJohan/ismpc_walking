#include "../include/ismpc_walking/ISMPC_Solver.h"
#include <mc_rtc/io_utils.h>

ISMPC_Solver::ISMPC_Solver() {}

ISMPC_Solver::ISMPC_Solver(double delta_controller, double delta, double Tp, double Tc)
{

  m_Tc = Tc;
  m_Tp = Tp;
  m_delta = delta;
  m_delta_control = delta_controller;

  m_eta = sqrt(mc_rtc::constants::GRAVITY / CoM_height);
  count_Dstep = 0;
  m_C = static_cast<int>(m_Tc / m_delta);
  m_P = static_cast<int>(m_Tp / m_delta);
  QPsuccess = false;

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
  CoM_height = config.stab_config.comHeight;
  m_eta = sqrt(mc_rtc::constants::GRAVITY / CoM_height);
  m_eta_free = m_eta;
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
    U_k = m_admittance_targets.empty() ? P_z_k : m_admittance_targets.front();
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

  P_u_k = P_c_k + (V_c_k / m_eta);
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
  m_eta = sqrt(mc_rtc::constants::GRAVITY / CoM_height);
  m_eta_free = m_eta;
  perturbation_duration = 0;
  Compute_Integration_Matrix(m_eta);
}

Eigen::Vector2d ISMPC_Solver::compute_dcm_delay()
{
  const double e_d_lpe = m_eta / (m_lambda + m_eta);
  const double epl = m_lambda + m_eta;
  const Eigen::Vector2d Puk = (P_c_k + (V_c_k / m_eta)).segment(0, 2);

  Eigen::Vector2d Pu_delay = Puk;
  Pu_delay -= (m_kappa * U_k - w_k).segment(0, 2) * (1 - exp(-m_eta * m_delay_elapsed));
  Pu_delay -= e_d_lpe * m_kappa * (P_z_k - U_k).segment(0, 2) * (1 - exp(-epl * m_delay_elapsed));

  Pu_delay *= exp(m_eta * m_delay_elapsed);

  return Pu_delay;
}

void ISMPC_Solver::compute_dcm(Eigen::MatrixXd & A_out,
                               Eigen::Vector2d & b_out,
                               const Eigen::Vector2d & dcm_delay,
                               const int indx)
{
  const double e_d_lpe = m_eta / (m_eta + m_lambda);
  const double lpe = m_eta + m_lambda;
  A_out = Eigen::MatrixXd::Zero(2, N_variable);
  b_out = Eigen::Vector2d::Zero();
  const double tj = static_cast<double>(indx) * m_delta;
  double tp = perturbation_duration;
  if(tj < tp)
  {
    tp = tj;
  }
  const int p = static_cast<int>(tp / m_delta);
  b_out = dcm_delay;
  b_out -= P_z_k_delayed.segment(0, 2) * (m_kappa * (1 - exp(-m_eta * tp)) + (exp(-m_eta * tp) - exp(-m_eta * tj)) * m_kappa_inf );
  b_out += w_k.segment(0, 2) * (1 - exp(-m_eta * tp)) + w_k_inf.segment(0,2) * exp(-m_eta * tp);
  b_out *= exp(m_eta * tj);

  for(int i = 0; i < indx; i++)
  {
    const double ti = static_cast<double>(i) * m_delta;
    if(i < p)
    {
      A_out.block(0, 2 * i, 2, 2) -= Eigen::Matrix2d::Identity() * exp(-m_eta * ti) * m_kappa
                                     * (1 - exp(-m_eta * (tp - ti)) - e_d_lpe * (1 - exp(-lpe * (tp - ti))));
      A_out.block(0, 2 * i, 2, 2) -= Eigen::Matrix2d::Identity() * exp(-m_eta * ti) * m_kappa_inf
                                     * (exp(-m_eta * (tp - ti)) - exp(-m_eta * (tj - ti))
                                        - e_d_lpe * (exp(-lpe * (tp - ti)) - exp(-lpe * (tj - ti))));
    }
    else
    {
      A_out.block(0, 2 * i, 2, 2) -= Eigen::Matrix2d::Identity() * exp(-m_eta * ti) * m_kappa_inf
                                     * (1 - exp(-m_eta * (tj - ti)) - e_d_lpe * (1 - exp(-lpe * (tj - ti))));
    }

    if(UseAngularMomentumDot)
    {
      A_out.block(0, 2 * (m_C + j_Max_C + i), 2, 2) << 0, -1, 1, 0;
      A_out.block(0, 2 * (m_C + j_Max_C + i), 2, 2) *= (exp(-m_eta * ti) - exp(-m_eta * (ti + m_delta)));
      A_out.block(0, 2 * (m_C + j_Max_C + i), 2, 2) /= (m_mass * CoM_height * std::pow(m_eta, 2));
    }
  }

  A_out *= exp(m_eta * tj);
}

void ISMPC_Solver::create_dcm_cost_function(Eigen::MatrixXd & M_dcm,
                                            Eigen::VectorXd & b_dcm,
                                            Eigen::MatrixXd & M_traj_dcm,
                                            Eigen::VectorXd & b_traj_dcm,
                                            Eigen::MatrixXd & M_traj_zmp,
                                            Eigen::VectorXd & b_traj_zmp)
{
  int step_indx = 0;

  double sgn = -1; // change between 1 and -1 depending of support foot (1 if right)
  if(m_support_foot == "RightFoot") // Right Support
  {
    sgn = 1;
  }
  const double sgn_init = sgn;

  const Eigen::Vector2d Pu_delayed = compute_dcm_delay();

  M_dcm = Eigen::MatrixXd::Zero(2 * m_C, N_variable);
  b_dcm = Eigen::VectorXd::Zero(2 * m_C);
  M_traj_dcm = Eigen::MatrixXd::Zero(2 * m_C, N_variable);
  b_traj_dcm = Eigen::VectorXd::Zero(2 * m_C);
  M_traj_zmp = Eigen::MatrixXd::Zero(2 * m_C, N_variable);
  b_traj_zmp = Eigen::VectorXd::Zero(2 * m_C);

  // //Pu0_stab = M_dcm_stab * x + b_dcm_stab
  Eigen::MatrixXd M_dcm_stab = Eigen::MatrixXd::Zero(2, N_variable);
  Eigen::Vector2d b_dcm_stab = Eigen::Vector2d::Zero(2);

  const Eigen::Vector2d offset = Eigen::Vector2d{0, m_dy / 2};
  const Eigen::Matrix2d R_support_0 = X_0_support_foot.rotation().transpose().block(0, 0, 2, 2);
  const Eigen::Vector2d P_support = X_0_support_foot.translation().segment(0, 2) + sgn * R_support_0 * offset;

  double ts_im1 = 0;

  if(!m_stop)
  {

    for(int i = 0; i < m_timestamp.size(); i++)
    {
      double ts_i = m_timestamp[i] - m_tk;
      if(i == m_timestamp.size() - 1)
      {
        ts_i = 1e9;
      }

      Eigen::Matrix2d R_i_0;
      Eigen::Vector2d P_i = P_support;

      if(i == 0)
      {
        b_dcm_stab += (exp(-m_eta * ts_im1) - exp(-m_eta * ts_i)) * P_support;
      }
      else
      {
        R_i_0 = input_steps_[i - 1].rotation().transpose().block(0, 0, 2, 2);
        P_i = input_steps_[i - 1].translation().segment(0, 2) + sgn * R_i_0 * offset;

        if(i - 1 < j_Max_C)
        {
          M_dcm_stab.block(0, 2 * (m_C + i - 1), 2, 2) =
              Eigen::Matrix2d::Identity() * (exp(-m_eta * ts_im1) - exp(-m_eta * ts_i));

          b_dcm_stab += R_i_0 * sgn * offset * (exp(-m_eta * ts_im1) - exp(-m_eta * ts_i));
        }
        else
        {
          b_dcm_stab += (exp(-m_eta * ts_im1) - exp(-m_eta * ts_i)) * P_i;
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

  Eigen::Matrix2d R_step_0 = X_0_support_foot.rotation().transpose().block(0, 0, 2, 2);
  Eigen::Matrix2d R_PrevStep_0 = X_0_support_foot.rotation().transpose().block(0, 0, 2, 2);

  sgn = sgn_init;
  Eigen::Vector2d P_stp = P_support;
  Eigen::Vector2d P_PrevStp = P_support;
  Eigen::Vector2d prevOffset = Eigen::Vector2d::Zero();

  for(int i = 0; i < m_C; i++)
  {
    Eigen::MatrixXd A_dcm = Eigen::MatrixXd::Zero(2, N_variable);
    Eigen::Vector2d c_dcm = Eigen::Vector2d::Zero();
    compute_dcm(A_dcm, c_dcm, Pu_delayed, i + 1);
    M_dcm.block(2 * i, 0, 2, N_variable) = A_dcm;
    b_dcm.segment(2 * i, 2) = c_dcm;
    if(!m_stop)
    {
      if(t + m_delta >= ts)
      {
        t_m_PrevTs = (t + m_delta - ts);
        prevOffset = sgn * R_step_0 * offset;
        P_PrevStp = P_stp;

        indx_step += 1;
        ts = m_timestamp[indx_step + 1];
        if(indx_step == m_timestamp.size() - 1)
        {
          ts = 1e6;
        }
        R_step_0 = input_steps_[indx_step].rotation().transpose().block(0, 0, 2, 2);
        sgn *= -1;
        if(indx_step != -1 && indx_step > j_Max_C)
        {
          P_stp = input_steps_[indx_step].translation().segment(0, 2) + sgn * R_step_0 * offset;
        }
      }

      if(indx_step - 1 < 0 || indx_step - 1 >= j_Max_C)
      {
        b_traj_dcm.segment(2 * i, 2) -= ((exp(m_eta * m_delta) - exp(m_eta * t_m_PrevTs)) * P_PrevStp);
        b_traj_zmp.segment(2 * i, 2) = P_PrevStp;
      }
      if(indx_step < 0 || indx_step >= j_Max_C)
      {

        b_traj_dcm.segment(2 * i, 2) -= ((exp(m_eta * t_m_PrevTs) - 1) * P_stp);
        b_traj_zmp.segment(2 * i, 2) = P_stp;
      }
      if(indx_step - 1 >= 0 && indx_step - 1 < j_Max_C)
      {

        b_traj_dcm.segment(2 * i, 2) -= ((exp(m_eta * m_delta) - exp(m_eta * t_m_PrevTs)) * prevOffset);
        M_traj_dcm.block(2 * i, 2 * (m_C + indx_step - 1), 2, 2) -=
            (exp(m_eta * m_delta) - exp(m_eta * t_m_PrevTs)) * Eigen::Matrix2d::Identity();

        b_traj_zmp.segment(2 * i, 2) = prevOffset;
        M_traj_zmp.block(2 * i, 2 * (m_C + indx_step - 1), 2, 2) = Eigen::Matrix2d::Identity();
      }
      if(indx_step >= 0 && indx_step < j_Max_C)
      {

        b_traj_dcm.segment(2 * i, 2) -= (exp(m_eta * t_m_PrevTs) - 1) * sgn * R_step_0 * offset;
        M_traj_dcm.block(2 * i, 2 * (m_C + indx_step), 2, 2) -=
            (exp(m_eta * t_m_PrevTs) - 1) * Eigen::Matrix2d::Identity();

        b_traj_zmp.segment(2 * i, 2) = sgn * R_step_0 * offset;
        M_traj_zmp.block(2 * i, 2 * (m_C + indx_step), 2, 2) = Eigen::Matrix2d::Identity();
      }
      t_m_PrevTs = m_delta;

      // adding previous dcm
      if(i == 0)
      {
        M_traj_dcm.block(0, 0, 2, N_variable) = exp(m_eta * m_delta) * M_dcm_stab;
        b_traj_dcm.segment(0, 2) += exp(m_eta * m_delta) * b_dcm_stab;
      }
      else
      {
        M_traj_dcm.block(2 * i, 0, 2, N_variable) +=
            exp(m_eta * m_delta) * M_traj_dcm.block(2 * (i - 1), 0, 2, N_variable);
        b_traj_dcm.segment(2 * i, 2) += exp(m_eta * m_delta) * b_traj_dcm.segment(2 * (i - 1), 2);
      }
      t += m_delta;
    }
    else
    {
      b_traj_dcm.segment(2 * i, 2) = m_ref_zmp.segment(0, 2);
      b_traj_zmp.segment(2 * i, 2) = m_ref_zmp.segment(0, 2);
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

void ISMPC_Solver::Compute_Integration_Matrix(const double eta)
{
  Integration_Mat.setZero();
  Integration_Mat(0, 0) = std::cosh(eta * m_delta_control);
  Integration_Mat(0, 1) = std::sinh(eta * m_delta_control) / eta;
  Integration_Mat(1, 0) = eta * std::sinh(eta * m_delta_control);
  Integration_Mat(1, 1) = std::cosh(eta * m_delta_control);
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

  Eigen::MatrixXd Delta = Eigen::MatrixXd::Zero(N_variable, N_variable); // Matrix to derive the ZMP position from u
  Eigen::MatrixXd DeltaNoDelay =
      Eigen::MatrixXd::Zero(N_variable, N_variable); // Matrix to derive the ZMP position from u
  Eigen::MatrixXd u_Delta = Delta;
  Delta.block(0, 0, 2 * m_C, 2 * m_C) = create_zmp_matrix(true);
  // DeltaNoDelay.block(0,0,2*m_C,2*m_C) = create_zmp_matrix(false);
  // u_Delta.block(0,0,2*m_C,2*m_C) = create_u_matrix();

  P_u_k_max = m_eta * m_delta * R_0_support * P_z_k;
  P_u_k_min = m_eta * m_delta * R_0_support * P_z_k;

  sva::PTransformd X_0_step_j = X_0_support_foot;
  sva::PTransformd X_0_step_jm1 = X_0_swing_foot_initial;

  for(int i = 0; i < m_C; i++)
  {

    sva::PTransformd X_0_step_stop =
        sva::PTransformd(X_0_step_j.rotation(), (Rect_j.get_center() + Rect_jm1.get_center()) * 0.5);

    sva::PTransformd ZMP_Zone = X_0_step_stop;

    // zmp location in the horizon is as such
    // zmp_i = P_z_k_delayed + A_zmp * X (having X the decision variables)
    // The ZMP reference trajectory in the QP is as such
    // min | zmp_i - zmp_ref|^2 => min |M X - b|^2
    // ZMP_ref_traj is the b part of the cost function
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

    // zmp location in the horizon is as such
    // zmp_i = P_z_k_delayed + Delta * X (having X the decision variables)
    // The cstr in the QP is as such
    // N_i * zmp_i < O_i
    b_zmp_ineq.push_back(offsets - normals * P_z_k_delayed.segment(0, 2));
    b_u_ineq.push_back(u_cstr_polygons.back().offsets()
                       - u_cstr_polygons.back().normals() * P_z_k_delayed.segment(0, 2));

    All_poly.push_back(zmp_cstr_polygons.back().Get_Polygone_Corners());
  }

  int N_zmp_cstr = 0;
  for(size_t k = 0; k < zmp_cstr_polygons.size(); k++)
  {
    N_zmp_cstr += static_cast<int>(zmp_cstr_polygons[k].normals().rows());
  }
  int N_u_cstr = 0;
  for(size_t k = 0; k < u_cstr_polygons.size(); k++)
  {
    N_u_cstr += static_cast<int>(u_cstr_polygons[k].normals().rows());
  }
  // mc_rtc::log::success("ZMP cstr computed, Ncstr = {}", N_zmp_cstr);
  Eigen::MatrixXd ZMP_Cstr = Eigen::MatrixXd::Zero(N_zmp_cstr, N_variable);
  Eigen::VectorXd b_zmp = Eigen::VectorXd::Zero(ZMP_Cstr.rows());
  // Eigen::MatrixXd U_Cstr = Eigen::MatrixXd::Zero(N_u_cstr, N_variable);
  // Eigen::VectorXd b_u = Eigen::VectorXd::Zero(U_Cstr.rows());

  // std::cout << "ZMP_cstr_rows" << ZMP_Cstr.rows() << std::endl;

  create_cstr_matrices(ZMP_Cstr, b_zmp, zmp_cstr_polygons, b_zmp_ineq);

  // create_cstr_matrices(U_Cstr,b_u,u_cstr_polygons,b_u_ineq);

  Aineq_zmp.resize(1 * ZMP_Cstr.rows(), N_variable);
  bineq_zmp.resize(Aineq_zmp.rows());

  Aineq_zmp << ZMP_Cstr * Delta; // , ZMP_Cstr * DeltaNoDelay;
  bineq_zmp << b_zmp; //, b_zmp;
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
  A_out.resize(N_integration * PolySS.offsets().rows(), N_variable);
  A_out.setZero();
  b_out.resize(A_out.rows());
  b_out.setZero();

  for(Eigen::Index i = 0; i < N_integration; i++)
  {
    for(Eigen::Index k = 0; k <= indx_transi_ds_ss; k++)
    {
      double t_m_tk = t_transi_ds_ss + static_cast<double>(i) * dt - static_cast<double>(k) * m_delta;
      // if(k == i){t_m_tk -= ( i==0 ? m_delay_elapsed : m_delay);}
      A_zmp.block(0, 2 * k, 2, 2) = Eigen::Matrix2d::Identity() * (1 - exp(-m_lambda * (t_m_tk)));
    }
    A_out.block(i * PolySS.offsets().rows(), 0, PolySS.offsets().rows(), N_variable) = PolySS.normals() * A_zmp;
    b_out.segment(i * PolySS.offsets().rows(), PolySS.offsets().rows()) =
        PolySS.offsets() - PolySS.normals() * P_z_k_delayed.segment(0, 2);
  }
}

void ISMPC_Solver::ZMP_Constraints()
{
  std::chrono::high_resolution_clock::time_point t_clock = std::chrono::high_resolution_clock::now();

  // std::chrono::high_resolution_clock::time_point t_clock_0 = std::chrono::high_resolution_clock::now();

  std::vector<Eigen::VectorXd> b_zmp_ineq = std::vector<Eigen::VectorXd>{};
  std::vector<Eigen::VectorXd> b_u_ineq = std::vector<Eigen::VectorXd>{};
  zmp_cstr_polygons = std::vector<SupportPolygon>{};
  std::vector<SupportPolygon> u_cstr_polygons = std::vector<SupportPolygon>{};
  double sgn = -1; // change between 1 and -1 depending of support foot (1 if right)
  if(m_support_foot == "RightFoot") // Right Support
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

  // std::chrono::duration<double, std::milli> time_span_0 = std::chrono::high_resolution_clock::now() - t_clock_0;
  // mc_rtc::log::info("[ZMP cstr init] offset {} ms", time_span_0.count());

  // t_clock_0 = std::chrono::high_resolution_clock::now();
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

  // TOFIX Make the controller crash when robot is falling
  // Eigen::MatrixXd Aineq_zmp_transi = Eigen::MatrixXd::Zero(0,N_variable);
  // Eigen::VectorXd bineq_zmp_transi = Eigen::VectorXd::Zero(0);
  // ZMP_Transition_Constraint(Aineq_zmp_transi,bineq_zmp_transi,S_Support_Poly);

  ZMP_ref_traj.clear();
  ZMP_max_ref_traj.clear();
  ZMP_min_ref_traj.clear();
  All_poly.clear();

  // Delta will convert the decision variable into the ZMP location w.r.t the footsteps location
  // It is a square matrix as we must keep the X size the same fot the QP
  Eigen::MatrixXd Delta = Eigen::MatrixXd::Zero(N_variable, N_variable);
  Delta.block(0, 0, 2 * m_C, 2 * m_C) = create_zmp_matrix(true);
  // Matrix similar to Delta but with a different footsteps location dependency to generate the reference zmp traj
  Eigen::MatrixXd Delta_zmp_ref = Delta;

  P_u_k_max = m_eta * m_delta * R_0_support * P_z_k;
  P_u_k_min = m_eta * m_delta * R_0_support * P_z_k;
  double NextStepTiming(0);
  if(m_timestamp.size() != 0)
  {
    NextStepTiming = m_timestamp[j_f];
    // std::cout << "Ts " << NextStepTiming << std::endl;
  }
  double PrevStepTime = 0;

  sva::PTransformd X_0_step_j = X_0_support_foot;
  sva::PTransformd X_0_step_jm1 = X_0_swing_foot_initial;

  for(int i = 0; i < m_C; i++)
  {

    if(m_tk + static_cast<double>(i) * m_delta >= NextStepTiming && j_f + 1 < static_cast<int>(m_timestamp.size()))
    {

      // j_f = std::min(j_f + 1, (int)input_steps_.size() - 1);
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

    // alpha is a factor to place the ZMP cstr square between both feet
    const double alpha = std::min(1.0, std::max(0., n / (static_cast<double>(m_D))));

    // Depending on j_f, the zmp constraint region depends on no footsteps (current double support poylgon)
    // One footsteps (first step and current support foot)
    // Two footsteps
    if(j_f == 0 || !AutoFootstepPlacement)
    {
      // t_clock_0 = std::chrono::high_resolution_clock::now();
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
                                                      + (Rect_jm1.get_center() + zmp_ref_offset_swing) * (1 - alpha));

      Rectangle ZMP_rect = Rectangle(ZMP_Zone, Eigen::Vector2d{m_dx, m_dy});

      sva::PTransformd U_Zone = sva::PTransformd(X_0_step_j.rotation(), (Rect_j_u.get_center()) * alpha
                                                                            + (Rect_jm1_u.get_center()) * (1 - alpha));

      Rectangle U_rect = Rectangle(U_Zone, Eigen::Vector2d{m_dx_u, m_dy_u});

      if(Slide_ZMP_region || alpha == 1)
      {
        if(alpha == 1)
        {
          zmp_cstr_polygons.push_back(S_Support_Poly);
          u_cstr_polygons.push_back(S_Support_Poly_u);
        }
        else
        {
          zmp_cstr_polygons.push_back(SupportPolygon(ZMP_rect));
          u_cstr_polygons.push_back(SupportPolygon(U_rect));
        }

        ZMP_max_ref_traj.push_back(ZMP_rect.get_center() + R_support_0 * Eigen::Vector3d{m_dx / 2, m_dy / 2, 0});
        ZMP_min_ref_traj.push_back(ZMP_rect.get_center() - R_support_0 * Eigen::Vector3d{m_dx / 2, m_dy / 2, 0});
      }
      else
      {
        u_cstr_polygons.push_back(SuppPoly_u);
        zmp_cstr_polygons.push_back(SuppPoly);

        ZMP_max_ref_traj.push_back(ZMP_rect.get_center() + R_support_0 * Eigen::Vector3d{m_dx / 2, m_dy / 2, 0});
        ZMP_min_ref_traj.push_back(ZMP_rect.get_center() - R_support_0 * Eigen::Vector3d{m_dx / 2, m_dy / 2, 0});
      }

      // zmp location in the horizon is as such
      // zmp_i = P_z_k_delayed + A_zmp * X (having X the decision variables)
      // The ZMP reference trajectory in the QP is as such
      // min | zmp_i - zmp_ref|^2 => min |M X - b|^2
      // ZMP_ref_traj is the b part of the cost function
      ZMP_ref_traj.push_back((Rect_j.get_center() + zmp_ref_offset_sg).x() - P_z_k_delayed.x());
      ZMP_ref_traj.push_back((Rect_j.get_center() + zmp_ref_offset_sg).y() - P_z_k_delayed.y());

      Eigen::MatrixX2d normals(zmp_cstr_polygons.back().normals());
      Eigen::VectorXd offsets(zmp_cstr_polygons.back().offsets());

      // zmp location in the horizon is as such
      // zmp_i = P_z_k_delayed + Delta * X (having X the decision variables)
      // The cstr in the QP is as such
      // N_i * zmp_i < O_i
      b_zmp_ineq.push_back(offsets - normals * P_z_k_delayed.segment(0, 2));
      b_u_ineq.push_back(u_cstr_polygons.back().offsets()
                         - u_cstr_polygons.back().normals() * P_z_k_delayed.segment(0, 2));

      All_poly.push_back(zmp_cstr_polygons.back().Get_Polygone_Corners());

      // time_span_0 = std::chrono::high_resolution_clock::now() - t_clock_0;
      // mc_rtc::log::info("[ZMP cstr] Cstr at j {} took {} ms", j_f ,time_span_0.count());
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
                                        - Eigen::Vector3d{m_dx_f / 2, int(m_support_foot == "LeftFoot") * m_dy_f, 0}));
      X_0_step_j_max =
          sva::PTransformd(X_0_step_j.rotation(),
                           X_0_step_jm1.translation()
                               + X_0_step_j.rotation().transpose()
                                     * (Eigen::Vector3d{0., l, 0.}
                                        + Eigen::Vector3d{m_dx_f / 2, int(m_support_foot == "RightFoot") * m_dy_f, 0}));

      sva::PTransformd ZMP_Zone_min(Eigen::Matrix3d::Identity(),
                                    (X_0_step_j_min.translation() * alpha + X_0_step_jm1.translation() * (1 - alpha)));
      sva::PTransformd ZMP_Zone_max(Eigen::Matrix3d::Identity(),
                                    (X_0_step_j_max.translation() * alpha + X_0_step_jm1.translation() * (1 - alpha)));

      sva::PTransformd ZMP_Zone =
          sva::PTransformd(Eigen::Matrix3d::Identity(), (Rect_jm1.get_center() + zmp_ref_offset_swing) * (1 - alpha));

      sva::PTransformd U_Zone = sva::PTransformd(Eigen::Matrix3d::Identity(), (Rect_jm1_u.get_center()) * (1 - alpha));

      ZMP_max_ref_traj.push_back(ZMP_Zone_max.translation()
                                 + X_0_step_j.rotation().transpose() * Eigen::Vector3d{m_dx / 2, m_dy / 2, 0});
      ZMP_min_ref_traj.push_back(ZMP_Zone_min.translation()
                                 - X_0_step_j.rotation().transpose() * Eigen::Vector3d{m_dx / 2, m_dy / 2, 0});

      zmp_cstr_polygons.push_back(Poly_Rect);
      u_cstr_polygons.push_back(Poly_Rect_u);

      Eigen::MatrixX2d normals(zmp_cstr_polygons[i].normals());
      Eigen::VectorXd offsets(zmp_cstr_polygons[i].offsets());

      Eigen::VectorXd bcstr = offsets - normals * P_z_k_delayed.segment(0, 2)
                              + normals * ZMP_Zone.translation().segment(0, 2)
                              + normals * (rect_offset_support).segment(0, 2) * alpha;

      b_zmp_ineq.push_back(bcstr);
      b_u_ineq.push_back(u_cstr_polygons.back().offsets()
                         - u_cstr_polygons.back().normals() * P_z_k_delayed.segment(0, 2)
                         + u_cstr_polygons.back().normals() * U_Zone.translation().segment(0, 2));

      ZMP_ref_traj.push_back(-P_z_k_delayed.x() + (rect_offset_support + zmp_ref_offset_sg).x());
      ZMP_ref_traj.push_back(-P_z_k_delayed.y() + (rect_offset_support + zmp_ref_offset_sg).y());

      Delta.block(2 * i, 2 * m_C + 2 * (j_f - 1), 2, 2) = -Eigen::Matrix2d::Identity() * alpha;
      Delta_zmp_ref.block(2 * i, 2 * m_C + 2 * (j_f - 1), 2, 2) = -Eigen::Matrix2d::Identity();

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

      b_zmp_ineq.push_back(offsets - normals * P_z_k_delayed.segment(0, 2)
                           + normals * ((rect_offset_support).segment(0, 2)) * alpha
                           + normals * ((rect_offset_swing).segment(0, 2)) * (1 - alpha));

      b_u_ineq.push_back(u_cstr_polygons.back().offsets()
                         - u_cstr_polygons.back().normals() * P_z_k_delayed.segment(0, 2));

      Delta.block(2 * i, 2 * m_C + 2 * (j_f - 1), 2, 2) = -Eigen::Matrix2d::Identity() * alpha;
      Delta.block(2 * i, 2 * m_C + 2 * (j_fm1 - 1), 2, 2) = -Eigen::Matrix2d::Identity() * (1 - alpha);

      Delta_zmp_ref.block(2 * i, 2 * m_C + 2 * (j_f - 1), 2, 2) = -Eigen::Matrix2d::Identity();

      if(i == 0)
      {
        SuppPolyCorners = zmp_cstr_polygons[i].Get_Polygone_Corners();
      }
    }
    if(alpha == 1)
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

  // std::cout << ("[ZMP cstr] generated") << std::endl;

  // t_clock = std::chrono::high_resolution_clock::now();

  int N_zmp_cstr = 0;
  int N_u_cstr = 0;
  for(size_t k = 0; k < zmp_cstr_polygons.size(); k++)
  {
    N_zmp_cstr += static_cast<int>(zmp_cstr_polygons[k].normals().rows());
  }
  for(size_t k = 0; k < u_cstr_polygons.size(); k++)
  {
    N_u_cstr += static_cast<int>(u_cstr_polygons[k].normals().rows());
  }

  Eigen::MatrixXd ZMP_Cstr = Eigen::MatrixXd::Zero(N_zmp_cstr, N_variable);

  Eigen::MatrixXd U_Cstr = Eigen::MatrixXd::Zero(N_u_cstr, N_variable);

  Eigen::VectorXd b_zmp = Eigen::VectorXd::Zero(ZMP_Cstr.rows());
  // Eigen::VectorXd b_u = Eigen::VectorXd::Zero(U_Cstr.rows());

  // ZMP_Cstr * X <= b_zmp (X are the decision variables)
  create_cstr_matrices(ZMP_Cstr, b_zmp, zmp_cstr_polygons, b_zmp_ineq);
  // create_cstr_matrices(U_Cstr,b_u,u_cstr_polygons,b_u_ineq);

  // Eigen::MatrixXd u_Delta = Delta;
  // u_Delta.block(0,0,2*m_C,2*m_C) = create_u_matrix();
  // Eigen::MatrixXd DeltaNoDelay = Delta;
  // DeltaNoDelay.block(0,0,2*m_C,2*m_C) = create_zmp_matrix(false);

  Aineq_zmp = Eigen::MatrixXd::Zero(ZMP_Cstr.rows(), N_variable);
  bineq_zmp = Eigen::VectorXd::Zero(Aineq_zmp.rows());

  Aineq_zmp << ZMP_Cstr * Delta;
  bineq_zmp << b_zmp;

  b_zmp_traj = Eigen::Map<Eigen::VectorXd>(ZMP_ref_traj.data(), ZMP_ref_traj.size());
  M_zmp_traj = Eigen::MatrixXd::Zero(b_zmp_traj.rows(), N_variable);
  M_zmp_traj.block(0, 0, b_zmp_traj.rows(), N_variable) = Delta_zmp_ref.block(0, 0, b_zmp_traj.rows(), N_variable);
  A_zmp = Delta.block(0, 0, 2 * m_C, N_variable);
  // We remove the footsteps component part
  A_zmp.block(0, 2 * m_C, 2 * m_C, N_variable - 2 * m_C).setZero();

  // time_span = std::chrono::high_resolution_clock::now() - t_clock;
  // mc_rtc::log::info("[ZMP cstr] matrix gen time {} ms", time_span.count());
  // std::cout << ("[ZMP cstr] matrix built") << std::endl;
}

void ISMPC_Solver::FootSteps_Constraints()
{
  std::vector<Eigen::VectorXd> b_kin_cstr_vec;
  std::vector<Eigen::MatrixX2d> kin_cstr_normals_vec;
  std::vector<Eigen::MatrixX2d> step_cstr_normals_vec;
  std::vector<Eigen::VectorXd> b_step_cstr_vec;
  Eigen::MatrixXd Delta = Eigen::MatrixXd::Identity(2 * j_Max_C, 2 * j_Max_C); // Matrix to differentiate two footsteps

  double l = m_feet_distance;
  if(m_support_foot == "LeftFoot")
  {
    l *= -1;
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

    Eigen::Vector3d offset = R_Theta_i_0 * Eigen::Vector3d{0, l + (l / std::abs(l)) * m_dy_f / 2, 0};

    Rectangle Kinematic_Rectangle = Rectangle(theta_i, Eigen::Vector2d{m_dx_f, m_dy_f}, offset);

    if(i > 0)
    {
      Delta.block(2 * i, 2 * (i - 1), 2, 2) = -Eigen::Matrix2d::Identity();
    }
    else
    {
      Kinematic_Rectangle = Rectangle(X_0_support_foot, Eigen::Vector2d{m_dx_f, m_dy_f}, offset);

      // Rectangle step_admissible_region_rect = Rectangle(X_0_step_i, Eigen::Vector2d{m_dx_f_rect, m_dy_f_rect});
      // SupportPolygon step_admissible_region_poly = SupportPolygon(step_admissible_region_rect);

      // step_cstr_normals_vec.push_back(step_admissible_region_poly.normals());
      // b_step_cstr_vec.push_back(step_admissible_region_poly.offsets());
      // N_footsteps_cstr += static_cast<int>(step_cstr_normals_vec.back().rows());
    }
    SupportPolygon Kinematic_Poly = SupportPolygon(Kinematic_Rectangle);
    b_kin_cstr_vec.push_back(Kinematic_Poly.offsets());
    kin_cstr_normals_vec.push_back(Kinematic_Poly.normals());

    N_footsteps_kin_cstr += static_cast<int>(kin_cstr_normals_vec.back().rows());
    l *= -1;
  }

  Eigen::MatrixXd foosteps_kin_cstr = Eigen::MatrixXd::Zero(N_footsteps_kin_cstr, 2 * (j_Max_C));
  Eigen::MatrixXd foosteps_cstr = Eigen::MatrixXd::Zero(N_footsteps_cstr, 2 * (j_Max_C));
  Eigen::VectorXd b_kin_cstr(N_footsteps_kin_cstr);
  Eigen::VectorXd b_steps_cstr(N_footsteps_cstr);
  Aineq_steps = Eigen::MatrixXd::Zero(N_footsteps_kin_cstr + N_footsteps_cstr, N_variable);
  bineq_steps = Eigen::VectorXd::Zero(N_footsteps_kin_cstr + N_footsteps_cstr);

  create_cstr_matrices(foosteps_kin_cstr, b_kin_cstr, kin_cstr_normals_vec, b_kin_cstr_vec);
  create_cstr_matrices(foosteps_cstr, b_steps_cstr, step_cstr_normals_vec, b_step_cstr_vec);

  Aineq_steps.block(0, 2 * m_C, N_footsteps_kin_cstr, 2 * j_Max_C) = foosteps_kin_cstr * Delta;
  bineq_steps.segment(0, N_footsteps_kin_cstr) = b_kin_cstr;
  // Aineq_steps.block(N_footsteps_kin_cstr, 2 * m_C, N_footsteps_cstr, 2 * j_Max_C) = foosteps_cstr;
  // bineq_steps.segment(N_footsteps_kin_cstr,N_footsteps_cstr) = b_steps_cstr;
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

void ISMPC_Solver::Stability_Constraints()
{

  Eigen::Vector3d c_k;
  c_k.setZero();

  A_stab = Eigen::MatrixXd::Zero(2, N_variable);
  b_stab = Eigen::VectorXd::Zero(2);
  Eigen::Vector3d u_delay = U_k - P_z_k;
  const double l_d_l_p_e = (m_lambda / (m_lambda + m_eta));
  const double e_d_l_p_e = (m_eta / (m_lambda + m_eta));

  double t = 0;
  double duration = m_delta + m_delay_elapsed;
  const double disturbance_duration = perturbation_duration;

  for(int j = 0; j < m_C; j++)
  {
    const double tj = static_cast<double>(j) * m_delta;
    if(tj >= perturbation_duration)
    {
      A_stab.block(0, 2 * j, 2, 2) = Eigen::Matrix2d::Identity() * l_d_l_p_e * m_kappa_inf * exp(-m_eta * tj);
    }
    else
    {
      A_stab.block(0, 2 * j, 2, 2) =
          Eigen::Matrix2d::Identity() * exp(-m_eta * tj)
          * (m_kappa * (1 - exp(-m_eta * (perturbation_duration - tj)))
             + m_kappa_inf * exp(-m_eta * (perturbation_duration - tj))
             + e_d_l_p_e
                   * (m_kappa * (exp(-(m_eta + m_lambda) * (perturbation_duration - tj)) - 1)
                      - m_kappa_inf * exp(-(m_eta + m_lambda) * (perturbation_duration - tj))));
    }

    if(UseAngularMomentumDot)
    {
      A_stab.block(0, 2 * (m_C + j_Max_C + j), 2, 2) << 0, 1, -1, 0;
      A_stab.block(0, 2 * (m_C + j_Max_C + j), 2, 2) /= (m_mass * CoM_height * std::pow(m_eta, 2));
      A_stab.block(0, 2 * (m_C + j_Max_C + j), 2, 2) *= exp(-m_eta * t) * (1 - exp(-m_eta * duration));
      t += duration;
      duration = m_delta;
    }
  }
  A_stab.block(0, 0, 2, 2 * m_C) *= exp(-m_eta * m_delay_elapsed);

  P_u_k = P_c_k + (V_c_k / m_eta);
  // b_stab = (P_u_k
  //         - (
  //           P_z_k
  //           + l_d_l_p_e * (U_k - P_z_k)
  //           - l_d_l_p_e * (U_k - P_z_k_delayed) * exp(-m_eta * m_delay_elapsed)
  //           - w_k * ( 1 - exp(-m_eta * (disturbance_duration + m_delay_elapsed )))
  //           )
  //         ).segment(0,2);
  b_stab = P_u_k.segment(0, 2);
  b_stab -= m_kappa
            * (U_k * (1 - exp(-m_eta * m_delay_elapsed))
               + e_d_l_p_e * (P_z_k - U_k) * (1 - exp(-(m_eta + m_lambda) * m_delay_elapsed)))
                  .segment(0, 2);

  b_stab -= exp(-m_eta * m_delay_elapsed) * P_z_k_delayed.segment(0, 2)
            * (m_kappa * (1 - exp(-m_eta * perturbation_duration)) + m_kappa_inf * exp(-m_eta * perturbation_duration));
  b_stab -= -(w_k * (1 - exp(-m_eta * (perturbation_duration + m_delay_elapsed)))
              + w_k_inf * exp(-m_eta * (perturbation_duration + m_delay_elapsed)))
                 .segment(0, 2);
}

void ISMPC_Solver::Compute_Stability_Range()
{
  P_u_k_min.setZero();
  P_u_k_max.setZero();

  Eigen::Vector3d PzM_k;
  Eigen::Vector3d u_m_k;
  Eigen::Vector3d u_M_k;

  Eigen::MatrixXd Delta; // 2*m_C square Matrix to derive the ZMP position from u
  Eigen::VectorXd PzM = Eigen::VectorXd::Zero(2 * m_C);
  Eigen::VectorXd Pzm = Eigen::VectorXd::Zero(2 * m_C);
  Eigen::VectorXd Pz0 = Eigen::VectorXd::Zero(2 * m_C);
  Delta = create_zmp_matrix(true);

  // mc_rtc::log::info("ZMP boundrie size {}\nControl size {}",ZMP_max_ref_traj.size(),m_C);
  for(size_t k = 1; k < ZMP_max_ref_traj.size(); k++)
  {
    Pzm.segment(2 * k, 2) = ZMP_min_ref_traj[k].head(2);
    PzM.segment(2 * k, 2) = ZMP_max_ref_traj[k].head(2);
    Pz0.segment(2 * k, 2) = P_z_k.segment(0, 2);
  }
  Eigen::VectorXd u_M = Delta.inverse() * (PzM - Pz0);
  Eigen::VectorXd u_m = Delta.inverse() * (Pzm - Pz0);
  P_u_k_max.segment(0, 2) = A_stab.topLeftCorner(2, 2 * m_C) * u_M + P_z_k.segment(0, 2);
  P_u_k_min.segment(0, 2) = A_stab.topLeftCorner(2, 2 * m_C) * u_m + P_z_k.segment(0, 2);
}

void ISMPC_Solver::Compute_Standing_Stability_Range()
{
  Eigen::VectorXd offset = m_double_support_polygon.normals() * P_z_k.segment(0, 2) * (1 - exp(-m_eta * 3 * m_delta))
                           + m_double_support_polygon.offsets() * exp(-m_eta * 3 * m_delta);
  m_feasibility_standing_region = SupportPolygon(m_double_support_polygon.normals(), offset);
  // for(auto & p : m_feasibility_standing_region.Get_Polygone_Corners())
  // {
  //   mc_rtc::log::info(p);
  // }
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
  int N_perturbation = static_cast<int>(perturbation_duration / m_delta_control);
  double eta = m_eta;
  double kappa = m_kappa;

  Eigen::Vector2d state_x = Eigen::Vector2d{P_c_k.x(), V_c_k.x()};
  Eigen::Vector2d state_y = Eigen::Vector2d{P_c_k.y(), V_c_k.y()};

  Eigen::Vector2d w = w_k.segment(0, 2);

  m_X_MPC.push_back(Eigen::Vector3d{state_x.x(), state_x.y(), P_z_k.x()});
  m_Y_MPC.push_back(Eigen::Vector3d{state_y.x(), state_y.y(), P_z_k.y()});

  Eigen::Vector2d Lc_dot_comp = Eigen::Vector2d::Zero();
  Lc_dot_comp << -m_Ldot_c(m_C), m_Ldot_c(0);
  Lc_dot_comp /= (m_mass * std::pow(eta, 2) * CoM_height);
  Eigen::Vector2d Pzi = (kappa * P_z_k.segment(0, 2) - w - Lc_dot_comp);

  Eigen::Vector2d zmp_ref = kappa * U_k.segment(0, 2) - w - Lc_dot_comp;

  for(int k = 0; k < N_delay; k++)
  {
    const double tk = static_cast<double>(k) * m_delta_control;

    Compute_Integration_Vector(eta, Pzi, zmp_ref, 0, tk);

    state_x = Integration_Mat * state_x + Integration_Vec_x;
    state_y = Integration_Mat * state_y + Integration_Vec_y;

    Eigen::Vector2d zmp = zmp_ref + (Pzi - zmp_ref) * exp(-m_lambda * (tk + m_delta_control));

    m_X_MPC.push_back(Eigen::Vector3d{state_x.x(), state_x.y(), (zmp + w + Lc_dot_comp).x() / kappa});
    m_Y_MPC.push_back(Eigen::Vector3d{state_y.x(), state_y.y(), (zmp + w + Lc_dot_comp).y() / kappa});
  }

  zmp_ref = kappa * P_z_k_delayed.segment(0, 2) - w;

  m_admittance_targets.clear();
  for(Eigen::Index i = 0; i < m_C; i++)
  {

    if(static_cast<double>(i) * m_delta == perturbation_duration)
    {
      zmp_ref += w;
      zmp_ref /= kappa;
      zmp_ref *= m_kappa_inf;
      zmp_ref -= w_k_inf.segment(0, 2);
      w = w_k_inf.segment(0, 2);
      kappa = m_kappa_inf;
    }
    Lc_dot_comp << -m_Ldot_c(i + m_C), m_Ldot_c(i);
    Lc_dot_comp /= (m_mass * std::pow(eta, 2) * CoM_height);

    zmp_ref.x() += kappa * m_ZMP_u(i) - Lc_dot_comp.x();
    zmp_ref.y() += kappa * m_ZMP_u(i + m_C) - Lc_dot_comp.y();

    Pzi = (Eigen::Vector2d{m_X_MPC.back()[2], m_Y_MPC.back()[2]} * kappa - w - Lc_dot_comp);

    m_admittance_targets.push_back(Eigen::Vector3d{(zmp_ref + w + Lc_dot_comp).x(), (zmp_ref + w + Lc_dot_comp).y(), 0}
                                   / kappa);

    for(int k = 0; k < N; k++)
    {
      const double tk = static_cast<double>(k) * m_delta_control;
      Compute_Integration_Vector(eta, Pzi, zmp_ref, 0, tk);

      state_x = Integration_Mat * state_x + Integration_Vec_x;
      state_y = Integration_Mat * state_y + Integration_Vec_y;

      Eigen::Vector2d zmp = zmp_ref + (Pzi - zmp_ref) * exp(-m_lambda * (tk + m_delta_control));

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
      m_feasibilitySolver.configure(m_eta, m_delta_control, m_tds_range, m_tss_range, m_ts_range,
                                    Eigen::Vector2d{m_dx_f, 2 * m_dy_f}, Eigen::Vector2d{m_dx , m_dy},
                                    m_feet_distance, 8);
      std::vector<sva::PTransformd> & stepsRef = corr_steps_.size() != 0 ? corr_steps_ : input_steps_;

      m_feas_res = m_feasibilitySolver.solve(m_tk, m_t_lift, DoubleSupport, P_u_k.segment(0, 2), P_z_k.segment(0, 2),
                                             m_support_foot, X_0_support_foot, X_0_swing_foot_initial, m_input_Tds,
                                             input_steps_, m_timestamp,w_k_inf.segment(0,2),m_kappa_inf);
    }

    std::vector<double> optimalTs = m_feasibilitySolver.get_optimal_steps_timings();
    std::vector<double> optimalTds = m_feasibilitySolver.get_optimal_steps_ds_duration();
    // for (int k = 1 ; k < optimalTs.size() ; k++)
    // {
    //   optimalTs[k] = optimalTs[0] + k * Ts;
    //   optimalTds[k] = m_input_Tds;
    // }

    // mc_rtc::log::info("Ts {}",mc_rtc::io::to_string(optimalTs));
    // mc_rtc::log::info("Tds {}",mc_rtc::io::to_string(optimalTds));
    std::vector<sva::PTransformd> optimalPf = m_feasibilitySolver.get_optimal_footsteps();

    // mc_rtc::log::info("optimal t {}",timings);
    // mc_rtc::log::info("optimal tds {}",timings_ds);
    // if(optimalPf.size() != 0)
    // {
    //   mc_rtc::log::info("ref p {}",input_steps_[0].translation());
    //   mc_rtc::log::info("optimal p {}",optimalPf[0].translation());
    // }
    if(m_feas_res)
    {
      m_timestamp = optimalTs;
      if(DoubleSupport)
      {
        m_Tds = optimalTds[0];
      }
      // input_steps_ = optimalPf;
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

  if (m_tk - m_timestamp[0] > 0)
  {
    mc_rtc::log::warning("[ISMPC] t_k is over the first step, increasing step duration");
    m_timestamp[0] = m_tk + 2 * m_delta;
  }

  NextOptimalTs = m_timestamp[0];
  QPsuccess = false;
  InStabilityRange = false;
  m_stop = stop;

  double tc = m_tk + m_Tc;
  size_t tstep_indx = 0;

  j_Max_C = 0;
  if(m_timestamp.size() != 0)
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

  // mc_rtc::log::info("countD {}, m_D {} ,t_k : {}",count_Dstep,m_D,m_tk);
  // m_Tc,m_timestamp[0],m_Tds,j_Max_C);
  // mc_rtc::log::info("m_C {}",m_C); t_clock = std::chrono::high_resolution_clock::now();
  double beta_dcm = m_Beta_dcm;
  double beta_dcm_vel = m_Beta_dcm_vel;
  double beta_zmp_traj = m_Beta_zmp_traj;

  if(perturbation_duration == 0)
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
    // Compute_Standing_Stability_Range();
  }
  else
  {
    ZMP_Constraints();
  }
  // time_span = std::chrono::high_resolution_clock::now() - t_clock;
  // mc_rtc::log::info("ZMP cstr time {} ms",time_span.count());

  // t_clock = std::chrono::high_resolution_clock::now();
  FootSteps_Constraints();
  // time_span = std::chrono::high_resolution_clock::now() - t_clock;
  // mc_rtc::log::info("Steps cstr time {} ms",time_span.count());

  // t_clock = std::chrono::high_resolution_clock::now();
  Stability_Constraints();
  // time_span = std::chrono::high_resolution_clock::now() - t_clock;
  // mc_rtc::log::info("Stab cstr time {} ms",time_span.count());
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
      M_zmp_vel.block(2 * i, 2 * j, 2, 2) += m_lambda * Eigen::Matrix2d::Identity();
    }
  }
  // M_zmp_vel.block(0, 0, 2 * m_C, 2 * m_C) = Eigen::MatrixXd::Identity(2 * m_C , 2 * m_C);

  Eigen::MatrixXd M_dcm = Eigen::MatrixXd::Zero(0, N_variable);
  Eigen::VectorXd b_dcm = Eigen::VectorXd::Zero(0);
  Eigen::VectorXd b_dcm_traj = Eigen::VectorXd::Zero(0);
  Eigen::MatrixXd M_dcm_traj = Eigen::MatrixXd::Zero(0, N_variable);
  Eigen::VectorXd b_refDcm_zmp_traj = Eigen::VectorXd::Zero(0);
  Eigen::MatrixXd M_refDcm_zmp_traj = Eigen::MatrixXd::Zero(0, N_variable);

  create_dcm_cost_function(M_dcm, b_dcm, M_dcm_traj, b_dcm_traj, M_refDcm_zmp_traj, b_refDcm_zmp_traj);

  Eigen::MatrixXd M_dcmVel = m_eta * (M_dcm - A_zmp);
  Eigen::VectorXd b_dcmVel = b_dcm;
  for(int i = 0; i < m_C; i++)
  {
    b_dcmVel.segment(2 * i, 2) -= P_z_k_delayed.segment(0, 2);
  }
  b_dcmVel *= m_eta;

  Eigen::MatrixXd M_dcmVelRef = m_eta * (M_dcm_traj - M_refDcm_zmp_traj);
  Eigen::VectorXd b_dcmVelRef = m_eta * (b_dcm_traj - b_refDcm_zmp_traj);

  // Eigen::MatrixXd M_steps = Eigen::MatrixXd::Zero(2*j_Max_C, N_variable);
  // M_steps.block(0, 2 * m_C, 2 * j_Max_C, 2 * j_Max_C) = Eigen::MatrixXd::Identity(2 * j_Max_C, 2 * j_Max_C);
  // Eigen::VectorXd b_steps = Eigen::VectorXd::Zero(2*j_Max_C);

  Eigen::MatrixXd M_steps = Eigen::MatrixXd::Zero(2, N_variable);
  if(j_Max_C != 0)
  {
    M_steps.block(0, 2 * m_C, 2 , 2) = Eigen::MatrixXd::Identity(2, 2);
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
      b_steps.segment(2 * i, 2) = input_steps_[i].translation().segment(0, 2);
    }
    if(i < j_Max_C - 1)
    {
      M_stepsDelta.block(2 * i, 2 * (m_C + i + 1), 2, 2) = -Eigen::Matrix2d::Identity();
      b_stepsDelta.segment(2 * i, 2) =
          (input_steps_[i].translation() - input_steps_[i + 1].translation()).segment(0, 2);
    }
  }

  // t_clock = std::chrono::high_resolution_clock::now();

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
    beq.segment(0, 2) = b_stab;
  }

  if(m_timestamp[0] - m_tk < 0.3)
  {
    Aeq.block(2, 2 * m_C, 2, 2) = Eigen::Matrix2d::Identity();
    beq.segment(2, 2) = X_0_swing_foot_target.translation().segment(0, 2);
    Aineq_steps.block(0, 0, 4, N_variable).setZero();
    bineq_steps.segment(0, 4).setZero();
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
    Aineq_Ld.block(0, 2 * (m_C + j_Max_C), 2 * m_C, 2 * m_C) = Eigen::MatrixXd::Identity(2 * m_C, 2 * m_C);
    Aineq_Ld.block(2 * m_C, 2 * (m_C + j_Max_C), 2 * m_C, 2 * m_C) = -Eigen::MatrixXd::Identity(2 * m_C, 2 * m_C);
    bineq_Ld = Eigen::VectorXd::Zero(Aineq_Ld.rows());

    for(int i = 0; i < m_C; i++)
    {
      for(int k = 0; k <= i; k++)
      {
        Delta_Lc.block(2 * i, 2 * k, 2, 2) = Eigen::Matrix2d::Identity() * m_delta;
      }
      b_L.segment(2 * i, 2) = Lc_k.segment(0, 2);
      bineq_Ld.segment(2 * i, 2) = Eigen::Vector2d::Ones() * m_Ld_max;
      bineq_Ld.segment(2 * (m_C + i), 2) = Eigen::Vector2d::Ones() * m_Ld_max;
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
                      + N * X_0_swing_foot.translation().segment(0, 2) / (m_timestamp[0] - m_tk);
    A_swingVel_cstr = Eigen::MatrixXd::Zero(4, N_variable);
    A_swingVel_cstr.block(0, 2 * m_C, 4, 2) = N / (m_timestamp[0] - m_tk);
  }

  Aineq = Eigen::MatrixXd::Zero(Aineq_steps.rows() + Aineq_zmp.rows() + Aineq_Ld.rows() + A_swingVel_cstr.rows(),
                                N_variable);
  bineq = Eigen::VectorXd::Zero(Aineq.rows());
  Aineq << Aineq_zmp, Aineq_steps, Aineq_Ld, A_swingVel_cstr;
  bineq << bineq_zmp, bineq_steps, bineq_Ld, b_swingVel_cstr;

  QP_Output = solveQP();
  stab_error = (A_stab * QP_Output - b_stab).block(0, 0, 2, 1);

  // std::cout << "QP out " << QP_Output << std::endl;

  Eigen::VectorXd zmp_u = QP_Output.segment(0, 2 * m_C);
  // mc_rtc::log::info(zmp_u);
  if(!(((zmp_u - zmp_u).array() == (zmp_u - zmp_u).array()).all()))
  {

    mc_rtc::log::warning("[ISMPC] nan");
    QPsuccess = false;
    return true;
  }

  // time_span = std::chrono::high_resolution_clock::now() - t_clock;
  // mc_rtc::log::success("ZMPvel QP computed in {} ms ",time_span.count());

  if(!QPsuccess && m_Tail != "None" && Allow_None && !Use_Stability_Task)
  {

    QPsuccess = false;

    mc_rtc::log::warning("[ISMPC] Ignoring Stability cstr");
    m_Tail = "None";
    Stability_Constraints();
    m_p += m_Beta_stab * (-A_stab.transpose() * b_stab);
    m_Q += m_Beta_stab * (A_stab.transpose() * A_stab);
    Aeq.block(0, 0, 2, N_variable).setZero();
    beq.segment(0, 2).setZero();
    QP_Output = solveQP();
    stab_error = (A_stab * QP_Output - b_stab);
    mc_rtc::log::warning("[ISMPC] stab error {}", stab_error);
  }

  if(!QPsuccess)
  {

    // mc_m_Ldot_crtc::log::error_and_throw<std::runtime_error>("QP Failed");
    mc_rtc::log::warning("[ISMPC] Ignoring QP");

    // Eigen::VectorXd ineq = Aineq * QP_Output - bineq;
    //  for (int i = 0 ; i < ineq.rows() ; i++)
    //  {
    //    double in(ineq(i));
    //    if (in > 0)
    //    {
    //      mc_rtc::log::info("ctsr broken idx {}, {}",i,in);
    //    }
    //  }
  }

  else
  {

    corr_steps_.clear();

    m_QP_zmp = (A_zmp * QP_Output).segment(0, 2 * m_C);
    m_QP_dcm = M_dcm * QP_Output + b_dcm;
    dcm_ref_traj.clear();
    const Eigen::VectorXd dcm_traj = M_dcm_traj * QP_Output + b_dcm_traj;

    m_ZMP_u.resize(2 * m_C, 1);
    m_Ldot_c = Eigen::VectorXd::Zero(2 * m_C);
    for(int k = 0; k < m_C; k++)
    {
      dcm_ref_traj.push_back(dcm_traj.segment(2 * k, 2));
      m_ZMP_u(k) = QP_Output(2 * k);
      m_ZMP_u(k + m_C) = QP_Output(2 * k + 1);
      if(UseAngularMomentumDot)
      {
        // m_Ldot_c(k) = std::min( std::max( QP_Output(2 * (m_C + j_Max_C + k)) , -m_Ld_max),m_Ld_max);
        // m_Ldot_c(k + m_C) = std::min( std::max( QP_Output(2 * (m_C + j_Max_C + k) + 1), -m_Ld_max),m_Ld_max);
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

        corr_steps_.push_back(sva::PTransformd(input_steps_[k].rotation(), Eigen::Vector3d{xf, yf, 0}));
      }
      else
      {
        corr_steps_.push_back(input_steps_[k]);
      }
    }

    if(m_Tail == "None" || Use_Stability_Task)
    {

      Eigen::Vector2d P_u_k_2 = P_u_k.segment(0, 2) + stab_error;
      V_c_k.segment(0, 2) = m_eta * (P_u_k_2 - P_c_k.segment(0, 2));
      // P_c_k.segment(0,2) = P_u_k_2 - P_c_k.segment(0,2)/m_eta;

      Eigen::Vector2d P_u_error = P_u_k.segment(0, 2) - P_u_k_2;

      // mc_rtc::log::info("P_u_error \n{}",P_u_error);
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
