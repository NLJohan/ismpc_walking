#pragma once
#include <mc_control/api.h>
#include <mc_control/mc_controller.h>
#include "ControllerConfiguration.h"
#include "MPC_state.h"
#include "Polygon.h"
#include "eigen-quadprog/QuadProg.h"
#include "eigen-quadprog/eigen_quadprog_api.h"
#include <pendulum_feasibility_solver/feasibility_solver.h>
#include <thread>


class ISMPC_Solver
{
public:
  ISMPC_Solver();

  /**
   * Initialize the fixed parameters of the MPC
   * @tparam delta_controller : controller timestep
   * @tparam delta : MPC timestep
   * @tparam Tp : Preview horizon lenght, horizon where footsteps and trajectory will be computed
   * @tparam Tc : Control horizon lenght, horizon where the CoM/ZMP trajectory will be computed (Tc < Tp)
   */
  ISMPC_Solver(double delta_controller, double delta, double Tp, double Tc);

  ~ISMPC_Solver() = default;
  /**
   * Initialize the fixed parameters of the MPC
   * @tparam delta_controller : controller timestep
   * @tparam delta : MPC timestep
   * @tparam Tp : Preview horizon lenght, horizon where footsteps and trajectory will be computed
   * @tparam Tc : Control horizon lenght, horizon where the CoM/ZMP trajectory will be computed (Tc < Tp)
   * @tparam Beta : Weight of the footsteps position in the QP cost function
   */
  void Init(double delta_controller, double delta, double Tp, double Tc, double Beta);

  void init_MPC(const MPC_state & mpc_state, std::string Tail, int Steps_Desired, int Step);

  /**
   * Set the robot walking charateristics
   * @tparam p_c_k , v_c_k , p_z_k : Initial CoM , CoMd and ZMP position
   * @tparam Pfm1 : Previous swing foot or actual support foot position
   * @tparam timesstp , timesindx : Steps timing and their indexes in the horizon
   * @tparam Tail , choice of the velocity tail (Truncated, Periodic, Anticipative or None)
   * @tparam Steps_Desired, steps choosen to be performed, the number of steps done must be updated manually with Steps
   */
  void SetWalkingParameters(const Eigen::Vector3d & p_c_k,
                            const Eigen::Vector3d & v_c_k,
                            const Eigen::Vector3d & p_z_k,
                            const Eigen::Vector3d & Pfm1,
                            const std::vector<double> & timesstp,
                            std::string Tail,
                            int Steps_Desired,
                            int Steps);

  /**
   * Compute the CoM, CoMd, ZMP trajectory for previously set Walking parameters
   * QP is build such as the output vector contains :
   * -The ZMP reference in x and y (world frame) ordered by timesteps then x then y
   * -The Optimized Footstep (if computed) in x and y ordered by timesteps then x then y
   * -The inequality constraints are set in the constraints matrix such as the first part represent the zmp position
   * constraints and then the Footsteps position constraints
   */
  bool GetWalkingParameters(bool stop);

  /**
   * @brief Set The constraints region for the ZMP (during each delta time) and the footsteps in the robot frame
   *
   * @param ZMP
   * @param FootSteps
   */
  void MPC_Constraints_region(Eigen::Vector2d ZMP, Eigen::Vector2d FootSteps)
  {
    m_dx = ZMP.x();
    m_dy = ZMP.y();
    m_dx_f = FootSteps.x();
    m_dy_f = FootSteps.y();
  }
  // void MPC_Constraints_region(Eigen::Vector2d ZMP){
  //     m_dx = ZMP.x() ; m_dy = ZMP.y();
  // }
  // void MPC_Constraints_region(Eigen::Vector2d FootSteps){
  //     m_dx_f = FootSteps.x() ; m_dy_f = FootSteps.y();
  // }
  Eigen::Vector2d ZMP_Constraints_Size()
  {
    return Eigen::Vector2d{m_dx, m_dy};
  }
  Eigen::Vector2d Footsteps_Constraints_Size()
  {
    return Eigen::Vector2d{m_dx_f, m_dy_f};
  }

  double delta_mpc() const noexcept
  {
    return m_delta;
  }
  double delta_control() const noexcept
  {
    return m_delta_control;
  }

  /**
   * Returns if previous MPC was feasible
   */
  const bool & QPsucceeded() const noexcept
  {
    return QPsuccess;
  }
  const std::string & Tail() const noexcept
  {
    return m_Tail;
  }

  const std::vector<sva::PTransformd> & inputs_steps() const noexcept
  {
    return input_steps_;
  }

  const std::vector<sva::PTransformd> & optimal_steps() const noexcept
  {
    return corr_steps_;
  }

  const std::vector<double> timesteps()
  {
    return m_timestamp;
  }

  double Tds()
  {
    return m_Tds;
  }

  /**
   * Read-only diagnostic accessors -- NOT for control logic, only for
   * confirming episode-boundary reset behaviour via logging. NextOptimalTs
   * and m_feas_res are conditionally (not unconditionally) rewritten by
   * GetWalkingParameters(), so they can silently carry a stale or NaN value
   * from the previous episode across a Walking_controller::reset() call if
   * ResetEpisodeState() below is not invoked.
   */
  double PeekNextOptimalTs() const noexcept
  {
    return NextOptimalTs;
  }
  bool PeekFeasRes() const noexcept
  {
    return m_feas_res;
  }

  /**
   * Reset all internal solver state that persists across
   * GetWalkingParameters() calls (i.e. is only conditionally, not
   * unconditionally, overwritten each call) back to the same values used
   * at construction. Must be called once per RL episode boundary, from
   * Walking_controller::reset() -- this object is a single member of
   * Walking_controller constructed once for the controller's whole
   * lifetime and is never reconstructed per-episode, so without this call
   * it silently inherits state (including a possible NaN in NextOptimalTs/
   * m_timestamp) from the end of one episode into the start of the next.
   * Does NOT touch m_feasibilitySolver's own internal state -- that is a
   * separate, not-yet-investigated persistence risk (see feasibility
   * solver work, tracked separately).
   */
  void ResetEpisodeState();

  /**
   * Dump EVERY private/protected member of this class via mc_rtc::log::warning,
   * tagged with `tag` so two calls (e.g. "episode_N_start" / "episode_N+1_start")
   * can be grepped and diffed directly from the log file to prove -- not infer --
   * whether any state survives an RL episode boundary. See ISMPC_Solver.cpp for
   * the full member list this covers (164 of 170 declared members; QP, an internal
   * QuadProgDense working object rebuilt fresh every solveQP() call, is the only
   * deliberate exclusion; the remaining 5 non-member tokens were parser noise).
   * Diagnostic only -- no effect on control.
   */
  void DumpState(const std::string & tag);

  /**
   * Returns the computed trajectory, each vector3d in the vector contains the CoM , CoMd and ZMP value for a time step
   */
  const std::vector<Eigen::Vector3d> & X_MPC() const noexcept
  {
    return m_X_MPC;
  }
  /**
   * Returns the computed trajectory, each vector3d in the vector contains the CoM , CoMd and ZMP value for a time step
   */
  const std::vector<Eigen::Vector3d> & Y_MPC() const noexcept
  {
    return m_Y_MPC;
  }

  /**
   * Return the feasibility boundries in term of initial DCM in support foot frame
   */
  const Eigen::Vector3d & Puk_min()
  {
    return P_u_k_min;
  }

  /**
   * Return the feasibility boundries in term of initial DCM in support foot frame
   */
  const Eigen::Vector3d & Puk_max() const noexcept
  {
    return P_u_k_max;
  }

  const Eigen::Matrix3d & Support_ori() const noexcept
  {
    return R_support_0;
  }

  const Eigen::Vector3d & Disturbance() const noexcept
  {
    return w_k;
  }

  /**
   * @brief Set a disturbance over a duration
   *
   * @param w
   * @param kappa
   * @param d
   */
  void Disturbance(const Eigen::Vector3d & w, const double kappa = 1, const double d = 1e3) noexcept
  {
    w_k = w;
    m_kappa = kappa;
    perturbation_duration = d;
  }

  /**
   * @brief Set disturbance over an infinite duration
   * If you wish to combine two perturbation this one should take into account the temporary disturbance
   *
   * @param w
   * @param kappa
   */
  void InfiniteDisturbance(const Eigen::Vector3d & w, const double kappa = 1) noexcept
  {
    w_k_inf = w;
    m_kappa_inf = kappa;
  }

  /**
   * @brief Set the amplitude/frequency/phase/offset of an RL-driven CoM
   * height sine reference, consumed by init_MPC() (CoMHeightTestSignal::RlSine)
   * to build CoM_height[] and the analytic zc_ddot feedforward over the
   * control horizon.
   *
   * Caller (mc_mjlab, via the ismpc_walking_python bridge) is responsible for
   * ensuring amplitude <= offset, so the resulting height trajectory
   * (offset + amplitude*sin(...)) never goes negative; this setter does not
   * clamp or validate the values it is given.
   */
  void SetCoMHeightSineParams(double offset, double frequency, double sin_amp, double cos_amp) noexcept
  {
    m_rl_com_z_offset = offset;
    m_rl_com_z_frequency = frequency;
    m_rl_com_z_sin_amp = sin_amp;
    m_rl_com_z_cos_amp = cos_amp;
  }


  double eta()
  {
    return m_eta[0];
  }

  /**
   * Returns the per-horizon-step pendulum frequency vector (Smaldone-style time-varying eta).
   * eta()[0] is equivalent to the legacy scalar eta() accessor above.
   */
  const std::vector<double> & eta_vec() const noexcept
  {
    return m_eta;
  }

  /**
   * Returns Omega(t0), the solution at the start of the horizon of the scalar Riccati equation
   * \dot{Omega} = Omega^2 - a(t), a(t) = eta(t)^2, integrated backward from the tail.
   * This is NOT the same as eta()! eta(t0) = sqrt(a(t0)) is only the instantaneous frequency;
   * Omega(t0) is the value that makes xi = p_c + p_c_dot/Omega exactly decouple the time-varying
   * pendulum dynamics (see Stability_Constraints()). They coincide only when a(t) is constant.
   */
  double Omega_0() const noexcept
  {
    return m_Omega.empty() ? m_eta[0] : m_Omega[0];
  }

  /**
   * Returns the full fine-grid Omega(t) profile computed by the last call to
   * Compute_Riccati_Kernel(), sampled at spacing delta/riccati_substeps() over [t0, t0+Tc],
   * plus the tail node. Exposed mainly for logging/debugging.
   */
  const std::vector<double> & Omega_vec() const noexcept
  {
    return m_Omega;
  }

  /**
   * Number of Riccati/kernel integration sub-steps per MPC sample (m_delta).
   * The Riccati equation and the K/G/H_k quadratures are evaluated on a fine grid of spacing
   * m_delta / riccati_substeps(). Larger values improve the accuracy of the backward Riccati
   * integration and of the kernel quadratures at the cost of more computation per QP iteration;
   * tune this if real-time cost becomes an issue.
   */
  int riccati_substeps() const noexcept
  {
    return m_riccati_substeps;
  }

  void riccati_substeps(int n)
  {
    m_riccati_substeps = std::max(1, n);
  }

  /**
   * Returns the per-horizon-step CoM height trajectory z_c(t) computed by the solver
   * (e.g. the hand-crafted sigmoid/sinusoid profile, later to be replaced by NN output).
   */
  const std::vector<double> & CoM_height_vec() const noexcept
  {
    return CoM_height;
  }

  /**
   * Returns the analytic feedforward CoM-height VELOCITY z_c_dot(t) accompanying
   * CoM_height_vec(), for signal cases that have a genuine closed-form derivative
   * (Sine, RlSine). Empty for signal cases that don't populate it (Step; and
   * PerStepCosine currently -- see its case block's comment). Callers (e.g.
   * Walking_controller::MoveCoM()) must check .empty()/.size() before indexing,
   * exactly as they must already do for CoM_height_vec() at cold start.
   */
  const std::vector<double> & CoM_height_vel_vec() const noexcept
  {
    return CoM_height_vel;
  }

  /**
   * Returns the analytic feedforward CoM-height ACCELERATION z_c_ddot(t)
   * accompanying CoM_height_vec() -- the SAME value already computed and
   * consumed internally as zc_ddot to build m_eta[i] (see the RlSine/Sine
   * case blocks), now also retained here so the whole-body controller can
   * use it as a feedforward term instead of it being discarded after the
   * eta computation. Same emptiness/sizing caveats as CoM_height_vel_vec().
   */
  const std::vector<double> & CoM_height_acc_vec() const noexcept
  {
    return CoM_height_acc;
  }

  const std::vector<double> & CoM_height_fine_vec() const noexcept { return CoM_height_fine; }
  const std::vector<double> & CoM_height_vel_fine_vec() const noexcept { return CoM_height_vel_fine; }
  const std::vector<double> & CoM_height_acc_fine_vec() const noexcept { return CoM_height_acc_fine; }

  /**
   * Returns the initial DCM used in the MPC in the world frame
   */
  const Eigen::Vector3d & Puk() const noexcept
  {
    return P_u_k;
  }

  const Eigen::Vector3d & Uk()
  {
    return U_k;
  }

  const Eigen::Vector2d & stability_error() const noexcept
  {
    return stab_error;
  }

  const std::vector<Eigen::Vector3d> QP_zmp()
  {
    std::vector<Eigen::Vector3d> out;
    for(Eigen::Index i = 0; i < m_QP_zmp.size() / 2; i++)
    {
      out.push_back(P_z_k_delayed + Eigen::Vector3d{m_QP_zmp(2 * i), m_QP_zmp(2 * i + 1), 0});
    }
    return out;
  }

  const std::vector<Eigen::Vector3d> QP_dcm()
  {
    std::vector<Eigen::Vector3d> out;
    for(Eigen::Index i = 0; i < m_QP_dcm.size() / 2; i++)
    {
      out.push_back(Eigen::Vector3d{m_QP_dcm(2 * i), m_QP_dcm(2 * i + 1), CoM_height[i]});
    }
    return out;
  }

  /**
   * Set an initial DCM in the world frame
   */
  void Puk(Eigen::Vector3d puk)
  {
    P_u_k = puk;
    P_u_k.z() = 0;
  }

  const Eigen::VectorXd & GetAfterTc_ZMP_trajectory()
  {
    return AfterTc_ZMP_trajectory;
  }

  const std::vector<Eigen::Vector2d> & dcmRefTrajectory()
  {
    return dcm_ref_traj;
  }

  const Eigen::VectorXd & ZMP_vel() const noexcept
  {
    return m_ZMP_u;
  }

  const Eigen::VectorXd & Lc_dot()
  {
    return m_Ldot_c;
  }

  const Eigen::Vector3d & Initial_ZMP() const noexcept
  {
    return P_z_k;
  }

  const Eigen::Vector3d & Delayed_ZMP() const noexcept
  {
    return P_z_k_delayed;
  }

  double get_lambda()
  {
    return m_lambda;
  }

  double support_state()
  {
    return m_support_state;
  }

  Eigen::Vector3d zmp_ref()
  {
    return m_ref_zmp;
  }

  void set_lambda(const double in)
  {
    m_lambda = in;
  }

  double zmp_delay()
  {
    return m_delay;
  }

  void zmp_delay(const double t)
  {
    m_delay = std::max(0., std::min(m_delta, t));
  }

  std::vector<Eigen::Vector3d> feasibility_region()
  {
    if(m_stop)
    {
      return m_feasibility_standing_region.Get_Polygone_Corners();
    }
    else
    {
      if(m_feasibility_region.size() != 0)
      {
        return m_feasibility_region;
      }
      Eigen::Vector3d p0 = Puk_min();
      Eigen::Vector3d p2 = Puk_max();
      Eigen::Vector3d p1 = p0 + R_support_0 * Eigen::Vector3d{(R_0_support * (Puk_max() - Puk_min())).x(), 0, 0};
      Eigen::Vector3d p3 = p0 + R_support_0 * Eigen::Vector3d{0, (R_0_support * (Puk_max() - Puk_min())).y(), 0};
      return {p0, p1, p2, p3};
    }
  }

  const SupportPolygon & feasibility_region_switched()
  {
    return m_feasibility_standing_region_swing;
  }

  SupportPolygon & standing_feasibility_polygone()
  {
    return m_feasibility_standing_region;
  }

  const std::vector<Eigen::Vector3d> & get_polynome_support()
  {

    return SuppPolyCorners;
  }

  void configure(const ControllerConfiguration & config);

  void Allow_none(bool state)
  {
    Allow_None = state;
  }

  std::vector<Eigen::Vector3d> zmp_ref_traj()
  {
    std::vector<Eigen::Vector3d> Output;
    int n = static_cast<int>(b_zmp_traj.size() / 2);
    for(int i = 0; i < n; i++)
    {
      Output.push_back(Eigen::Vector3d{b_zmp_traj(2 * i), b_zmp_traj(2 * i + 1), 0} + P_z_k_delayed);
    }
    return Output;
  }
  const std::vector<Eigen::Vector3d> & admittance_references()
  {
    return m_admittance_targets;
  }

  bool AutoFootstepPlacement = false;
  bool UsePendulumSolver = false;
  bool UseAngularMomentumDot = false;

  std::vector<std::vector<Eigen::Vector3d>> All_poly;

  const std::vector<std::vector<Eigen::Vector3d>> & get_allpolys()
  {

    return All_poly;
  }

  bool stop()
  {
    return m_stop;
  }
  bool ComputeTrajectory = true;

private:
  /**
   * Generate the ZMP Trajectory constraints for locomotion :
   * ZMP constraints then includes the steps location decision variables
   *
   * The ZMP reference pose is also generated here
   */
  void ZMP_Constraints();

  void ZMP_Transition_Constraint(Eigen::MatrixXd & A_out, Eigen::VectorXd & b_out, SupportPolygon PolySS);

  /**
   * @brief ZMP Constraint in standing mode
   * The cstr is here the current support polygon on the entire horizon
   *
   * The ZMP reference pose is also generated here
   */
  void Static_ZMP_Constraints();

  void Compute_Stability_Range();

  void Compute_Standing_Stability_Range();

  /**
   * Footsteps kinematics cstr
   */
  void FootSteps_Constraints();

  /**
   * Stability constraints are the QP equality constraints, first line is the X axis, second is the Y axis
   */
  void Stability_Constraints();

  /**
   * @brief
   *  Compute the stability condition such as x_u_star = A * x + b
   *
   * @param A_out
   * @param b_out
   * @param eta
   * @param indx_start
   */
  void Stability_Condition(Eigen::MatrixXd & A_out,
                           Eigen::VectorXd & b_out,
                           const double eta,
                           const int indx_start,
                           const Eigen::Vector2d w);
  /**
   * @brief Convert the created contraints to a Eigen matrix and vector usable for the QP solver
   *
   * @param A_out
   * @param b_out
   * @param A_in
   * @param b_in
   */
  void create_cstr_matrices(Eigen::MatrixXd & A_out,
                            Eigen::VectorXd & b_out,
                            std::vector<SupportPolygon> & A_in,
                            const std::vector<Eigen::VectorXd> & b_in);

  /**
   * @brief Convert the created contraints to a Eigen matrix and vector usable for the QP solver
   *
   * @param A_out
   * @param b_out
   * @param A_in
   * @param b_in
   */
  void create_cstr_matrices(Eigen::MatrixXd & A_out,
                            Eigen::VectorXd & b_out,
                            std::vector<Eigen::MatrixX2d> & A_in,
                            const std::vector<Eigen::VectorXd> & b_in);

  /**
   * @brief Compute the dcm after the delayed reference have been applied
   *
   * @return Eigen::Vector2d
   */
  Eigen::Vector2d compute_dcm_delay();

  /**
   * @brief Compute A and b such as for tj = j * m_delta ; dcm_j = A * x + b
   * where x the decision variables
   *
   * @param A_out
   * @param b_out
   * @param dcm_delay
   * @param indx
   */
  void compute_dcm(Eigen::MatrixXd & A_out, Eigen::Vector2d & b_out, const Eigen::Vector2d & dcm_delay, const int indx);

  /**
   * @brief Create a dcm cost function such as
   * dcm = M_dcm * x + b_dcm
   * and generate the ref traj vector such as
   * dcm_traj = M_traj * x + b_traj
   *
   * where x is the decision variables
   *
   * @param M_dcm Matrix to compute dcm
   * @param b_dcm Vector to compute dcm
   * @param M_traj_dcm Matrix to compute dcm ref traj
   * @param b_traj_dcm Vector to compute dcm ref traj
   * @param M_traj_zmp Matrix to compute zmp ref traj
   * @param b_traj_zmp Vector to compute zmp ref traj
   */
  void create_dcm_cost_function(Eigen::MatrixXd & M_dcm,
                                Eigen::VectorXd & b_dcm,
                                Eigen::MatrixXd & M_traj_dcm,
                                Eigen::VectorXd & b_traj_dcm,
                                Eigen::MatrixXd & M_traj_zmp,
                                Eigen::VectorXd & b_traj_zmp);

  /**
   * @brief Create a 2*m_C square matrix A to retrieve the zmp location from the decision variable zmp references
   */
  Eigen::MatrixXd create_zmp_matrix(bool addDelay);
  Eigen::MatrixXd create_u_matrix();

  void Compute_Integration_Matrix(const std::vector<double> & eta);

  /**
   * @brief Backward-integrate the scalar Riccati equation \dot{Omega} = Omega^2 - a(t),
   * a(t) = eta(t)^2, on a fine grid of spacing m_delta / m_riccati_substeps over
   * [t0, t0 + m_Tc], then extend to a constant-height tail node.
   *
   * Fills m_riccati_grid_dt (uniform fine spacing), m_Omega, m_beta = a/Omega, and the
   * cumulative divergence m_B_cum(i) = integral_{t0}^{t_i} beta. All four are sized
   * N_fine + 1, where N_fine = m_C * m_riccati_substeps, with index N_fine corresponding
   * to the tail node t0 + m_Tc.
   *
   * The tail (t > t0 + m_Tc) is assumed constant-height with a_inf = g / CoM_height_avg,
   * matching the terminal condition Omega(t_f) = sqrt(a_inf) (Prop. 8.1 of the checked
   * variable-height stability note). This backward solve must be redone whenever m_eta
   * (hence a(t)) changes, i.e. once per init_MPC() call, before Stability_Constraints().
   */
  void Compute_Riccati_Kernel();

  /**
   * @brief Given the Omega/beta/B/K profile from Compute_Riccati_Kernel(), compute the
   * closed-loop kernel G(t0,s) at every fine grid node via the O(N) backward recursion
   *   G_i = exp(-lambda*h_i)*G_{i+1} + lambda * (local closed-form integral of K*kappa over
   *   [t_i, t_{i+1}] against the exp(-lambda*(tau-t_i)) weight),
   * seeded at the tail node with the analytic closed-form
   *   G_N = lambda * kappa_inf * K_N / (sqrt(a_inf) + lambda),
   * then the flat cumulative integral S_i = integral_{t_i}^{inf} G(t0,tau) dtau via
   *   S_i = S_{i+1} + trapz(G_i, G_{i+1}), seeded at S_N = G_N / sqrt(a_inf).
   * H_k is then S evaluated (interpolated) at t_k + delta_d for each decision variable k.
   *
   * Also assembles b_free (the part of the boundedness condition coming from the already
   * decided/delayed ZMP on [t0, t0+delta_d] and the disturbance terms), per Eq. (5.9)-(5.10)
   * and Prop 7.1 of the checked note.
   *
   * @param H_k_out size-m_C vector, H_k_out(k) is the scalar coefficient of decision
   * variable u_k (applied identically on x and y) in the boundedness equality.
   * @param b_free_out the disturbance-and-delay-corrected free term of the boundedness
   * condition (2D, world/support frame consistent with P_u_k).
   */
  void Compute_Hk_And_bfree(Eigen::VectorXd & H_k_out, Eigen::Vector2d & b_free_out);

  /**
   * Integrate The ZMP velocity to compute the CoM, CoMd and ZMP trajectory
   */
  void Integrate();

  void Compute_Integration_Vector(const double eta,
                                  const Eigen::Vector2d & zmp0,
                                  const Eigen::Vector2d & zmpref,
                                  const double t0,
                                  const double tk);

  /**
   * Generate a ZMP trajectory that is the middle point of the zmp square constraints between the preview and control
   * horizon. Trajectory is computed in terms of ZMP velocity
   */
  void AntTailTrajectory();

  Eigen::VectorXd solveQP();

  Eigen::Vector3d P_z_k = Eigen::Vector3d::Zero(); // Initial ZMP position
  Eigen::Vector3d P_z_k_delayed = Eigen::Vector3d::Zero(); // ZMP pose after input U_k during input delay
  Eigen::Vector3d P_c_k = Eigen::Vector3d::Zero(); // Initial CoM Position
  Eigen::Vector3d V_c_k = Eigen::Vector3d::Zero(); // Initial CoM Velocity
  Eigen::Vector3d P_u_k = Eigen::Vector3d::Zero(); // Initial Unstable Component/DCM
  Eigen::Vector3d U_k = Eigen::Vector3d::Zero(); // Current admittance acting on the pendulum (z_0 + u_0)
  Eigen::Vector3d w_k = Eigen::Vector3d::Zero(); // Perturbance over a duration
  double m_kappa = 1;
  Eigen::Vector3d w_k_inf = Eigen::Vector3d::Zero(); // Perturbance over the infinty
  double m_kappa_inf = 1;
  Eigen::Vector3d Lc_k = Eigen::Vector3d::Zero(); // Initial Angular Momemtum
  double perturbation_duration = 0;

  Eigen::Matrix3d R_support_0 = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R_0_support = Eigen::Matrix3d::Identity();
  Eigen::VectorXd m_ZMP_u = Eigen::VectorXd::Zero(0); // Computed ZMP velocity in world frame
  Eigen::VectorXd m_Ldot_c = Eigen::VectorXd::Zero(0); // Computed Centroidal AngularMomemtum dot in world frame ori
  std::vector<double> m_timestamp; // Step TimesStamp Computed at the footStep Generation
  double NextOptimalTs = 10;

  sva::PTransformd X_0_support_foot = sva::PTransformd::Identity();
  sva::PTransformd X_0_swing_foot_initial = sva::PTransformd::Identity();
  sva::PTransformd X_0_swing_foot = sva::PTransformd::Identity();
  sva::PTransformd X_0_swing_foot_target = sva::PTransformd::Identity();

  std::vector<sva::PTransformd> input_steps_;
  std::vector<sva::PTransformd> corr_steps_;

  Eigen::Vector3d P_u_k_min = Eigen::Vector3d::Zero(); // Min initial DCM coordinates in support Foot Frame
  Eigen::Vector3d P_u_k_max = Eigen::Vector3d::Zero(); // Max initial DCM coordinates in support Foot Frame

  Eigen::VectorXd QP_Output;

  std::vector<Eigen::Vector3d> m_X_MPC; // Integrated CoM, CoMd, ZMP trajectory in world frame
  std::vector<Eigen::Vector3d> m_Y_MPC; // Integrated CoM, CoMd, ZMP trajectory in world frame

  double Ant_Tail_X = 0.0;
  double Ant_Tail_Y = 0.0;

  int N_Steps_Desired = -1;
  int N_Steps = 0;

  bool QPsuccess = false;
  bool m_feas_res = false;
  Eigen::Vector2d stab_error = Eigen::Vector2d::Zero();
  Eigen::VectorXd m_QP_zmp;
  Eigen::VectorXd m_QP_dcm;
  std::vector<Eigen::Vector3d> m_admittance_targets;
  bool Use_Stability_Task = false;
  bool Allow_None = true;
  bool InStabilityRange = false;
  bool DoubleSupport = true;
  bool m_stop = true;

  /**
   *Only during the first double support phase : If enabled, the admissible region is a sliding square,
   *otherwise it is a polygone defined by two  rectangle on both feets.
   */
  bool Slide_ZMP_region = false;

  // CoM_height[i] = m_rl_com_z_offset + m_rl_com_z_sin_amp * sin_phase + m_rl_com_z_cos_amp * cos_phase
  double m_rl_com_z_offset = 0.7;      // RL-set CoM height offset (m)
  double m_rl_com_z_frequency = 0.0;   // RL-set sine frequency (Hz)
  double m_rl_com_z_sin_amp = 0.0;   // RL-set sine amplitude
  double m_rl_com_z_cos_amp = 0.0;   // RL-set cosine amplitude

  std::vector<double> m_eta; // Prendulum frequency
  std::vector<double> m_eta_free; // Prendulum frequency disturbance free
  std::vector<double> CoM_height;
  // Analytic first/second time-derivatives of CoM_height[i], populated ONLY
  // for signal cases whose z(t) is smooth and has a genuine closed form
  // (Sine, RlSine; PerStepCosine reserved for a later pass -- see its case
  // block). Step's case is a true mathematical step and has neither. Sized
  // and filled in lockstep with CoM_height every solve; NOT filled/cleared
  // by cases that don't compute them, so callers must not assume staleness
  // is detectable from size alone across an active-signal-type change --
  // this codebase only ever runs one test_signal (compile-time constant),
  // so that situation does not currently arise in practice.
  std::vector<double> CoM_height_vel;
  std::vector<double> CoM_height_acc;
  // Fine-resolution (X_MPC/Y_MPC-matching, m_delta_control-spaced) CoM-height
  // reference, for smooth task-target consumption via MPC_state's accessors
  // (Get_CoM_planarTarget/Get_CoMVel_planarTarget/Get_CoMHeightAccel_target).
  // Populated ADDITIONALLY to (not instead of) the coarse CoM_height/vel/acc
  // above -- CoM_height (coarse) still exclusively feeds m_eta/Integrate()/
  // Compute_Riccati_Kernel(), completely unchanged by this addition. Do not
  // read these _fine members from Integrate() or anything feeding m_eta.
  std::vector<double> CoM_height_fine;
  std::vector<double> CoM_height_vel_fine;
  std::vector<double> CoM_height_acc_fine;
  double CoM_height_avg = 0.80;

  // --- Variable-height Riccati stability kernel (Compute_Riccati_Kernel / Compute_Hk_And_bfree) ---
  // All sized N_fine+1 with N_fine = m_C * m_riccati_substeps; index N_fine is the tail node at t0+Tc.
  int m_riccati_substeps = 10; // Tunable: fine-grid substeps per m_delta for the Riccati/kernel integration
  double m_riccati_dt = 0.0; // Fine grid spacing = m_delta / m_riccati_substeps, cached by Compute_Riccati_Kernel
  std::vector<double> m_Omega; // Omega(t) solving \dot{Omega} = Omega^2 - a(t), backward-integrated from the tail
  std::vector<double> m_beta; // beta(t) = a(t) / Omega(t)
  std::vector<double> m_B_cum; // B(t0,t) = integral_{t0}^{t} beta, cumulative from index 0
  std::vector<double> m_K_kernel; // K(t0,t) = beta(t) * exp(-B(t0,t))
  std::vector<double> m_G_kernel; // Closed-loop kernel G(t0,s), backward recursion, tail-seeded
  std::vector<double> m_S_cum; // S(t0,s) = integral_s^inf G(t0,tau) dtau, backward cumulative, tail-seeded
  double m_com_z_amplitude = 0.10; // Amplitude of the CoM height oscillation (metres)
  double m_com_z_test_t0 = 15.0;      // Step-test trigger time (s), wall-clock, replaces old hardcoded 15.0
  double m_com_z_test_period = 1.0;   // Sine-test period (s), wall-clock
  // Elapsed time since the start of the current step cycle, used to compute
  // the phase-based CoM height profile. Reset to 0 at each step switch.
  double m_tk_within_step = 0.0;
  double m_mass = 40.;
  double g = 9.8; // Gravity acceleration
  double m_tk = 0; // Represent the initial time in the MPC horizon
  double m_t_global = 0; // Global time of the control scheme
  double m_Tc = 2;
  double m_Tp = 5; // Control & Preview horizon time
  double m_Tds = 0.24; // Double Support Duration
  double m_input_Tds = 0;
  int Tds_offset = 0;
  double m_Dstep_ratio = 0.3; // T_DoubleStep/T_Step
  double m_delta = 0.05; // t_k - t_k-1
  double m_delta_control = 0.005; // Controller timestep
  double N_integration = 1;
  double m_dx_static = 0.1;
  double m_dy_static = 0.1;
  double m_dx = 0.1;
  double m_dy = 0.1; // ZMP square size at one timestep
  double m_dx_u = 0.1;
  double m_dy_u = 0.1; // ZMP square size at one timestep
  double m_foot_max_vel = 2.;
  Eigen::Vector2d m_ts_range{0.6, 2};
  Eigen::Vector2d m_tds_range{0.2, 1.5};
  Eigen::Vector2d m_tss_range{0.4, 1.5};

  Eigen::Vector2d rect_pose_offset = Eigen::Vector2d::Zero(); // cstr zone offset in the foot frame for y axis, positive
                                                              // offset is an offset toward the other feet;
  Eigen::Vector2d rect_pose_offset_static =
      Eigen::Vector2d::Zero(); // cstr zone offset in the foot frame for y axis, positive offset is an offset
                               // toward the other feet;

  Eigen::Vector2d zmp_ref_offset = Eigen::Vector2d::Zero();
  Eigen::Vector2d zmp_ref_offset_end_step =
      Eigen::Vector2d::Zero(); // adds to the zmp_ref_offset and applied in sg supp, x sign depends on step target
  Eigen::Vector2d zmp_ref_offset_start_step =
      Eigen::Vector2d::Zero(); // adds to the zmp_ref_offset and applied in sg supp, x sign depends on step target

  double zmp_cstr_next_stp_ratio = 2;
  double m_dx_f = 0.1;
  double m_dy_f = 0.1; // Step kinematic admissible Region
  double m_dx_f_rect = 0.1;
  double m_dy_f_rect = 0.1; // Step admissible region
  double m_Ld_max = 10;
  double m_Beta_zmp_vel = 1;
  double m_Beta_step = 1e1;
  double m_Beta_stab = 1e5;
  double m_Beta_zmp_traj = 0.;
  double m_Beta_zmp_traj_stop = 0;
  double m_Beta_Lc = 1e-2;
  double m_Beta_dcm = 1e2;
  double m_Beta_dcm_stop = 1000;
  double m_Beta_dcm_vel = 0;
  double m_Beta_dcm_vel_stop = 1000;
  double m_lambda = 25;
  double m_delay = 0.02; // delay ( < m_delta ) during which zmp is under previous input Uk
  double m_delay_elapsed = 0; // Between 0 and m_delay represent the remaining time the delay must be applied
  double m_t_delay = 0; // represent when the delay has been applied
  double m_t_lift = 0; // time when the foot contact has been released

  double m_feet_distance = 0.2;
  std::string m_support_foot = "LeftFoot";
  int j_Max_C = 0; // Number of footsteps in the Control Horizon
  int j_f = 0; // Index of the actual support foot
  int j_fm1 = 0; // Index of the previous support foot
  double m_support_state = 0;
  Eigen::Vector3d m_ref_zmp = Eigen::Vector3d::Zero(); // first ref zmp in the horizon
  int kfoot = 0;

  std::string m_Tail; // Velocity Tailing desired
  std::string m_Tail_save; // Save of the desired Velocity Tailing

  Eigen::VectorXd AfterTc_ZMP_velocity; // velocity generated by the midpoint between the ZMP constraints after Tc
  Eigen::VectorXd AfterTc_ZMP_trajectory;

  std::vector<double> ZMP_ref_traj;
  std::vector<Eigen::Vector3d> ZMP_min_ref_traj;
  std::vector<Eigen::Vector3d> ZMP_max_ref_traj;
  Eigen::MatrixXd M_zmp_traj;
  Eigen::VectorXd b_zmp_traj;

  std::vector<Eigen::Vector2d> dcm_ref_traj;

  // CoM,CoMd,ZMP Integration
  Eigen::Matrix2d Integration_Mat;
  Eigen::Vector2d Integration_Vec_x;
  Eigen::Vector2d Integration_Vec_y;

  // Pendulum dynamic to integrate state : \dot{s} = A * s + B * U
  Eigen::Matrix3d m_dynamic_matrix_A;
  Eigen::MatrixXd m_dynamic_matrix_B;

  Eigen::VectorXd Pzk_Offset; // Vector that represent the intial position of the ZMP
  Eigen::MatrixXd C; // Temporary matrix to compute ZMP constraints with footsteps placements
  // Eigen::MatrixXd Delta; //Matrix to derive the ZMP position to ZMP velocity

  int m_C; // Number of indexs in the Control time length Tc = m_C * m_delta
  int m_P; // Number of indexs in the Preview time length Tp = m_P * m_delta
  int m_D; // Number of Iteration on the double steps period
  double
      count_Dstep; // Number bounded between 1 and m_D describing the position of the zone during the doubleStep timing

  std::vector<SupportPolygon> zmp_cstr_polygons;

  SupportPolygon m_double_support_polygon;
  SupportPolygon m_feasibility_standing_region;

  std::vector<Eigen::Vector3d> m_feasibility_region;

  SupportPolygon m_feasibility_standing_region_swing; // standing feasibility region if support foot is switched

  std::vector<Eigen::Vector3d> SuppPolyCorners;

  Eigen::MatrixXd Aineq_zmp; // Inequality ZMP Matrix
  Eigen::VectorXd bineq_zmp; // Inequality ZMP Vector

  Eigen::MatrixXd Aineq_Ld; // Inequality ZMP Matrix
  Eigen::VectorXd bineq_Ld; // Inequality ZMP Vector

  Eigen::MatrixXd A_stab; // Equality stability cstr matrix
  Eigen::VectorXd b_stab; // Equality stability cstr vector

  Eigen::MatrixXd Aineq_steps; // Inequality Steps Matrix
  Eigen::VectorXd bineq_steps; // Inequality Steps Vector

  Eigen::MatrixXd A_zmp; // Matrix that computes the zmp from the QP output;

  // QP Problem
  Eigen::QuadProgDense QP;

  int N_variable;
  Eigen::MatrixXd m_Q; // QP Hessian
  Eigen::VectorXd m_p; // QP Grad
  Eigen::MatrixXd m_G; // QP constraints Matrix

  Eigen::MatrixXd Aeq; // Equality Matrix
  Eigen::VectorXd beq; // Equality Vector

  Eigen::MatrixXd Aineq; // Inequality Matrix
  Eigen::VectorXd bineq; // Inequality Vector

  feasibility_solver m_feasibilitySolver;
};