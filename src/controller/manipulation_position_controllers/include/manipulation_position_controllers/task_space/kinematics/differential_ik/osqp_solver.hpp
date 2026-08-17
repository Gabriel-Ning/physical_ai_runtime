// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// QP-based differential IK solver using OSQP via OsqpEigen.
//
// Formulation (per update), aligned with cyclo_control's QPBase:
//
//   Decision variables: x = [dq; s_min; s_max]  (3n)
//     dq     — joint position increment [rad],  n = n_joints
//     s_min  — slack for lower CBF constraints
//     s_max  — slack for upper CBF constraints
//
//   Cost:
//     min   0.5 · x'·P·x  +  q'·x
//     P = blkdiag( 2·J'·J + 2·λ²I,  ρ·I,  ρ·I )
//     q = [ -2·J'·v_desired;  0;  0 ]
//
//     v_desired = K · error   (gain applied to error — same as DLS)
//     ρ = slack penalty (large — slack is used only when necessary)
//
//   Constraints (inequality only, m = 6n):
//     For each joint i ∈ [0, n):
//       Row 0  dq_i          ≥  -v_max·dt                   (velocity lower)
//       Row 1  dq_i          ≤  +v_max·dt                   (velocity upper)
//       Row 2  dq_i + s_min_i ≥  -α·(q_i - q_min_i)          (CBF lower, soft)
//       Row 3  dq_i - s_max_i ≤  +α·(q_max_i - q_i)          (CBF upper, soft)
//       Row 4  s_min_i       ≥  0                            (slack ≥ 0)
//       Row 5  s_max_i       ≥  0                            (slack ≥ 0)
//
//   CBF parameter α controls how aggressively the limit approach is
//   constrained (higher = tighter approach to limits).
//
// Comparing with our original box-constraint formulation:
//   - velocity limits: same (hard)
//   - joint position limits: changed from hard box to CBF inequality + slack,
//     guaranteeing QP feasibility near limits.  The slack absorbs residual
//     infeasibility instead of triggering `lb > ub → hold`.
//
// Cartesian velocity clamp (linear + angular) is applied to v_desired
// before the QP cost is formed, matching PinocchioDlsSolver behaviour.

#pragma once

#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/Solver.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "manipulation_position_controllers/task_space/kinematics/differential_ik/solver_interface.hpp"

namespace manipulation_position_controllers::task_space
{

/// OSQP-based QP differential IK solver (OsqpEigen backend).
///
/// Uses Pinocchio for FK + Jacobian, OsqpEigen for the QP.
/// Cost and constraint formulation aligned with cyclo_control's QPBase.
class OsqpIkSolver : public DifferentialIkSolver
{
public:
  OsqpIkSolver() = default;
  ~OsqpIkSolver() override = default;

  bool configure(
    const std::string & urdf_xml,
    const std::vector<std::string> & joint_names,
    const std::string & base_frame,
    const std::string & tip_frame,
    const SolverConfig & config) override
  {
    config_ = config;
    joint_names_ = joint_names;
    base_frame_ = base_frame;
    tip_frame_ = tip_frame;
    n_joints_ = joint_names.size();

    // Build Pinocchio model.
    model_ = std::make_unique<pinocchio::Model>();
    try {
      pinocchio::urdf::buildModelFromXML(urdf_xml, *model_);
    } catch (const std::exception & e) {
      last_error_ = std::string("Pinocchio URDF build failed: ") + e.what();
      return false;
    }
    data_ = std::make_unique<pinocchio::Data>(*model_);

    if (!model_->existFrame(tip_frame_)) {
      last_error_ = "Frame '" + tip_frame_ + "' not found in URDF.";
      return false;
    }
    tip_frame_id_ = model_->getFrameId(tip_frame_);

    // Base frame: all task-space poses are expressed relative to it. Identity
    // transform when base_frame is the URDF root.
    if (!model_->existFrame(base_frame_)) {
      last_error_ = "Base frame '" + base_frame_ + "' not found in URDF.";
      return false;
    }
    base_frame_id_ = model_->getFrameId(base_frame_);

    if (model_->nq <= 0 || model_->nv <= 0 ||
        model_->nq > 256 || model_->nv > 256) {
      last_error_ = "Invalid model: nq=" + std::to_string(model_->nq) +
        " nv=" + std::to_string(model_->nv);
      return false;
    }
    if (model_->nv < static_cast<int>(n_joints_)) {
      last_error_ = "nv < n_joints";
      return false;
    }

    // Joint-index mapping.
    joint_q_index_.resize(n_joints_, -1);
    joint_v_index_.resize(n_joints_, -1);
    for (size_t i = 0; i < n_joints_; ++i) {
      const auto & name = joint_names_[i];
      if (model_->existJointName(name)) {
        pinocchio::JointIndex jid = model_->getJointId(name);
        joint_q_index_[i] = static_cast<int>(model_->joints[jid].idx_q());
        joint_v_index_[i] = static_cast<int>(model_->joints[jid].idx_v());
      } else {
        last_error_ = "Joint '" + name + "' not found in URDF.";
        return false;
      }
    }

    // --- Pre-allocate working buffers ---
    const int n = static_cast<int>(n_joints_);
    const int n_vars = 3 * n;   // [dq; s_min; s_max]
    const int n_constr = 6 * n; // inequality constraints

    q_.resize(model_->nq);
    q_.setZero();
    neutral_q_ = pinocchio::neutral(*model_);
    q_neutral_.resize(model_->nq);
    q_neutral_.setZero();
    task_error_.resize(6);
    task_error_.setZero();
    jacobian_.resize(6, model_->nv);
    jacobian_.setZero();
    J_act_.resize(6, n);
    J_act_.setZero();
    P_dq_dense_.resize(n, n);
    P_dq_dense_.setZero();
    q_vec_.resize(n);
    q_vec_.setZero();

    // OSQP vectors
    gradient_.resize(n_vars);
    gradient_.setZero();
    lower_bound_.resize(n_constr);
    lower_bound_.setConstant(-OsqpEigen::INFTY);
    upper_bound_.resize(n_constr);
    upper_bound_.setConstant(OsqpEigen::INFTY);
    solution_.resize(n_vars);
    solution_.setZero();

    // Build A matrix sparsity pattern (constant — only bounds change per step).
    build_A_matrix(n);

    // Build P matrix sparsity pattern and set constant slack penalty.
    build_P_matrix(n);

    // Setup OSQP solver via OsqpEigen.
    solver_ = std::make_unique<OsqpEigen::Solver>();
    solver_->settings()->resetDefaultSettings();
    solver_->settings()->setVerbosity(false);
    solver_->settings()->setWarmStart(true);
    solver_->settings()->setAbsoluteTolerance(config_.osqp_abs_tolerance);
    solver_->settings()->setRelativeTolerance(config_.osqp_rel_tolerance);
    solver_->settings()->setAdaptiveRho(false);
    solver_->settings()->setRho(config_.osqp_rho);
    solver_->settings()->setMaxIteration(config_.osqp_max_admm_iterations);
    solver_->settings()->setPolish(false);

    solver_->data()->setNumberOfVariables(n_vars);
    solver_->data()->setNumberOfConstraints(n_constr);

    if (!solver_->data()->setHessianMatrix(P_sparse_)) {
      last_error_ = "Failed to set OSQP Hessian matrix.";
      return false;
    }
    if (!solver_->data()->setGradient(gradient_)) {
      last_error_ = "Failed to set OSQP gradient.";
      return false;
    }
    if (!solver_->data()->setLinearConstraintsMatrix(A_sparse_)) {
      last_error_ = "Failed to set OSQP linear constraints matrix.";
      return false;
    }
    if (!solver_->data()->setBounds(lower_bound_, upper_bound_)) {
      last_error_ = "Failed to set OSQP bounds.";
      return false;
    }

    if (!solver_->initSolver()) {
      last_error_ = "OsqpEigen initSolver() failed.";
      return false;
    }

    last_error_.clear();
    allocate_solve_result(result_, n_joints_);
    configured_ = true;
    return true;
  }

  void reset(const std::vector<double> & q_current) override
  {
    write_q(q_current);
    q_neutral_ = q_;
    solution_.setZero();
    gradient_.setZero();
    lower_bound_.setConstant(-OsqpEigen::INFTY);
    upper_bound_.setConstant(OsqpEigen::INFTY);
  }

  bool compute_tip_pose(
    const std::vector<double> & q_current,
    std::array<double, 7> & pose) override
  {
    if (!configured_) {
      return false;
    }

    write_q(q_current);
    pinocchio::forwardKinematics(*model_, *data_, q_);
    pinocchio::updateFramePlacements(*model_, *data_);

    // Tip pose expressed in base frame (bMt = oMb⁻¹ · oMt).
    const pinocchio::SE3 bMt =
      data_->oMf[base_frame_id_].inverse() * data_->oMf[tip_frame_id_];
    const Eigen::Quaterniond quat(bMt.rotation());
    pose[0] = bMt.translation().x();
    pose[1] = bMt.translation().y();
    pose[2] = bMt.translation().z();
    pose[3] = quat.x();
    pose[4] = quat.y();
    pose[5] = quat.z();
    pose[6] = quat.w();
    return true;
  }

  const SolveResult & solve(
    const std::vector<double> & q_current,
    const std::array<double, 7> & pose_target,
    double period) override
  {
    reset_solve_result(result_);
    auto & result = result_;

    if (!configured_ || !solver_) {
      result.status_code = -1;
      return result;
    }

    const int n = static_cast<int>(n_joints_);
    const double dt = std::max(period, kMinSolverDtS);

    // 1. FK from q_current.
    write_q(q_current);
    pinocchio::forwardKinematics(*model_, *data_, q_);
    pinocchio::updateFramePlacements(*model_, *data_);

    // 2. Task-space error.
    const auto & tip_M = data_->oMf[tip_frame_id_];
    const Eigen::Vector3d current_pos = tip_M.translation();
    const Eigen::Quaterniond current_quat(tip_M.rotation());

    // Lift the base-frame target into the model-root (world) frame to match
    // the world-aligned Jacobian below. No-op when base_frame is the root.
    Eigen::Quaterniond target_quat_base(
      pose_target[6], pose_target[3], pose_target[4], pose_target[5]);
    target_quat_base.normalize();
    const pinocchio::SE3 world_T_target = data_->oMf[base_frame_id_] *
      pinocchio::SE3(
        target_quat_base.toRotationMatrix(),
        Eigen::Vector3d(pose_target[0], pose_target[1], pose_target[2]));
    const Eigen::Vector3d target_pos = world_T_target.translation();
    Eigen::Quaterniond target_quat(world_T_target.rotation());

    task_error_.head<3>() = target_pos - current_pos;

    Eigen::Quaterniond q_error = target_quat * current_quat.conjugate();
    if (q_error.w() < 0.0) { q_error.coeffs() *= -1.0; }
    task_error_.tail<3>() = 2.0 * q_error.vec();

    result.task_position_error_norm = task_error_.head<3>().norm();
    result.task_orientation_error_norm = task_error_.tail<3>().norm();

    // 3. Desired Cartesian twist (gain applied to error — same as DLS).
    Eigen::Matrix<double, 6, 1> v_desired;
    v_desired.head<3>() = config_.position_gain * task_error_.head<3>();
    v_desired.tail<3>() = config_.orientation_gain * task_error_.tail<3>();

    // Cartesian velocity clamp (matches PinocchioDlsSolver behaviour).
    const double linear_speed = v_desired.head<3>().norm();
    if (linear_speed > config_.max_linear_velocity_m_s && linear_speed > 1e-12) {
      v_desired.head<3>() *= config_.max_linear_velocity_m_s / linear_speed;
    }
    const double angular_speed = v_desired.tail<3>().norm();
    if (angular_speed > config_.max_angular_velocity_rad_s && angular_speed > 1e-12) {
      v_desired.tail<3>() *= config_.max_angular_velocity_rad_s / angular_speed;
    }

    // 4. Frame Jacobian → actuated-joint Jacobian.
    pinocchio::computeFrameJacobian(
      *model_, *data_, q_, tip_frame_id_,
      pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_);

    for (int j = 0; j < n; ++j) {
      J_act_.col(j) = jacobian_.col(joint_v_index_[j]);
    }

    // 5. QP cost. The decision variable dq is a position INCREMENT, so the
    //    task target must be the Cartesian increment over this timestep,
    //    v_desired·dt — mirroring DLS, which computes qdot then integrates
    //    dq = qdot·dt. Building the cost from v_desired directly (no dt) makes
    //    the QP solve for dq = J⁺·v_desired, i.e. a step 1/dt (≈500× at 500 Hz)
    //    larger than DLS for the same gain. That over-correction is clamped to
    //    v_max·dt every cycle, producing a sustained period-2 limit cycle of
    //    amplitude ≈ v_max·dt even with a stationary target. Folding dt in below
    //    aligns OSQP exactly with DLS:
    //      (JᵀJ + λ²I) dq = Jᵀ (v_desired·dt)   ⇒   dq = dt · qdot_DLS
    const double lambda_sq = config_.damping * config_.damping;
    P_dq_dense_.noalias() = 2.0 * J_act_.transpose() * J_act_;
    for (int i = 0; i < n; ++i) {
      P_dq_dense_(i, i) += 2.0 * lambda_sq;
    }
    q_vec_.noalias() = -2.0 * J_act_.transpose() * v_desired;

    // Optional posture task gradient. Here posture is a soft *competing*
    // secondary cost, not a null-space projection: the QP minimizes the sum of
    // the task term and this posture term. With posture_weight ≪ the (unit)
    // task weight it acts like the DLS null-space pull, but a sufficiently
    // large posture_weight WILL trade away primary-task accuracy. Keep it
    // small (default 1e-3).
    if (config_.posture_weight > 0.0) {
      for (int i = 0; i < n; ++i) {
        int qi = joint_q_index_[i];
        if (qi >= 0 && qi < static_cast<int>(model_->nq)) {
          q_vec_[i] += config_.posture_weight * (q_[qi] - q_neutral_[qi]);
        }
      }
    }

    // Integrate over the timestep: scaling the whole gradient by dt scales the
    // QP solution dq by dt (P unchanged), giving dq = dt·qdot and preserving
    // the task/posture balance. NOTE: OSQP gains now share DLS semantics —
    // use comparable position_gain/orientation_gain values (≈ DLS).
    q_vec_ *= dt;

    // 6. Update P matrix (only the upper-left n×n dq block changes).
    update_P_dq_block(n);

    // 7. Gradient: [q_dq; 0; 0].
    gradient_.head(n) = q_vec_;
    gradient_.segment(n, 2 * n).setZero();
    solver_->updateGradient(gradient_);

    // 8. Bounds: velocity limits + CBF constraints.
    const double v_max_dt = config_.max_joint_velocity_rad_s * dt;
    const double cbf_gain = config_.osqp_cbf_gain;

    for (int i = 0; i < n; ++i) {
      int qi = joint_q_index_[i];
      double q_i = (qi >= 0 && qi < static_cast<int>(model_->nq))
        ? q_[qi] : q_current[i];

      // User-provided joint limits (from controller parameter), fallback to
      // model limits.
      double q_lb = -OsqpEigen::INFTY, q_ub = OsqpEigen::INFTY;
      if (i < static_cast<int>(joint_limit_lower_.size())) {
        q_lb = joint_limit_lower_[i];
      }
      if (i < static_cast<int>(joint_limit_upper_.size())) {
        q_ub = joint_limit_upper_[i];
      }
      // "No limit" can arrive here as either true IEEE infinity (the
      // controller fills lower_limits_/upper_limits_ with
      // std::numeric_limits<double>::infinity() when the YAML parameter is
      // empty) or OSQP's own finite pseudo-infinity (q_lb/q_ub's default
      // initializer above, OsqpEigen::INFTY = OSQP_INFTY, a large *finite*
      // sentinel like 1e30 -- OSQP's C API has no representation for real
      // IEEE infinity in its bound arrays). std::isinf() alone would miss the
      // second case, so compare against OsqpEigen::INFTY directly: -inf <=
      // -OsqpEigen::INFTY and -OsqpEigen::INFTY <= -OsqpEigen::INFTY are both
      // true, so this one check covers both sentinel representations.
      if (q_lb <= -OsqpEigen::INFTY) {
        q_lb = model_->lowerPositionLimit[qi];
      }
      if (q_ub >= OsqpEigen::INFTY) {
        q_ub = model_->upperPositionLimit[qi];
      }

      const int row = 6 * i;

      // Velocity lower:  dq_i ≥ -v_max_dt
      lower_bound_[row + 0] = -v_max_dt;
      upper_bound_[row + 0] = OsqpEigen::INFTY;

      // Velocity upper:  dq_i ≤ +v_max_dt
      lower_bound_[row + 1] = -OsqpEigen::INFTY;
      upper_bound_[row + 1] = v_max_dt;

      // CBF lower (soft):  dq_i + s_min_i ≥ -cbf_gain · (q_i - q_lb)
      if (std::isfinite(q_lb)) {
        lower_bound_[row + 2] = -cbf_gain * (q_i - q_lb);
        upper_bound_[row + 2] = OsqpEigen::INFTY;
      } else {
        // No lower limit — constraint is effectively disabled.
        lower_bound_[row + 2] = -OsqpEigen::INFTY;
        upper_bound_[row + 2] = OsqpEigen::INFTY;
      }

      // CBF upper (soft):  dq_i - s_max_i ≤ +cbf_gain · (q_ub - q_i)
      if (std::isfinite(q_ub)) {
        lower_bound_[row + 3] = -OsqpEigen::INFTY;
        upper_bound_[row + 3] = cbf_gain * (q_ub - q_i);
      } else {
        lower_bound_[row + 3] = -OsqpEigen::INFTY;
        upper_bound_[row + 3] = OsqpEigen::INFTY;
      }

      // Slack non-negativity:  s_min_i ≥ 0
      lower_bound_[row + 4] = 0.0;
      upper_bound_[row + 4] = OsqpEigen::INFTY;

      // Slack non-negativity:  s_max_i ≥ 0
      lower_bound_[row + 5] = 0.0;
      upper_bound_[row + 5] = OsqpEigen::INFTY;
    }

    solver_->updateBounds(lower_bound_, upper_bound_);

    // 9. Solve.
    const auto exit_flag = solver_->solveProblem();
    const auto status = solver_->getStatus();

    // 10. Extract solution.
    if (status == OsqpEigen::Status::Solved ||
        status == OsqpEigen::Status::SolvedInaccurate) {
      solution_ = solver_->getSolution();
      for (size_t i = 0; i < n_joints_; ++i) {
        int qi = joint_q_index_[i];
        double q_i = (qi >= 0 && qi < static_cast<int>(model_->nq))
          ? q_[qi] : q_current[i];
        result.q_command[i] = q_i + solution_[i];
        result.joint_velocity_estimate[i] = solution_[i] / dt;
      }
      result.success = true;
    } else {
      // QP failed → hold current position.
      for (size_t i = 0; i < n_joints_; ++i) {
        result.q_command[i] = q_current[i];
      }
      result.success = false;
    }

    result.iterations = 1;
    result.status_code = static_cast<int>(exit_flag);

    return result;
  }

  const std::string & last_error() const override { return last_error_; }

  /// Set joint limits (called by controller after loading parameters).
  void set_joint_limits(
    const std::vector<double> & lower,
    const std::vector<double> & upper)
  {
    joint_limit_lower_ = lower;
    joint_limit_upper_ = upper;
  }

private:
  // OSQP ADMM iteration budget, CBF gain, and slack penalty are read from
  // config_ (osqp_max_admm_iterations / osqp_cbf_gain / osqp_slack_penalty)
  // -- see SolverConfig in solver_interface.hpp. They used to be compile-time
  // constants here with no YAML parameter path; CBF gain in particular is
  // robot- and joint-limit-margin-dependent and should not share one
  // hardcoded default across every robot.

  /// Write q_current into the Pinocchio q-vector (neutral + joint values).
  void write_q(const std::vector<double> & q_current)
  {
    q_ = neutral_q_;
    for (size_t i = 0; i < n_joints_; ++i) {
      int qi = joint_q_index_[i];
      if (qi >= 0 && qi < static_cast<int>(model_->nq)) {
        q_[qi] = q_current[i];
      }
    }
  }

  /// Build the inequality constraint matrix A (6n × 3n, sparse, constant
  /// sparsity pattern — only bounds change per solve).
  void build_A_matrix(int n)
  {
    const int n_vars = 3 * n;
    const int n_constr = 6 * n;

    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(8 * n);

    for (int i = 0; i < n; ++i) {
      const int row = 6 * i;

      // Row 0:  dq_i ≥ -v_max_dt  (velocity lower)
      triplets.emplace_back(row + 0, i, 1.0);

      // Row 1:  dq_i ≤ +v_max_dt  (velocity upper)
      triplets.emplace_back(row + 1, i, 1.0);

      // Row 2:  dq_i + s_min_i ≥ -α·(q_i - q_lb)  (CBF lower)
      triplets.emplace_back(row + 2, i, 1.0);
      triplets.emplace_back(row + 2, n + i, 1.0);

      // Row 3:  dq_i - s_max_i ≤ +α·(q_ub - q_i)  (CBF upper)
      triplets.emplace_back(row + 3, i, 1.0);
      triplets.emplace_back(row + 3, 2 * n + i, -1.0);

      // Row 4:  s_min_i ≥ 0  (slack non-negativity)
      triplets.emplace_back(row + 4, n + i, 1.0);

      // Row 5:  s_max_i ≥ 0  (slack non-negativity)
      triplets.emplace_back(row + 5, 2 * n + i, 1.0);
    }

    A_sparse_.resize(n_constr, n_vars);
    A_sparse_.setFromTriplets(triplets.begin(), triplets.end());
  }

  /// Build the P matrix sparsity pattern (3n × 3n, upper triangle).
  /// The upper-left n×n block (dq,dq) is dense; slack entries are diagonal.
  void build_P_matrix(int n)
  {
    const int n_vars = 3 * n;

    std::vector<Eigen::Triplet<double>> triplets;
    // Upper-left n×n block: upper triangle only (pattern never changes).
    triplets.reserve(static_cast<size_t>(n * (n + 1) / 2 + 2 * n));

    // Dense upper-triangle of the dq,dq block (values updated in solve).
    for (int i = 0; i < n; ++i) {
      for (int j = i; j < n; ++j) {
        triplets.emplace_back(i, j, 0.0);
      }
    }

    // Slack diagonal entries (constant — never change).
    for (int i = 0; i < n; ++i) {
      triplets.emplace_back(n + i, n + i, config_.osqp_slack_penalty);
    }
    for (int i = 0; i < n; ++i) {
      triplets.emplace_back(2 * n + i, 2 * n + i, config_.osqp_slack_penalty);
    }

    P_sparse_.resize(n_vars, n_vars);
    P_sparse_.setFromTriplets(triplets.begin(), triplets.end());
  }

  /// Update the upper-left n×n block of P with the current P_dq_dense_
  /// values.  The sparsity pattern (upper triangle) never changes so
  /// OsqpEigen performs a fast in-place value update.
  void update_P_dq_block(int n)
  {
    // Rebuild the P sparse matrix from the dense block + constant slack
    // diagonal.  For n=6 this is 33 nonzeros — negligible at 500 Hz.
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(static_cast<size_t>(n * (n + 1) / 2 + 2 * n));

    for (int i = 0; i < n; ++i) {
      for (int j = i; j < n; ++j) {
        triplets.emplace_back(i, j, P_dq_dense_(i, j));
      }
    }
    for (int i = 0; i < n; ++i) {
      triplets.emplace_back(n + i, n + i, config_.osqp_slack_penalty);
    }
    for (int i = 0; i < n; ++i) {
      triplets.emplace_back(2 * n + i, 2 * n + i, config_.osqp_slack_penalty);
    }

    P_sparse_.setFromTriplets(triplets.begin(), triplets.end());
    solver_->updateHessianMatrix(P_sparse_);
  }

  // --- Configuration ---
  SolverConfig config_;
  std::vector<std::string> joint_names_;
  std::string base_frame_, tip_frame_;
  size_t n_joints_{0};
  bool configured_{false};
  std::string last_error_;

  // --- Pinocchio ---
  std::unique_ptr<pinocchio::Model> model_;
  std::unique_ptr<pinocchio::Data> data_;
  pinocchio::FrameIndex tip_frame_id_{0};
  pinocchio::FrameIndex base_frame_id_{0};
  std::vector<int> joint_q_index_, joint_v_index_;

  // --- OsqpEigen ---
  std::unique_ptr<OsqpEigen::Solver> solver_;

  // --- Dense working buffers ---
  Eigen::VectorXd q_;
  Eigen::VectorXd neutral_q_;
  Eigen::VectorXd q_neutral_;
  Eigen::Matrix<double, 6, 1> task_error_;
  Eigen::MatrixXd jacobian_;
  Eigen::MatrixXd J_act_;
  Eigen::MatrixXd P_dq_dense_;     // n×n dense P dq-block
  Eigen::VectorXd q_vec_;           // n gradient for dq variables

  // --- Sparse matrices (pattern constant; values updated per solve) ---
  Eigen::SparseMatrix<double> A_sparse_;  // 6n × 3n linear constraints
  Eigen::SparseMatrix<double> P_sparse_;  // 3n × 3n Hessian (upper triangle)

  // --- OSQP vectors ---
  Eigen::Matrix<double, Eigen::Dynamic, 1> gradient_;     // 3n
  Eigen::Matrix<double, Eigen::Dynamic, 1> lower_bound_;  // 6n
  Eigen::Matrix<double, Eigen::Dynamic, 1> upper_bound_;  // 6n
  Eigen::Matrix<double, Eigen::Dynamic, 1> solution_;     // 3n

  // --- User-provided joint limits ---
  std::vector<double> joint_limit_lower_;
  std::vector<double> joint_limit_upper_;
  SolveResult result_;
};

}  // namespace manipulation_position_controllers::task_space
