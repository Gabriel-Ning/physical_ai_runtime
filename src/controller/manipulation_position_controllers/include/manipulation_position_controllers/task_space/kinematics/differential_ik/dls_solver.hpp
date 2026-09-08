// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// Differential IK solver backed by Pinocchio (FK + frame Jacobian + DLS).
// Generic — the kinematic model is built from a URDF string at configure() time.

#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "manipulation_position_controllers/task_space/kinematics/differential_ik/solver_interface.hpp"

namespace manipulation_position_controllers::task_space
{

/// Pinocchio-based damped least-squares solver.
///
/// Thread-safe for a single realtime thread after configure().
/// All working buffers are allocated in configure() — solve() does not allocate.
class PinocchioDlsSolver : public DifferentialIkSolver
{
public:
  PinocchioDlsSolver() = default;
  ~PinocchioDlsSolver() override = default;

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

    // Build Pinocchio model from URDF string.
    // The URDF joint order must match joint_names_.
    model_ = std::make_unique<pinocchio::Model>();
    try {
      pinocchio::urdf::buildModelFromXML(urdf_xml, *model_);
    } catch (const std::exception & e) {
      last_error_ = std::string("Pinocchio URDF build failed: ") + e.what();
      return false;
    }

    data_ = std::make_unique<pinocchio::Data>(*model_);

    // Verify the tip frame exists.
    if (!model_->existFrame(tip_frame_)) {
      last_error_ =
        "Frame '" + tip_frame_ +
        "' not found in URDF. Available frames: " +
        std::to_string(model_->nframes) + " total.";
      return false;
    }
    tip_frame_id_ = model_->getFrameId(tip_frame_);

    // Verify the base frame exists. All task-space poses (targets and the FK
    // returned by compute_tip_pose) are expressed relative to this frame, so
    // the solver must be able to locate it. When base_frame is the URDF root
    // the transform below reduces to identity and behaviour is unchanged.
    if (!model_->existFrame(base_frame_)) {
      last_error_ =
        "Base frame '" + base_frame_ + "' not found in URDF.";
      return false;
    }
    base_frame_id_ = model_->getFrameId(base_frame_);

    // Validate model dimensions.
    if (model_->nq <= 0 || model_->nv <= 0 ||
        model_->nq > 256 || model_->nv > 256) {
      last_error_ = "Invalid Pinocchio model dimensions: nq=" +
        std::to_string(model_->nq) + " nv=" + std::to_string(model_->nv) +
        " njoints=" + std::to_string(model_->njoints) +
        " nframes=" + std::to_string(model_->nframes);
      return false;
    }
    if (model_->nv < static_cast<int>(n_joints_)) {
      last_error_ = "Model nv=" + std::to_string(model_->nv) +
        " < expected n_joints=" + std::to_string(n_joints_);
      return false;
    }

    // Pre-allocate working buffers.
    q_.resize(model_->nq);
    q_.setZero();
    neutral_q_ = pinocchio::neutral(*model_);
    q_command_.resize(n_joints_);
    q_command_.setZero();
    dq_.resize(model_->nv);
    dq_.setZero();
    task_error_.resize(6);
    task_error_.setZero();
    jacobian_.resize(6, model_->nv);
    jacobian_.setZero();
    J_act_.resize(6, n_joints_);
    J_act_.setZero();
    jt_j_.resize(n_joints_, n_joints_);
    jt_j_.setZero();
    rhs_.resize(n_joints_);
    rhs_.setZero();
    qdot_act_.resize(n_joints_);
    qdot_act_.setZero();
    dq_act_.resize(n_joints_);
    dq_act_.setZero();

    // Posture (neutral) configuration — initialised to zeros.
    q_neutral_.resize(model_->nq);
    q_neutral_.setZero();

    // Null-space posture workspaces (sized once; reused every solve so the
    // realtime path performs no heap allocation).
    posture_z_.resize(n_joints_);
    posture_z_.setZero();
    posture_jtjz_.resize(n_joints_);
    posture_jtjz_.setZero();
    posture_proj_.resize(n_joints_);
    posture_proj_.setZero();

    // Map from our joint index → Pinocchio q index.
    joint_q_index_.resize(n_joints_, -1);
    joint_v_index_.resize(n_joints_, -1);
    for (size_t i = 0; i < n_joints_; ++i) {
      const auto & name = joint_names_[i];
      if (model_->existJointName(name)) {
        pinocchio::JointIndex jid = model_->getJointId(name);
        joint_q_index_[i] = static_cast<int>(model_->joints[jid].idx_q());
        joint_v_index_[i] = static_cast<int>(model_->joints[jid].idx_v());
      } else {
        last_error_ = "Joint '" + name + "' not found in URDF model.";
        return false;
      }
    }

    last_error_.clear();
    allocate_solve_result(result_, n_joints_);
    // Prime Eigen's factorization workspace outside the realtime path.
    jt_j_.setIdentity();
    ldlt_.compute(jt_j_);
    jt_j_.setZero();
    configured_ = true;
    return true;
  }

  void reset(const std::vector<double> & q_current) override
  {
    // Seed neutral posture to current configuration.
    write_q(q_current);
    q_neutral_ = q_;
    q_command_.resize(n_joints_);
    q_command_.setZero();
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

    // Express the tip pose in the base frame (bMt = oMb⁻¹ · oMt) so the value
    // is consistent with the base-frame targets passed to solve().
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

    if (!configured_) {
      result.status_code = -1;
      return result;
    }

    // 1. Update Pinocchio state from current joint positions.
    write_q(q_current);
    pinocchio::forwardKinematics(*model_, *data_, q_);
    pinocchio::updateFramePlacements(*model_, *data_);

    // 2. Compute task-space error.
    const auto & tip_M = data_->oMf[tip_frame_id_];
    const Eigen::Vector3d current_pos = tip_M.translation();
    const Eigen::Quaterniond current_quat(tip_M.rotation());

    // The target is expressed in base_frame. Lift it into the model-root
    // (world) frame so it is consistent with the world-aligned Jacobian and FK
    // below: world_T_target = oMb · base_T_target. For base_frame == root,
    // oMb is identity and this is a no-op.
    Eigen::Quaterniond target_quat_base(
      pose_target[6],   // w
      pose_target[3],   // x
      pose_target[4],   // y
      pose_target[5]);  // z
    target_quat_base.normalize();
    const pinocchio::SE3 base_T_target(
      target_quat_base.toRotationMatrix(),
      Eigen::Vector3d(pose_target[0], pose_target[1], pose_target[2]));
    const pinocchio::SE3 world_T_target =
      data_->oMf[base_frame_id_] * base_T_target;
    const Eigen::Vector3d target_pos = world_T_target.translation();
    Eigen::Quaterniond target_quat(world_T_target.rotation());

    // Position error (world frame).
    task_error_.head<3>() = target_pos - current_pos;

    // Orientation error (world frame, using quaternion difference).
    // δω ≈ 2 * Im(q_target * q_current.conjugate())
    Eigen::Quaterniond q_error = target_quat * current_quat.conjugate();
    if (q_error.w() < 0.0) {
      q_error.coeffs() *= -1.0;  // take the shorter path
    }
    task_error_.tail<3>() = 2.0 * q_error.vec();

    result.task_position_error_norm = task_error_.head<3>().norm();
    result.task_orientation_error_norm = task_error_.tail<3>().norm();

    // 3. Compute frame Jacobian (6 × nv) in world frame.
    pinocchio::computeFrameJacobian(
      *model_, *data_, q_, tip_frame_id_,
      pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_);

    // Map full Jacobian → actuated-joint Jacobian.
    for (size_t j = 0; j < static_cast<size_t>(n_joints_); ++j) {
      J_act_.col(j) = jacobian_.col(joint_v_index_[j]);
    }

    // Convert task error into a desired Cartesian twist. Linear/angular task
    // velocity limits are separate from joint velocity limits; the latter is
    // applied after IK.
    Eigen::Matrix<double, 6, 1> v_desired;
    v_desired.head<3>() = config_.position_gain * task_error_.head<3>();
    v_desired.tail<3>() = config_.orientation_gain * task_error_.tail<3>();

    const double linear_speed = v_desired.head<3>().norm();
    if (linear_speed > config_.max_linear_velocity_m_s && linear_speed > 1e-12) {
      v_desired.head<3>() *= config_.max_linear_velocity_m_s / linear_speed;
    }
    const double angular_speed = v_desired.tail<3>().norm();
    if (angular_speed > config_.max_angular_velocity_rad_s && angular_speed > 1e-12) {
      v_desired.tail<3>() *= config_.max_angular_velocity_rad_s / angular_speed;
    }

    // 4. Damped least-squares: qdot = J^T * (J*J^T + λ²I)^(-1) * v_desired.
    // Equivalent to: min ||J*qdot - v_desired||² + λ²||qdot||².
    const double lambda_sq = config_.damping * config_.damping;
    jt_j_.noalias() = J_act_.transpose() * J_act_;
    for (int i = 0; i < static_cast<int>(n_joints_); ++i) {
      jt_j_(i, i) += lambda_sq;
    }
    rhs_.noalias() = J_act_.transpose() * v_desired;

    ldlt_.compute(jt_j_);
    if (ldlt_.info() != Eigen::Success) {
      result.status_code = -2;
      return result;
    }
    qdot_act_ = ldlt_.solve(rhs_);

    // 5. Optional posture task, projected through the damped null-space
    //    projector of the primary task:
    //      qdot += N·z,  N = I − J⁺J,  J⁺ = (JᵀJ + λ²I)⁻¹Jᵀ
    //    Computed matrix-free as N·z = z − J⁺(J·z), reusing the LDLT of
    //    (JᵀJ + λ²I) already factored above (workspaces preallocated in
    //    configure() — no realtime allocation).
    //
    //    NOTE: with λ > 0 this is the *damped* projector, not an exact
    //    null-space projector — J·N is small but not identically zero, so a
    //    little posture motion leaks into the task. The leakage scales with λ²
    //    and the posture weight and vanishes as λ → 0. Keep posture_weight
    //    small. (For a non-redundant arm, e.g. a 6-DOF manipulator with a
    //    square Jacobian, the null space is empty and posture has ~no effect by
    //    construction.) This is still strictly better than the previous bare
    //    additive term qdot += z, which injected the full posture velocity into
    //    the task.
    if (config_.posture_weight > 0.0) {
      for (int i = 0; i < static_cast<int>(n_joints_); ++i) {
        const int qi = joint_q_index_[i];
        posture_z_[i] = (qi >= 0 && qi < static_cast<int>(model_->nq))
          ? config_.posture_weight * (q_neutral_[qi] - q_[qi])
          : 0.0;
      }
      const Eigen::Matrix<double, 6, 1> Jz = J_act_ * posture_z_;
      posture_jtjz_.noalias() = J_act_.transpose() * Jz;
      posture_proj_.noalias() = ldlt_.solve(posture_jtjz_);
      qdot_act_.noalias() += posture_z_ - posture_proj_;
    }

    // 6. Clamp qdot per joint by max velocity, then integrate one controller step.
    double max_qdot = 0.0;
    for (int i = 0; i < static_cast<int>(n_joints_); ++i) {
      max_qdot = std::max(max_qdot, std::abs(qdot_act_[i]));
    }
    if (max_qdot > config_.max_joint_velocity_rad_s && max_qdot > 1e-12) {
      const double scale = config_.max_joint_velocity_rad_s / max_qdot;
      qdot_act_ *= scale;
    }
    const double dt = std::max(period, kMinSolverDtS);
    dq_act_.noalias() = qdot_act_ * dt;

    // 7. Write result: q_command = q_current + dq (q_current comes from controller).
    for (size_t i = 0; i < n_joints_; ++i) {
      int qi = joint_q_index_[i];
      if (qi >= 0 && qi < static_cast<int>(model_->nq)) {
        result.q_command[i] = q_[qi] + dq_act_[i];
        result.joint_velocity_estimate[i] = qdot_act_[i];
      } else {
        result.q_command[i] = q_current[i];
      }
    }

    result.success = true;
    result.iterations = 1;
    result.status_code = 0;
    return result;
  }

  const std::string & get_last_error() const { return last_error_; }
  const std::string & last_error() const override { return last_error_; }

private:
  void write_q(const std::vector<double> & q_current)
  {
    // Neutralise Pinocchio q vector first.
    q_ = neutral_q_;
    for (size_t i = 0; i < n_joints_; ++i) {
      int qi = joint_q_index_[i];
      if (qi >= 0 && qi < static_cast<int>(model_->nq)) {
        q_[qi] = q_current[i];
      }
    }
  }

  // --- Configuration ---
  SolverConfig config_;
  std::vector<std::string> joint_names_;
  std::string base_frame_;
  std::string tip_frame_;
  size_t n_joints_{0};
  bool configured_{false};
  std::string last_error_;

  // --- Pinocchio model ---
  std::unique_ptr<pinocchio::Model> model_;
  std::unique_ptr<pinocchio::Data> data_;
  pinocchio::FrameIndex tip_frame_id_{0};
  pinocchio::FrameIndex base_frame_id_{0};

  // Joint index mapping: controller order → Pinocchio q/v index.
  std::vector<int> joint_q_index_;
  std::vector<int> joint_v_index_;

  // --- Pre-allocated buffers (no allocation in solve()) ---
  Eigen::VectorXd q_;
  Eigen::VectorXd neutral_q_;
  Eigen::VectorXd q_command_;
  Eigen::VectorXd dq_;
  Eigen::VectorXd q_neutral_;
  Eigen::Matrix<double, 6, 1> task_error_;
  Eigen::MatrixXd jacobian_;
  Eigen::MatrixXd J_act_;
  Eigen::MatrixXd jt_j_;
  Eigen::LDLT<Eigen::MatrixXd> ldlt_;
  Eigen::VectorXd rhs_;
  Eigen::VectorXd qdot_act_;
  Eigen::VectorXd dq_act_;
  Eigen::VectorXd posture_z_;      // posture velocity in actuated-joint space
  Eigen::VectorXd posture_jtjz_;   // Jᵀ(J·z) scratch
  Eigen::VectorXd posture_proj_;   // J⁺(J·z) = damped projection of z
  SolveResult result_;
};

}  // namespace manipulation_position_controllers::task_space
