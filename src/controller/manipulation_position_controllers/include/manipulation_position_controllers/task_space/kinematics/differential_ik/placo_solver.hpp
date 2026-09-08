// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// PlaCo QP-based IK backend for TaskSpaceKinematicPositionController.

#pragma once

#include "manipulation_position_controllers/task_space/kinematics/differential_ik/solver_interface.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <placo/kinematics/joints_task.h>
#include <placo/kinematics/kinematics_solver.h>
#include <placo/kinematics/kinetic_energy_regularization_task.h>
#include <placo/kinematics/manipulability_task.h>
#include <placo/kinematics/orientation_task.h>
#include <placo/kinematics/position_task.h>
#include <placo/kinematics/regularization_task.h>
#include <placo/model/robot_wrapper.h>

namespace manipulation_position_controllers::task_space
{

/// PlaCo-backed QP IK solver.
///
/// Mathematical formulation implemented here, per controller update:
///
///   Current and target tip poses are expressed in the world frame:
///     T_wt(q) = FK(tip_frame)
///     T_wg    = T_wb * T_bg_target
///
///   Tracking errors used for status reporting:
///     e_p = p_wg - p_wt
///     e_R = 2 * vec(q_wg * q_wt^{-1}) with the shortest quaternion sign
///
///   PlaCo task stack:
///     PositionTask(tip_frame, p_wg), soft weight = position_gain
///     OrientationTask(tip_frame, R_wg), soft weight = orientation_gain
///     optional JointsTask(q_neutral), soft weight = posture_weight
///     optional RegularizationTask(weight = damping^2)
///     optional per-joint RegularizationTask weights:
///       weight_i = damping^2 * joint_motion_weights[i]
///     optional KineticEnergyRegularizationTask(weight = kinetic_energy_weight)
///     optional ManipulabilityTask(tip_frame, "both")
///
///   PlaCo sees the full Cartesian target every cycle. Speed is bounded by its
///   native joint velocity limits, not by shrinking the Cartesian target before
///   the QP. The gain parameters are therefore task weights in this backend.
///
///   PlaCo solves its internal QP over a joint displacement Δq:
///     minimize  position_gain * ||PositionTask(q + Δq) - p_wg||²
///             + orientation_gain * ||OrientationTask(q + Δq) - R_wg||²
///             + posture_weight * ||q + Δq - q_neutral||²
///             + damping² * ||Δq||²
///             + kinetic_energy_weight * T_kinetic(Δq)
///             + manipulability_weight * M(q + Δq)
///     subject to PlaCo native joint-position and joint-velocity limits.
///
///   This is a PlaCo-native Pink/Mink-style weighted least-squares stack:
///   posture is a small soft objective in the same QP, not an analytical
///   null-space projection.
///
///   This backend intentionally does not add an acceleration-limit task or
///   constraint. Hardware testing showed that teleoperation smoothness is best
///   handled by kinetic energy regularization, with damping and per-joint
///   regularization weights shaping joint distribution.
///
///   A final velocity clamp is kept as a defensive safety net around the
///   extracted qdot.
///
/// Core difference from OsqpIkSolver:
///   OSQP builds one explicit 6D least-squares objective
///     min ||J6*dq - [v_linear; v_angular]*dt||^2 + λ^2||dq||^2
///   with position and orientation coupled in the same Jacobian cost. This
///   PlaCo backend delegates to separate PositionTask and OrientationTask soft
///   costs plus secondary regularizers, so per-joint motion weights are only a
///   regularization preference, not the metric of the primary 6D pose task.
class PlacoKinematicSolver : public DifferentialIkSolver
{
public:
  PlacoKinematicSolver() = default;
  ~PlacoKinematicSolver() override = default;

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

    if (n_joints_ == 0) {
      last_error_ = "PlaCo solver requires at least one controlled joint.";
      return false;
    }

    try {
      robot_ = std::make_unique<placo::model::RobotWrapper>(
        "", placo::model::RobotWrapper::IGNORE_COLLISIONS, urdf_xml);

      (void)robot_->get_frame_index(tip_frame_);
      (void)robot_->get_frame_index(base_frame_);
      for (size_t i = 0; i < n_joints_; ++i) {
        const auto & joint_name = joint_names_[i];
        (void)robot_->get_joint_v_offset(joint_name);
        robot_->set_velocity_limit(joint_name, config_.max_joint_velocity_rad_s);
      }

      solver_ = std::make_unique<placo::kinematics::KinematicsSolver>(*robot_);
      solver_->mask_fbase(true);
      solver_->enable_joint_limits(true);
      solver_->enable_velocity_limits(true);

      position_task_ = &solver_->add_position_task(
        tip_frame_, Eigen::Vector3d::Zero());
      orientation_task_ = &solver_->add_orientation_task(
        tip_frame_, Eigen::Matrix3d::Identity());

      position_task_->configure(
        "task_space_ee_pos", "soft", task_weight(config_.position_gain));
      orientation_task_->configure(
        "task_space_ee_ori", "soft", task_weight(config_.orientation_gain));

      if (config_.posture_weight > 0.0) {
        joints_task_ = &solver_->add_joints_task();
        for (const auto & joint_name : joint_names_) {
          joints_task_->set_joint(joint_name, 0.0);
        }
        joints_task_->configure(
          "posture", "soft", config_.posture_weight);
      }

      const bool want_joint_motion_weights =
        !config_.joint_motion_weights.empty();
      if (config_.damping > 0.0 || want_joint_motion_weights) {
        const double reg = std::max(
          config_.damping * config_.damping, 1.0e-8);
        regularization_task_ = &solver_->add_regularization_task(reg);
        apply_joint_motion_weights(reg);
      }

      if (config_.kinetic_energy_weight > 0.0) {
        kinetic_task_ = &solver_->add_kinetic_energy_regularization_task(
          config_.kinetic_energy_weight);
      }

      if (config_.manipulability_weight > 0.0) {
        manipulability_task_ = &solver_->add_manipulability_task(
          tip_frame_, "both", 1.0);
        manipulability_task_->configure(
          "manipulability", "soft", config_.manipulability_weight);
      }

      robot_->update_kinematics();
      const Eigen::Affine3d tip0 = robot_->get_T_world_frame(tip_frame_);
      position_task_->target_world = tip0.translation();
      orientation_task_->R_world_frame = tip0.rotation();
    } catch (const std::exception & e) {
      last_error_ = std::string("PlaCo configure failed: ") + e.what();
      configured_ = false;
      return false;
    }

    last_error_.clear();
    configured_ = true;
    dq_.assign(n_joints_, 0.0);
    qdot_.assign(n_joints_, 0.0);
    allocate_solve_result(result_, n_joints_);
    return true;
  }

  void reset(const std::vector<double> & q_current) override
  {
    if (!configured_) {
      return;
    }
    write_joints(q_current);
    q_neutral_ = q_current;
    update_posture_targets();

    robot_->update_kinematics();
    const Eigen::Affine3d tip = robot_->get_T_world_frame(tip_frame_);
    position_task_->target_world = tip.translation();
    orientation_task_->R_world_frame = tip.rotation();
  }

  bool compute_tip_pose(
    const std::vector<double> & q_current,
    std::array<double, 7> & pose) override
  {
    if (!configured_) {
      return false;
    }

    try {
      write_joints(q_current);
      robot_->update_kinematics();
      const Eigen::Affine3d bMt =
        robot_->get_T_world_frame(base_frame_).inverse() *
        robot_->get_T_world_frame(tip_frame_);
      const Eigen::Quaterniond quat(bMt.rotation());
      pose[0] = bMt.translation().x();
      pose[1] = bMt.translation().y();
      pose[2] = bMt.translation().z();
      pose[3] = quat.x();
      pose[4] = quat.y();
      pose[5] = quat.z();
      pose[6] = quat.w();
    } catch (const std::exception & e) {
      last_error_ = std::string("PlaCo FK failed: ") + e.what();
      return false;
    }

    return true;
  }

  const SolveResult & solve(
    const std::vector<double> & q_current,
    const std::array<double, 7> & pose_target,
    double period) override
  {
    reset_solve_result(result_);
    auto & result = result_;

    if (!configured_ || period <= 0.0) {
      result.status_code = -1;
      return result;
    }

    try {
      write_joints(q_current);
      robot_->update_kinematics();

      const double dt = std::max(period, kMinSolverDtS);

      const Eigen::Affine3d current = robot_->get_T_world_frame(tip_frame_);
      const Eigen::Affine3d target =
        robot_->get_T_world_frame(base_frame_) * target_from_pose(pose_target);

      result.task_position_error_norm =
        (target.translation() - current.translation()).norm();

      Eigen::Quaterniond current_quat(current.rotation());
      Eigen::Quaterniond target_quat(target.rotation());
      Eigen::Quaterniond q_error = target_quat * current_quat.conjugate();
      if (q_error.w() < 0.0) {
        q_error.coeffs() *= -1.0;
      }
      result.task_orientation_error_norm = (2.0 * q_error.vec()).norm();

      const int iterations = std::max(1, config_.max_iterations);
      const double sub_dt = dt / static_cast<double>(iterations);
      solver_->dt = sub_dt;
      for (int iter = 0; iter < iterations; ++iter) {
        robot_->update_kinematics();
        set_task_targets(target);
        (void)solver_->solve(true);
      }

      // Read the solved displacement back as a joint-velocity vector.
      for (size_t i = 0; i < n_joints_; ++i) {
        qdot_[i] = (robot_->get_joint(joint_names_[i]) - q_current[i]) / dt;
      }

      // Defensive velocity clamp. PlaCo also enforces this natively.
      double max_qdot = 0.0;
      for (size_t i = 0; i < n_joints_; ++i) {
        max_qdot = std::max(max_qdot, std::abs(qdot_[i]));
      }
      if (max_qdot > config_.max_joint_velocity_rad_s && max_qdot > 1e-12) {
        const double scale = config_.max_joint_velocity_rad_s / max_qdot;
        for (size_t i = 0; i < n_joints_; ++i) {
          qdot_[i] *= scale;
        }
      }

      for (size_t i = 0; i < n_joints_; ++i) {
        dq_[i] = qdot_[i] * dt;
        result.q_command[i] = q_current[i] + dq_[i];
        result.joint_velocity_estimate[i] = qdot_[i];
      }
    } catch (const std::exception & e) {
      last_error_ = std::string("PlaCo solve failed: ") + e.what();
      result.status_code = -2;
      return result;
    }

    last_error_.clear();
    result.success = true;
    result.iterations = std::max(1, config_.max_iterations);
    result.status_code = 0;
    return result;
  }

  const std::string & last_error() const override { return last_error_; }

private:
  void apply_joint_motion_weights(double base_reg)
  {
    if (!regularization_task_) {
      return;
    }
    for (size_t i = 0; i < n_joints_; ++i) {
      double scale = 1.0;
      if (i < config_.joint_motion_weights.size()) {
        scale = config_.joint_motion_weights[i];
      }
      if (scale > 0.0) {
        regularization_task_->set_joint_weight(
          joint_names_[i], base_reg * scale);
      }
    }
  }

  void write_joints(const std::vector<double> & q)
  {
    for (size_t i = 0; i < n_joints_ && i < q.size(); ++i) {
      robot_->set_joint(joint_names_[i], q[i]);
    }
  }

  void update_posture_targets()
  {
    if (!joints_task_ || q_neutral_.size() != n_joints_) {
      return;
    }
    for (size_t i = 0; i < n_joints_; ++i) {
      joints_task_->set_joint(joint_names_[i], q_neutral_[i]);
    }
  }

  void set_task_targets(const Eigen::Affine3d & target)
  {
    position_task_->target_world = target.translation();
    orientation_task_->R_world_frame = target.rotation();
  }

  static double task_weight(double value)
  {
    return std::max(value, 1.0e-8);
  }

  static Eigen::Affine3d target_from_pose(const std::array<double, 7> & pose)
  {
    Eigen::Quaterniond quat(pose[6], pose[3], pose[4], pose[5]);
    quat.normalize();

    Eigen::Affine3d target = Eigen::Affine3d::Identity();
    target.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    target.linear() = quat.toRotationMatrix();
    return target;
  }

  SolverConfig config_;
  std::vector<std::string> joint_names_;
  std::vector<double> q_neutral_;
  std::vector<double> dq_;
  std::vector<double> qdot_;
  std::string base_frame_;
  std::string tip_frame_;
  size_t n_joints_{0};
  bool configured_{false};
  std::string last_error_;

  std::unique_ptr<placo::model::RobotWrapper> robot_;
  std::unique_ptr<placo::kinematics::KinematicsSolver> solver_;
  placo::kinematics::PositionTask * position_task_{nullptr};
  placo::kinematics::OrientationTask * orientation_task_{nullptr};
  placo::kinematics::JointsTask * joints_task_{nullptr};
  placo::kinematics::RegularizationTask * regularization_task_{nullptr};
  placo::kinematics::KineticEnergyRegularizationTask * kinetic_task_{nullptr};
  placo::kinematics::ManipulabilityTask * manipulability_task_{nullptr};
  SolveResult result_;
};

}  // namespace manipulation_position_controllers::task_space
