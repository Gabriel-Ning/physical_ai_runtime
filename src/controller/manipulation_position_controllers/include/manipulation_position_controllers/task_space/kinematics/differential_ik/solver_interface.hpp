// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// Pose IK solver abstraction for task-space kinematic controllers.
//
// The interface solves one step from current joints toward a target tip pose.
// Backends may be DLS, OSQP, PlaCo, or future constrained IK solvers.
//
// Replacing the backend (e.g. swapping PinocchioDlsSolver for a future
// PlacoKinematicSolver) does not require controller code changes.

#pragma once

#include <array>
#include <string>
#include <vector>

#include "manipulation_position_controllers/common/parameter_validation.hpp"

namespace manipulation_position_controllers::task_space
{

/// Minimum controller period [s] all solve() backends clamp to before
/// integrating velocity into a position increment (dq = qdot * dt or
/// equivalent). Guards against a degenerate zero/negative period (e.g. an
/// unusual first update() call) without materially affecting any real
/// control loop -- 1e-6 s = 1 MHz is far above any real controller_manager
/// update_rate. Previously DLS applied no floor at all, OSQP floored at
/// 1e-9 s, and PlaCo floored at 1e-6 s; all three now share this constant.
constexpr double kMinSolverDtS = 1.0e-6;

/// Result returned by DifferentialIkSolver::solve().
struct SolveResult
{
  /// True when an IK solution was computed (may still be clamped).
  bool success{false};

  /// Command joint positions [rad], size = n_joints. Preallocated by solver.
  std::vector<double> q_command;

  /// Estimated joint velocity [rad/s], size = n_joints.
  std::vector<double> joint_velocity_estimate;

  /// Task-space position error norm [m].
  double task_position_error_norm{0.0};

  /// Task-space orientation error norm (approximate, from quaternion diff).
  double task_orientation_error_norm{0.0};

  /// Number of solver iterations used.
  int iterations{0};

  /// Solver-specific status code (0 = success).
  int status_code{0};
};

inline void allocate_solve_result(SolveResult & result, size_t joint_count)
{
  result.q_command.assign(joint_count, 0.0);
  result.joint_velocity_estimate.assign(joint_count, 0.0);
}

inline void reset_solve_result(SolveResult & result)
{
  result.success = false;
  std::fill(result.q_command.begin(), result.q_command.end(), 0.0);
  std::fill(
    result.joint_velocity_estimate.begin(),
    result.joint_velocity_estimate.end(), 0.0);
  result.task_position_error_norm = 0.0;
  result.task_orientation_error_norm = 0.0;
  result.iterations = 0;
  result.status_code = 0;
}

/// Solver configuration — all values have safe defaults.
struct SolverConfig
{
  /// Backend identifier (e.g. "pinocchio_dls", "placo").  Set by controller.
  std::string backend{"pinocchio_dls"};

  /// Proportional gain for position error [1/s].  Higher = faster convergence.
  double position_gain{4.0};

  /// Proportional gain for orientation error [1/s].
  double orientation_gain{1.0};

  /// Damping factor λ for damped least-squares (higher = more stable near
  /// singularities, at the cost of slower convergence).
  double damping{0.05};

  /// Maximum joint velocity [rad/s] used to clamp Δq after solve.
  double max_joint_velocity_rad_s{1.0};

  /// Maximum joint acceleration [rad/s²]. Bounds the per-cycle change in joint
  /// velocity (|Δq̇| ≤ a_max·dt) so commanded motion is C¹-smooth — a
  /// kinodynamic constraint, not output blending. 0 disables. Currently applied
  /// by the PlaCo backend.
  double max_joint_acceleration_rad_s2{0.0};

  /// Maximum linear task-space velocity [m/s].
  double max_linear_velocity_m_s{0.5};

  /// Maximum angular task-space velocity [rad/s].
  double max_angular_velocity_rad_s{1.0};

  /// Posture-task regularisation weight (attracts solution toward q_neutral).
  /// Set to 0.0 to disable.
  double posture_weight{1.0e-3};

  /// PlaCo manipulability task weight (soft). 0 disables. Useful on redundant
  /// arms to stay away from singularities.
  double manipulability_weight{0.0};

  /// PlaCo kinetic-energy regularization (smooth, mass-weighted Δq). Requires
  /// solver.dt. 0 disables. See ref/placo regularization.rst.
  double kinetic_energy_weight{0.0};

  /// Per-joint multipliers on PlaCo Δq regularization cost (>1 penalises motion
  /// on that joint, e.g. keep base joint quiet during orientation teleop).
  std::vector<double> joint_motion_weights;

  /// Maximum solver iterations per update.
  int max_iterations{1};

  // --- OSQP backend only. Ignored by pinocchio_dls and placo. ---

  /// OSQP internal ADMM iteration budget per solve() call. Distinct from
  /// max_iterations above (which PlaCo uses as outer time-substeps); this
  /// bounds the QP solver's own convergence loop. 200 is generous for a
  /// warm-started, well-conditioned 500 Hz solve.
  int osqp_max_admm_iterations{200};

  /// Joint-limit control-barrier-function gain [rad/s per rad]. Higher =
  /// tighter approach to the limit before the constraint engages. Robot- and
  /// limit-margin-dependent -- tune per robot, do not assume the default fits.
  double osqp_cbf_gain{2.0};

  /// Penalty on the CBF slack variables (large = use slack only when
  /// physically necessary to keep the QP feasible near limits).
  double osqp_slack_penalty{1.0e4};

  /// OSQP absolute convergence tolerance.
  double osqp_abs_tolerance{1.0e-6};

  /// OSQP relative convergence tolerance.
  double osqp_rel_tolerance{1.0e-6};

  /// OSQP ADMM step-size parameter (fixed; setAdaptiveRho(false)).
  double osqp_rho{0.1};
};

inline bool validate_solver_config(const SolverConfig & config, std::string & error)
{
  if (config.backend != "pinocchio_dls" && config.backend != "osqp" &&
      config.backend != "placo")
  {
    error = "solver.backend must be one of: pinocchio_dls, osqp, placo";
    return false;
  }
  if (!common::is_nonnegative_finite(config.position_gain) ||
      !common::is_nonnegative_finite(config.orientation_gain) ||
      !common::is_positive_finite(config.damping) ||
      !common::is_positive_finite(config.max_joint_velocity_rad_s) ||
      !common::is_positive_finite(config.max_linear_velocity_m_s) ||
      !common::is_positive_finite(config.max_angular_velocity_rad_s) ||
      !common::is_nonnegative_finite(config.max_joint_acceleration_rad_s2) ||
      !common::is_nonnegative_finite(config.posture_weight) ||
      !common::is_nonnegative_finite(config.manipulability_weight) ||
      !common::is_nonnegative_finite(config.kinetic_energy_weight) ||
      config.max_iterations <= 0)
  {
    error = "solver gains/weights/limits must be finite and within their documented positive ranges";
    return false;
  }
  if (!config.joint_motion_weights.empty() &&
      !common::all_positive_finite(config.joint_motion_weights))
  {
    error = "solver.joint_motion_weights entries must be finite and positive";
    return false;
  }
  if (config.backend == "osqp" &&
      (config.osqp_max_admm_iterations <= 0 ||
       !common::is_positive_finite(config.osqp_cbf_gain) ||
       !common::is_positive_finite(config.osqp_slack_penalty) ||
       !common::is_positive_finite(config.osqp_abs_tolerance) ||
       !common::is_positive_finite(config.osqp_rel_tolerance) ||
       !common::is_positive_finite(config.osqp_rho)))
  {
    error = "OSQP iteration count and tuning parameters must be finite and positive";
    return false;
  }
  error.clear();
  return true;
}

/// Abstract pose IK solver — no ROS dependencies.
///
/// Usage:
///   1. configure(urdf_string, joint_names, base_frame, tip_frame, config)
///   2. Every controller update: solve(q_current, pose_target, period)
class DifferentialIkSolver
{
public:
  DifferentialIkSolver() = default;
  virtual ~DifferentialIkSolver() = default;

  /// One-time initialisation.  URDF string is passed so the solver can build
  /// its own kinematic model.  Called from on_configure().
  ///
  /// @param urdf_xml        Robot description as URDF / XML string.
  /// @param joint_names     Ordered joint names (size = n_joints).
  /// @param base_frame      Base / root link of the kinematic chain.
  /// @param tip_frame       End-effector frame whose pose is being controlled.
  /// @param config          Solver hyper-parameters.
  /// @return                true on success.
  virtual bool configure(
    const std::string & urdf_xml,
    const std::vector<std::string> & joint_names,
    const std::string & base_frame,
    const std::string & tip_frame,
    const SolverConfig & config) = 0;

  /// Re-seed the solver state to a given joint configuration.
  /// Called once on controller activation.
  virtual void reset(const std::vector<double> & q_current) = 0;

  /// Compute one IK step toward the target pose.
  ///
  /// @param q_current     Current joint positions [rad], size = n_joints.
  /// @param pose_target   Desired tip-frame pose {x,y,z, qx,qy,qz,qw}.
  /// @param period        Controller update period [s].
  /// @return              Reference to solver-owned, preallocated result.
  ///
  /// Must not allocate in the steady-state path.  Must not throw.
  virtual const SolveResult & solve(
    const std::vector<double> & q_current,
    const std::array<double, 7> & pose_target,
    double period) = 0;

  /// Compute the current tip-frame pose for q_current.
  ///
  /// @param q_current     Joint positions [rad], size = n_joints.
  /// @param pose          Output tip pose {x,y,z, qx,qy,qz,qw}.
  /// @return              true when FK was computed.
  ///
  /// Used by controllers that need to seed an internal pose target before
  /// integrating incremental commands. Must not throw.
  virtual bool compute_tip_pose(
    const std::vector<double> & q_current,
    std::array<double, 7> & pose)
  {
    (void)q_current;
    (void)pose;
    return false;
  }

  /// Human-readable backend error from the last configure/solve failure.
  virtual const std::string & last_error() const
  {
    static const std::string empty;
    return empty;
  }
};

}  // namespace manipulation_position_controllers::task_space
