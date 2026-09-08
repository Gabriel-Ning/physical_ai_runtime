// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// Ruckig Online Trajectory Generation adapter for JointSpacePositionController.
//
// Wraps ruckig::Ruckig<0> (runtime DOFs) to produce jerk-bounded position
// commands from sampled joint references.  Replaces EMA + rate-limiter
// when trajectory_behavior.mode == "ruckig".

#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include <ruckig/ruckig.hpp>

namespace manipulation_position_controllers::joint_space
{

/// Configuration for the Ruckig adapter — all values are per-joint.
struct RuckigConfig
{
  double max_velocity_rad_s{1.5};
  double max_acceleration_rad_s2{3.0};
  double max_jerk_rad_s3{10.0};
};

/// Thin wrapper around ruckig::Ruckig that manages input/output buffers and
/// presents a simple configure / reset / update interface for the controller.
class RuckigAdapter
{
public:
  RuckigAdapter() = default;

  /// Initialise (or re-initialise) the OTG for a given DOF count and timestep.
  /// Must be called before update().  Safe to call again on DOF change.
  void configure(size_t dofs, double dt_s, const RuckigConfig & config);

  /// Reset internal state so the next update() triggers a fresh trajectory
  /// calculation (e.g. after a target change or deactivate/activate cycle).
  void reset();

  /// Compute the next position command along the OTG trajectory.
  ///
  /// \param current_position  current joint positions [rad] (size == dofs)
  /// \param target_position   desired joint positions [rad] (size == dofs)
  /// \param[out] output_position  smoothed position command [rad] (size == dofs)
  /// \return true for Working or Finished, false on Ruckig error.
  bool update(
    const std::vector<double> & current_position,
    const std::vector<double> & target_position,
    std::vector<double> & output_position);

  bool update(
    const std::vector<double> & current_position,
    const std::vector<double> & target_position,
    std::vector<double> & output_position,
    std::vector<double> & output_velocity);

  /// Velocity-MG style step aligned with franka-vr-teleop:
  /// - current_position from measured q each cycle
  /// - current_velocity/acceleration from previous Ruckig output (command continuity)
  /// - target_velocity = 0; write new_velocity
  /// Does not use pass_to_input for position (unlike the Position-command update).
  bool update_velocity_command(
    const std::vector<double> & measured_position,
    const std::vector<double> & target_position,
    std::vector<double> & output_velocity);

  /// Extrapolate input state using constant acceleration model for missed time gap.
  /// \param miss_dt  elapsed time gap [s]
  void extrapolate_constant_accel(double miss_dt);

  /// Access the underlying Ruckig instance (for direct status queries).
  const ruckig::Ruckig<0> & otg() const { return *otg_; }

  /// Time on current trajectory [s].
  double trajectory_time() const { return output_ ? output_->time : 0.0; }

  /// Duration of the current trajectory [s].
  double trajectory_duration() const { return output_ ? output_->trajectory.get_duration() : 0.0; }

private:
  size_t dofs_{0};
  RuckigConfig config_;
  std::unique_ptr<ruckig::Ruckig<0>> otg_;
  std::unique_ptr<ruckig::InputParameter<0>> input_;
  std::unique_ptr<ruckig::OutputParameter<0>> output_;
  std::vector<double> target_position_;
  bool input_set_{false};
};

}  // namespace manipulation_position_controllers::joint_space
