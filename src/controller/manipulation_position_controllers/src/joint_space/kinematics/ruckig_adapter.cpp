// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include "manipulation_position_controllers/joint_space/kinematics/ruckig_adapter.hpp"

#include <algorithm>
#include <cmath>

namespace manipulation_position_controllers::joint_space
{

void RuckigAdapter::configure(
  size_t dofs, double dt_s, const RuckigConfig & config)
{
  dofs_ = dofs;
  config_ = config;

  otg_ = std::make_unique<ruckig::Ruckig<0>>(dofs, dt_s);

  // Construct input and output with the correct DOF count.
  // resize() is private in ruckig — we must use the sized constructors.
  input_ = std::make_unique<ruckig::InputParameter<0>>(dofs);
  output_ = std::make_unique<ruckig::OutputParameter<0>>(dofs);

  // Set per-DOF kinematic limits (these don't change during operation).
  for (size_t dof = 0; dof < dofs; ++dof) {
    input_->max_velocity[dof] = config_.max_velocity_rad_s;
    input_->max_acceleration[dof] = config_.max_acceleration_rad_s2;
    input_->max_jerk[dof] = config_.max_jerk_rad_s3;
    input_->enabled[dof] = true;
    input_->current_position[dof] = 0.0;
    input_->current_velocity[dof] = 0.0;
    input_->current_acceleration[dof] = 0.0;
    input_->target_position[dof] = 0.0;
    input_->target_velocity[dof] = 0.0;
    input_->target_acceleration[dof] = 0.0;
  }

  input_->control_interface = ruckig::ControlInterface::Position;
  input_->synchronization = ruckig::Synchronization::Time;

  target_position_.assign(dofs, 0.0);
  input_set_ = false;
}

void RuckigAdapter::reset()
{
  if (otg_) {
    otg_->reset();
  }
  input_set_ = false;
}

bool RuckigAdapter::update(
  const std::vector<double> & current_position,
  const std::vector<double> & target_position,
  std::vector<double> & output_position)
{
  if (!otg_ || !input_ || !output_ || dofs_ == 0) {
    return false;
  }
  if (
    current_position.size() < dofs_ || target_position.size() < dofs_ ||
    output_position.size() < dofs_)
  {
    return false;
  }

  if (!input_set_) {
    for (size_t dof = 0; dof < dofs_; ++dof) {
      input_->current_position[dof] = current_position[dof];
      input_->current_velocity[dof] = 0.0;
      input_->current_acceleration[dof] = 0.0;
      input_->target_velocity[dof] = 0.0;
      input_->target_acceleration[dof] = 0.0;
    }
    std::copy_n(target_position.begin(), dofs_, target_position_.begin());
    input_set_ = true;
  }

  bool target_changed = false;
  for (size_t dof = 0; dof < dofs_; ++dof) {
    if (std::abs(target_position_[dof] - target_position[dof]) > 1e-12) {
      target_changed = true;
      break;
    }
  }
  if (target_changed) {
    std::copy_n(target_position.begin(), dofs_, target_position_.begin());
  }

  for (size_t dof = 0; dof < dofs_; ++dof) {
    input_->target_position[dof] = target_position_[dof];
    input_->target_velocity[dof] = 0.0;
    input_->target_acceleration[dof] = 0.0;
  }

  const auto result = otg_->update(*input_, *output_);

  for (size_t dof = 0; dof < dofs_; ++dof) {
    output_position[dof] = output_->new_position[dof];
  }

  if (result != ruckig::Result::Working && result != ruckig::Result::Finished) {
    return false;
  }

  output_->pass_to_input(*input_);
  return true;
}

bool RuckigAdapter::update(
  const std::vector<double> & current_position,
  const std::vector<double> & target_position,
  std::vector<double> & output_position,
  std::vector<double> & output_velocity)
{
  const bool success = update(current_position, target_position, output_position);
  if (success && output_ && output_velocity.size() >= dofs_) {
    for (size_t dof = 0; dof < dofs_; ++dof) {
      output_velocity[dof] = output_->new_velocity[dof];
    }
  }
  return success;
}

bool RuckigAdapter::update_velocity_command(
  const std::vector<double> & measured_position,
  const std::vector<double> & target_position,
  std::vector<double> & output_velocity)
{
  if (!otg_ || !input_ || !output_ || dofs_ == 0) {
    return false;
  }
  if (
    measured_position.size() < dofs_ || target_position.size() < dofs_ ||
    output_velocity.size() < dofs_)
  {
    return false;
  }

  if (!input_set_) {
    // First cycle: match vr-teleop — zero commanded vel/accel, target = measured.
    for (size_t dof = 0; dof < dofs_; ++dof) {
      input_->current_position[dof] = measured_position[dof];
      input_->current_velocity[dof] = 0.0;
      input_->current_acceleration[dof] = 0.0;
      input_->target_position[dof] = measured_position[dof];
      input_->target_velocity[dof] = 0.0;
      input_->target_acceleration[dof] = 0.0;
      target_position_[dof] = measured_position[dof];
    }
    input_set_ = true;
  } else {
    // Subsequent cycles: measured q + previous commanded vel/accel for continuity.
    for (size_t dof = 0; dof < dofs_; ++dof) {
      input_->current_position[dof] = measured_position[dof];
      input_->current_velocity[dof] = output_->new_velocity[dof];
      input_->current_acceleration[dof] = output_->new_acceleration[dof];
    }
  }

  for (size_t dof = 0; dof < dofs_; ++dof) {
    input_->target_position[dof] = target_position[dof];
    input_->target_velocity[dof] = 0.0;
    input_->target_acceleration[dof] = 0.0;
    target_position_[dof] = target_position[dof];
  }

  const auto result = otg_->update(*input_, *output_);
  if (result != ruckig::Result::Working && result != ruckig::Result::Finished) {
    for (size_t dof = 0; dof < dofs_; ++dof) {
      output_velocity[dof] = 0.0;
    }
    return false;
  }

  for (size_t dof = 0; dof < dofs_; ++dof) {
    output_velocity[dof] = output_->new_velocity[dof];
  }
  // Intentionally no pass_to_input: next cycle re-seeds from measured q +
  // this cycle's commanded vel/accel (vr-teleop pattern).
  return true;
}

void RuckigAdapter::extrapolate_constant_accel(double miss_dt)
{
  // Match FCI's constant-acceleration gap fill (see franky / libfranka
  // "under the hood"). Only valid after the OTG has been seeded.
  if (!input_set_ || !input_ || dofs_ == 0 || miss_dt <= 0.0) {
    return;
  }
  for (size_t dof = 0; dof < dofs_; ++dof) {
    const double accel = input_->current_acceleration[dof];
    const double v_old = input_->current_velocity[dof];
    double v_new = v_old + accel * miss_dt;
    const double max_v = config_.max_velocity_rad_s;
    v_new = std::clamp(v_new, -max_v, max_v);

    const double pos_step = 0.5 * (v_old + v_new) * miss_dt;
    input_->current_position[dof] += pos_step;
    input_->current_velocity[dof] = v_new;
    // Acceleration held constant — FCI's missing-packet model.
  }
}

}  // namespace manipulation_position_controllers::joint_space
