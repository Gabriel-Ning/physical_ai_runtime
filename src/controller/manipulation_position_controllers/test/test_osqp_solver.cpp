// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// Unit tests for OsqpIkSolver, focused on the timestep-integration fix: the QP
// decision dq is a position increment, so the task target must be v_desired·dt
// (like DLS qdot·dt). Without the dt factor the per-cycle step is ~1/dt larger
// than DLS, which produced a stationary-target period-2 limit cycle. The key
// regression test asserts OSQP and DLS produce comparable single-step
// increments for identical inputs.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "manipulation_position_controllers/task_space/kinematics/differential_ik/osqp_solver.hpp"
#include "manipulation_position_controllers/task_space/kinematics/differential_ik/dls_solver.hpp"

namespace ts = manipulation_position_controllers::task_space;

namespace
{
const char * kUrdf = R"(<?xml version="1.0"?>
<robot name="test_arm">
  <link name="world"/>
  <joint name="mount" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin xyz="0.1 0.2 0.3" rpy="0 0 0"/>
  </joint>
  <link name="base_link">
    <inertial><mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.0" upper="3.0" effort="10" velocity="2"/>
  </joint>
  <link name="link1">
    <inertial><mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="j2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0.3 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.0" upper="3.0" effort="10" velocity="2"/>
  </joint>
  <link name="link2">
    <inertial><mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="j3" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="0.3 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.0" upper="3.0" effort="10" velocity="2"/>
  </joint>
  <link name="link3">
    <inertial><mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="to_tip" type="fixed">
    <parent link="link3"/><child link="tip_link"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/>
  </joint>
  <link name="tip_link">
    <inertial><mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
</robot>)";

const std::vector<std::string> kJoints = {"j1", "j2", "j3"};
}  // namespace

TEST(OsqpSolver, ConfigureSucceedsWithValidFrames)
{
  ts::OsqpIkSolver solver;
  ts::SolverConfig config;
  EXPECT_TRUE(solver.configure(kUrdf, kJoints, "base_link", "tip_link", config));
}

TEST(OsqpSolver, ConfigureFailsOnUnknownBaseFrame)
{
  ts::OsqpIkSolver solver;
  ts::SolverConfig config;
  EXPECT_FALSE(solver.configure(kUrdf, kJoints, "no_such_base", "tip_link", config));
}

TEST(OsqpSolver, SolveTowardCurrentPoseHasZeroError)
{
  ts::OsqpIkSolver solver;
  ts::SolverConfig config;
  config.posture_weight = 0.0;
  ASSERT_TRUE(solver.configure(kUrdf, kJoints, "base_link", "tip_link", config));

  const std::vector<double> q = {0.2, -0.3, 0.5};
  solver.reset(q);

  std::array<double, 7> pose{};
  ASSERT_TRUE(solver.compute_tip_pose(q, pose));

  const ts::SolveResult result = solver.solve(q, pose, 0.002);
  EXPECT_TRUE(result.success);
  EXPECT_LT(result.task_position_error_norm, 1e-6);
}

// Timestep-integration regression: for an unclamped small error, OSQP's
// single-step joint increment must match DLS's (both ≈ dt·J⁺·gain·error).
// Before the dt fix, OSQP's increment was ~1/dt (≈500×) larger, which is what
// drove the stationary-target limit cycle.
TEST(OsqpSolver, SingleStepIncrementMatchesDls)
{
  ts::SolverConfig config;
  config.posture_weight = 0.0;
  config.position_gain = 4.0;
  config.orientation_gain = 1.0;
  config.damping = 0.05;
  config.max_joint_velocity_rad_s = 5.0;   // high so the step is not clamped

  ts::OsqpIkSolver osqp;
  ts::PinocchioDlsSolver dls;
  ASSERT_TRUE(osqp.configure(kUrdf, kJoints, "base_link", "tip_link", config));
  ASSERT_TRUE(dls.configure(kUrdf, kJoints, "base_link", "tip_link", config));

  const std::vector<double> q = {0.2, -0.3, 0.5};
  osqp.reset(q);
  dls.reset(q);

  // Small target offset → desired Cartesian velocity well below the clamp.
  std::array<double, 7> target{};
  ASSERT_TRUE(dls.compute_tip_pose(q, target));
  target[0] += 0.001;

  const double dt = 0.002;
  const ts::SolveResult r_osqp = osqp.solve(q, target, dt);
  const ts::SolveResult r_dls = dls.solve(q, target, dt);
  ASSERT_TRUE(r_osqp.success);
  ASSERT_TRUE(r_dls.success);

  for (size_t i = 0; i < kJoints.size(); ++i) {
    const double dq_osqp = r_osqp.q_command[i] - q[i];
    const double dq_dls = r_dls.q_command[i] - q[i];
    // Same order of magnitude and same sign (not ~500× apart).
    EXPECT_NEAR(dq_osqp, dq_dls, 5e-4)
      << "joint " << i << " dq_osqp=" << dq_osqp << " dq_dls=" << dq_dls;
  }
}

// Stationary target must not sustain a limit cycle: integrating the command
// open-loop (as TSKPC does) toward a fixed reachable target converges instead
// of oscillating at amplitude ≈ v_max·dt.
TEST(OsqpSolver, StationaryTargetConverges)
{
  ts::SolverConfig config;
  config.posture_weight = 0.0;
  config.position_gain = 4.0;
  config.max_joint_velocity_rad_s = 1.0;
  ts::OsqpIkSolver solver;
  ASSERT_TRUE(solver.configure(kUrdf, kJoints, "base_link", "tip_link", config));

  const std::vector<double> q0 = {0.2, -0.3, 0.5};
  solver.reset(q0);

  std::array<double, 7> target{};
  ASSERT_TRUE(solver.compute_tip_pose(q0, target));
  target[0] += 0.01;  // small reachable offset

  std::vector<double> command = q0;
  double last_window_range = 0.0;
  std::vector<double> recent;  // track one joint's last-window command
  const int cycles = 800;
  const double dt = 0.002;
  for (int k = 0; k < cycles; ++k) {
    const ts::SolveResult r = solver.solve(command, target, dt);
    ASSERT_TRUE(r.success);
    command = r.q_command;  // open-loop integration (seed from command)
    if (k >= cycles - 100) {
      recent.push_back(command[0]);
    }
  }
  double lo = recent.front(), hi = recent.front();
  for (double v : recent) { lo = std::min(lo, v); hi = std::max(hi, v); }
  last_window_range = hi - lo;

  // A sustained limit cycle would hold range ≈ v_max·dt = 0.002 rad. Converged
  // motion settles far below that.
  EXPECT_LT(last_window_range, 5e-4) << "joint 0 last-100 range " << last_window_range;
}

TEST(OsqpSolver, ReusesPreallocatedResultStorage)
{
  ts::OsqpIkSolver solver;
  ts::SolverConfig config;
  ASSERT_TRUE(solver.configure(kUrdf, kJoints, "base_link", "tip_link", config));
  const std::vector<double> q = {0.2, -0.3, 0.5};
  std::array<double, 7> pose{};
  ASSERT_TRUE(solver.compute_tip_pose(q, pose));

  const auto & first = solver.solve(q, pose, 0.002);
  const auto * command_data = first.q_command.data();
  const auto * velocity_data = first.joint_velocity_estimate.data();
  const auto & second = solver.solve(q, pose, 0.002);
  EXPECT_EQ(&first, &second);
  EXPECT_EQ(command_data, second.q_command.data());
  EXPECT_EQ(velocity_data, second.joint_velocity_estimate.data());
}
