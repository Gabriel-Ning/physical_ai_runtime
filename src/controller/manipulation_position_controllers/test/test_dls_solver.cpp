// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// Unit tests for PinocchioDlsSolver covering the behavior added on this branch:
//   - configure() validates base_frame and tip_frame against the URDF;
//   - solve() and compute_tip_pose() share a single (base_frame) pose
//     convention — verified with a non-root base_frame (non-identity oMb);
//   - one IK step drives the tip toward the target;
//   - the damped null-space posture term does not materially perturb the task.
//
// Robust invariants are used instead of hand-computed kinematics so the tests
// do not depend on exact link geometry.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "manipulation_position_controllers/task_space/kinematics/differential_ik/dls_solver.hpp"

namespace ts = manipulation_position_controllers::task_space;

namespace
{
// A 3R serial arm mounted on a fixed offset so that base_frame ("base_link")
// has a non-identity placement relative to the model root ("world"). This is
// what exercises the oMb transform in solve()/compute_tip_pose().
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

double pos_distance(const std::array<double, 7> & a, const std::array<double, 7> & b)
{
  const double dx = a[0] - b[0];
  const double dy = a[1] - b[1];
  const double dz = a[2] - b[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}
}  // namespace

TEST(DlsSolver, ConfigureSucceedsWithValidFrames)
{
  ts::PinocchioDlsSolver solver;
  ts::SolverConfig config;
  EXPECT_TRUE(solver.configure(kUrdf, kJoints, "base_link", "tip_link", config));
}

TEST(DlsSolver, ConfigureFailsOnUnknownBaseFrame)
{
  ts::PinocchioDlsSolver solver;
  ts::SolverConfig config;
  EXPECT_FALSE(solver.configure(kUrdf, kJoints, "no_such_base", "tip_link", config));
  EXPECT_FALSE(solver.last_error().empty());
}

TEST(DlsSolver, ConfigureFailsOnUnknownTipFrame)
{
  ts::PinocchioDlsSolver solver;
  ts::SolverConfig config;
  EXPECT_FALSE(solver.configure(kUrdf, kJoints, "base_link", "no_such_tip", config));
}

// Frame-consistency invariant: solving toward the pose reported by
// compute_tip_pose() must produce ~zero task error. If solve() and
// compute_tip_pose() disagreed on the reference frame (e.g. one used the model
// root and the other base_link), the non-identity oMb would make this error
// large.
TEST(DlsSolver, SolveTowardCurrentPoseHasZeroError)
{
  ts::PinocchioDlsSolver solver;
  ts::SolverConfig config;
  config.posture_weight = 0.0;  // isolate the task term
  ASSERT_TRUE(solver.configure(kUrdf, kJoints, "base_link", "tip_link", config));

  const std::vector<double> q = {0.2, -0.3, 0.5};
  solver.reset(q);

  std::array<double, 7> pose{};
  ASSERT_TRUE(solver.compute_tip_pose(q, pose));

  const ts::SolveResult result = solver.solve(q, pose, 0.002);
  EXPECT_TRUE(result.success);
  EXPECT_LT(result.task_position_error_norm, 1e-6);
  EXPECT_LT(result.task_orientation_error_norm, 1e-6);
}

// One IK step must move the tip closer to a perturbed target.
TEST(DlsSolver, SolveMovesTipTowardTarget)
{
  ts::PinocchioDlsSolver solver;
  ts::SolverConfig config;
  config.posture_weight = 0.0;
  ASSERT_TRUE(solver.configure(kUrdf, kJoints, "base_link", "tip_link", config));

  const std::vector<double> q = {0.2, -0.3, 0.5};
  solver.reset(q);

  std::array<double, 7> current{};
  ASSERT_TRUE(solver.compute_tip_pose(q, current));

  std::array<double, 7> target = current;
  target[0] += 0.05;  // shift +x in base_frame (within reach)

  const ts::SolveResult result = solver.solve(q, target, 0.1);
  ASSERT_TRUE(result.success);

  std::array<double, 7> after{};
  ASSERT_TRUE(solver.compute_tip_pose(result.q_command, after));

  EXPECT_LT(pos_distance(after, target), pos_distance(current, target));
}

// Damped null-space posture term must not materially perturb the task: with a
// zero task error (target == current pose) and posture enabled, the tip should
// barely move after one step. Bounds the leakage contract documented in
// dls_solver.hpp / ISSUES.md (MPC-104).
TEST(DlsSolver, PostureDoesNotMateriallyPerturbTask)
{
  ts::PinocchioDlsSolver solver;
  ts::SolverConfig config;
  config.posture_weight = 1.0e-3;
  ASSERT_TRUE(solver.configure(kUrdf, kJoints, "base_link", "tip_link", config));

  // Seed neutral away from the evaluated configuration so posture has a nonzero
  // pull to project.
  const std::vector<double> q_neutral = {0.0, 0.0, 0.0};
  solver.reset(q_neutral);

  const std::vector<double> q = {0.4, -0.5, 0.6};
  std::array<double, 7> current{};
  ASSERT_TRUE(solver.compute_tip_pose(q, current));

  const ts::SolveResult result = solver.solve(q, current, 0.002);
  ASSERT_TRUE(result.success);

  std::array<double, 7> after{};
  ASSERT_TRUE(solver.compute_tip_pose(result.q_command, after));

  EXPECT_LT(pos_distance(after, current), 1.0e-3);
}

TEST(DlsSolver, ReusesPreallocatedResultStorage)
{
  ts::PinocchioDlsSolver solver;
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
