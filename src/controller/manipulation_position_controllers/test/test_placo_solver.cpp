// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// Unit tests for PlacoKinematicSolver.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "manipulation_position_controllers/task_space/kinematics/differential_ik/placo_solver.hpp"

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

double pos_distance(const std::array<double, 7> & a, const std::array<double, 7> & b)
{
  const double dx = a[0] - b[0];
  const double dy = a[1] - b[1];
  const double dz = a[2] - b[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}
}  // namespace

TEST(PlacoSolver, ConfigureSucceedsWithValidFrames)
{
  ts::PlacoKinematicSolver solver;
  ts::SolverConfig config;
  EXPECT_TRUE(solver.configure(kUrdf, kJoints, "base_link", "tip_link", config));
}

TEST(PlacoSolver, SolveTowardCurrentPoseHasZeroError)
{
  ts::PlacoKinematicSolver solver;
  ts::SolverConfig config;
  config.posture_weight = 0.0;
  config.damping = 0.0;
  ASSERT_TRUE(solver.configure(kUrdf, kJoints, "base_link", "tip_link", config));

  const std::vector<double> q = {0.2, -0.3, 0.5};
  solver.reset(q);

  std::array<double, 7> pose{};
  ASSERT_TRUE(solver.compute_tip_pose(q, pose));

  const ts::SolveResult result = solver.solve(q, pose, 0.002);
  EXPECT_TRUE(result.success);
  EXPECT_LT(result.task_position_error_norm, 1e-3);
}

TEST(PlacoSolver, SolveMovesTipTowardTarget)
{
  ts::PlacoKinematicSolver solver;
  ts::SolverConfig config;
  config.posture_weight = 0.0;
  ASSERT_TRUE(solver.configure(kUrdf, kJoints, "base_link", "tip_link", config));

  const std::vector<double> q = {0.2, -0.3, 0.5};
  solver.reset(q);

  std::array<double, 7> current{};
  ASSERT_TRUE(solver.compute_tip_pose(q, current));

  std::array<double, 7> target = current;
  target[0] += 0.05;

  const ts::SolveResult result = solver.solve(q, target, 0.1);
  ASSERT_TRUE(result.success);

  std::array<double, 7> after{};
  ASSERT_TRUE(solver.compute_tip_pose(result.q_command, after));

  EXPECT_LT(pos_distance(after, target), pos_distance(current, target));
}

TEST(PlacoSolver, JointVelocityClampRespected)
{
  ts::PlacoKinematicSolver solver;
  ts::SolverConfig config;
  config.posture_weight = 0.0;
  config.position_gain = 50.0;
  config.max_joint_velocity_rad_s = 0.5;
  ASSERT_TRUE(solver.configure(kUrdf, kJoints, "base_link", "tip_link", config));

  const std::vector<double> q = {0.0, 0.0, 0.0};
  solver.reset(q);

  std::array<double, 7> current{};
  ASSERT_TRUE(solver.compute_tip_pose(q, current));
  std::array<double, 7> target = current;
  target[0] += 0.2;

  const double period = 0.002;
  const ts::SolveResult result = solver.solve(q, target, period);
  ASSERT_TRUE(result.success);

  for (size_t i = 0; i < kJoints.size(); ++i) {
    const double qdot = std::abs(result.joint_velocity_estimate[i]);
    EXPECT_LE(qdot, config.max_joint_velocity_rad_s + 1e-6);
  }
}

TEST(PlacoSolver, ReusesPreallocatedResultStorage)
{
  ts::PlacoKinematicSolver solver;
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
