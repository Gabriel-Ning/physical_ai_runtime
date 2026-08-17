// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "manipulation_position_controllers/common/limits.hpp"
#include "manipulation_position_controllers/common/parameter_validation.hpp"
#include "manipulation_position_controllers/task_space/kinematics/differential_ik/solver_interface.hpp"

namespace
{

// Mirrors TaskSpaceJointImpedanceController impedance + clamps (no ROS / IK).
double compute_impedance_torque(
  double q_des, double q_meas, double qdot_des, double dq_filt,
  double kp, double kd, double max_torque,
  double prev_tau, double max_tau_change)
{
  const double pos_err = q_des - q_meas;
  const double vel_err = qdot_des - dq_filt;
  double tau = kp * pos_err + kd * vel_err;
  tau = std::clamp(tau, prev_tau - max_tau_change, prev_tau + max_tau_change);
  return std::clamp(tau, -max_torque, max_torque);
}

double filter_velocity(double prev_filt, double qdot_meas, double alpha)
{
  // α = measurement weight (same as controllers).
  return (1.0 - alpha) * prev_filt + alpha * qdot_meas;
}

}  // namespace

TEST(TaskSpaceJointImpedanceControllerTest, ZeroErrorProducesZeroTorque)
{
  const double tau = compute_impedance_torque(
    1.0, 1.0, 0.0, 0.0, 600.0, 30.0, 87.0, 0.0, 1000.0);
  EXPECT_DOUBLE_EQ(tau, 0.0);
}

TEST(TaskSpaceJointImpedanceControllerTest, PositionErrorGeneratesProportionalTorque)
{
  const double tau = compute_impedance_torque(
    1.1, 1.0, 0.0, 0.0, 600.0, 30.0, 87.0, 0.0, 1000.0);
  EXPECT_NEAR(tau, 60.0, 1e-6);
}

TEST(TaskSpaceJointImpedanceControllerTest, TorqueIsClampedToMaxLimits)
{
  const double tau = compute_impedance_torque(
    2.0, 1.0, 0.0, 0.0, 600.0, 30.0, 87.0, 0.0, 1000.0);
  EXPECT_DOUBLE_EQ(tau, 87.0);
}

TEST(TaskSpaceJointImpedanceControllerTest, TorqueRateLimitPreventsStep)
{
  const double tau = compute_impedance_torque(
    1.1, 1.0, 0.0, 0.0, 600.0, 30.0, 87.0, 0.0, 5.0);
  EXPECT_DOUBLE_EQ(tau, 5.0);
}

TEST(TaskSpaceJointImpedanceControllerTest, VelocityFilterMeasurementWeight)
{
  // α=0.99 → nearly follow measurement (near pass-through).
  EXPECT_NEAR(filter_velocity(0.0, 1.0, 0.99), 0.99, 1e-12);
  // α=0.0 → hold previous (full smooth / ignore measurement).
  EXPECT_DOUBLE_EQ(filter_velocity(0.5, 1.0, 0.0), 0.5);
}

TEST(TaskSpaceJointImpedanceControllerTest, LimitVectorsRejectWrongSize)
{
  const std::vector<std::string> joints{"j1", "j2"};
  EXPECT_TRUE(
    manipulation_position_controllers::common::validate_limit_vectors(
      joints, {}, {}));
  EXPECT_FALSE(
    manipulation_position_controllers::common::validate_limit_vectors(
      joints, {-1.0}, {1.0}));
  EXPECT_TRUE(
    manipulation_position_controllers::common::validate_limit_vectors(
      joints, {-1.0, -2.0}, {1.0, 2.0}));
}

TEST(TaskSpaceJointImpedanceControllerTest, StaleHoldPinsReferenceToMeasured)
{
  // Soft stale contract: q_des ← q_meas, qdot_des ← 0 → zero impedance torque.
  const double q_meas = 0.4;
  const double q_des = q_meas;
  const double qdot_des = 0.0;
  const double tau = compute_impedance_torque(
    q_des, q_meas, qdot_des, 0.0, 600.0, 30.0, 87.0, 0.0, 1000.0);
  EXPECT_DOUBLE_EQ(tau, 0.0);
}

TEST(TaskSpaceJointImpedanceControllerTest, NumericValidationRejectsNanAndInfinity)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(manipulation_position_controllers::common::is_positive_finite(nan));
  EXPECT_FALSE(manipulation_position_controllers::common::is_positive_finite(inf));
  EXPECT_FALSE(manipulation_position_controllers::common::is_unit_interval_finite(nan));
  EXPECT_FALSE(manipulation_position_controllers::common::all_positive_finite({1.0, inf}));
  EXPECT_TRUE(manipulation_position_controllers::common::all_unique_nonempty({"j1", "j2"}));
  EXPECT_FALSE(manipulation_position_controllers::common::all_unique_nonempty({"j1", "j1"}));
  EXPECT_FALSE(manipulation_position_controllers::common::all_unique_nonempty({"j1", ""}));
}

TEST(TaskSpaceJointImpedanceControllerTest, SolverConfigRejectsUnknownBackend)
{
  manipulation_position_controllers::task_space::SolverConfig config;
  config.backend = "osqp_typo";
  std::string error;
  EXPECT_FALSE(
    manipulation_position_controllers::task_space::validate_solver_config(config, error));
  EXPECT_FALSE(error.empty());
}

TEST(TaskSpaceJointImpedanceControllerTest, SolverConfigRejectsInvalidNumericValues)
{
  manipulation_position_controllers::task_space::SolverConfig config;
  config.backend = "osqp";
  std::string error;
  config.osqp_rho = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
    manipulation_position_controllers::task_space::validate_solver_config(config, error));

  config = manipulation_position_controllers::task_space::SolverConfig{};
  config.backend = "placo";
  config.joint_motion_weights = {1.0, 0.0};
  EXPECT_FALSE(
    manipulation_position_controllers::task_space::validate_solver_config(config, error));
}

TEST(TaskSpaceJointImpedanceControllerTest, SolverConfigAcceptsAllSupportedBackends)
{
  for (const std::string backend : {"pinocchio_dls", "osqp", "placo"}) {
    manipulation_position_controllers::task_space::SolverConfig config;
    config.backend = backend;
    std::string error;
    EXPECT_TRUE(
      manipulation_position_controllers::task_space::validate_solver_config(config, error))
      << backend << ": " << error;
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
