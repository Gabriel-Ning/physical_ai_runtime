// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace
{

// Mirrors JointSpaceImpedanceController impedance + clamps (no ROS).
double compute_impedance_torque(
  double q_ref, double q_meas, double qdot_ref, double dq_filt,
  double kp, double kd, double max_torque,
  double prev_tau, double max_tau_change)
{
  const double pos_err = q_ref - q_meas;
  const double vel_err = qdot_ref - dq_filt;
  double tau = kp * pos_err + kd * vel_err;
  tau = std::clamp(tau, prev_tau - max_tau_change, prev_tau + max_tau_change);
  return std::clamp(tau, -max_torque, max_torque);
}

double filter_velocity(double prev_filt, double qdot_meas, double alpha)
{
  return (1.0 - alpha) * prev_filt + alpha * qdot_meas;
}

}  // namespace

TEST(JointSpaceImpedanceControllerTest, ZeroErrorProducesZeroTorque)
{
  const double tau = compute_impedance_torque(1.0, 1.0, 0.0, 0.0, 500.0, 30.0, 87.0, 0.0, 1000.0);
  EXPECT_DOUBLE_EQ(tau, 0.0);
}

TEST(JointSpaceImpedanceControllerTest, PositionErrorGeneratesProportionalTorque)
{
  const double tau = compute_impedance_torque(1.1, 1.0, 0.0, 0.0, 500.0, 30.0, 87.0, 0.0, 1000.0);
  EXPECT_NEAR(tau, 50.0, 1e-6);
}

TEST(JointSpaceImpedanceControllerTest, VelocityErrorDampsMotion)
{
  // Official-style damping on filtered measured velocity when qdot_ref = 0.
  const double tau = compute_impedance_torque(1.0, 1.0, 0.0, 2.0, 500.0, 30.0, 87.0, 0.0, 1000.0);
  EXPECT_DOUBLE_EQ(tau, -60.0);
}

TEST(JointSpaceImpedanceControllerTest, TorqueIsClampedToMaxLimits)
{
  const double tau = compute_impedance_torque(2.0, 1.0, 0.0, 0.0, 500.0, 30.0, 87.0, 0.0, 1000.0);
  EXPECT_DOUBLE_EQ(tau, 87.0);

  const double tau_neg = compute_impedance_torque(0.0, 1.0, 0.0, 0.0, 500.0, 30.0, 87.0, 0.0, 1000.0);
  EXPECT_DOUBLE_EQ(tau_neg, -87.0);
}

TEST(JointSpaceImpedanceControllerTest, TorqueRateLimitPreventsStep)
{
  // Raw would be 50 Nm, but rate limit only allows +5 Nm from prev.
  const double tau = compute_impedance_torque(1.1, 1.0, 0.0, 0.0, 500.0, 30.0, 87.0, 0.0, 5.0);
  EXPECT_DOUBLE_EQ(tau, 5.0);
}

TEST(JointSpaceImpedanceControllerTest, VelocityFilterMatchesMeasurementWeight)
{
  // α=0.99 → nearly follow measurement (near pass-through).
  const double filt = filter_velocity(0.0, 1.0, 0.99);
  EXPECT_NEAR(filt, 0.99, 1e-12);
  EXPECT_DOUBLE_EQ(filter_velocity(0.5, 1.0, 0.0), 0.5);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
