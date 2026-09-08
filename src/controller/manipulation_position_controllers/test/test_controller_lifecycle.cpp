// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "manipulation_position_controllers/joint_space/dynamics/joint_space_impedance_controller.hpp"
#include "manipulation_position_controllers/joint_space/kinematics/joint_space_position_controller.hpp"
#include "manipulation_position_controllers/task_space/kinematics/task_space_kinematic_position_controller.hpp"

namespace mpc = manipulation_position_controllers;
namespace jsd = manipulation_position_controllers::joint_space::dynamics;

class ControllerLifecycleTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(ControllerLifecycleTest, JointImpedanceDeactivateWritesZeroEffort)
{
  auto controller = std::make_unique<jsd::JointSpaceImpedanceController>();
  ASSERT_EQ(
    controller->init(
      "joint_impedance_lifecycle_test", "", 500, "", rclcpp::NodeOptions()),
    controller_interface::return_type::OK);

  auto node = controller->get_node();
  node->set_parameter(rclcpp::Parameter("joints", std::vector<std::string>{"j1", "j2"}));
  node->set_parameter(rclcpp::Parameter("input_topic", "/test/joint_reference"));
  node->set_parameter(rclcpp::Parameter("kp_stiffness", std::vector<double>{10.0, 10.0}));
  node->set_parameter(rclcpp::Parameter("kd_damping", std::vector<double>{1.0, 1.0}));
  node->set_parameter(rclcpp::Parameter("max_torques", std::vector<double>{20.0, 20.0}));
  ASSERT_EQ(
    controller->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  std::vector<double> commands{4.0, -3.0};
  std::vector<double> positions{0.2, -0.1};
  std::vector<double> velocities{0.0, 0.0};
  std::vector<hardware_interface::CommandInterface> command_handles;
  std::vector<hardware_interface::StateInterface> state_handles;
  command_handles.reserve(2);
  state_handles.reserve(4);
  for (size_t i = 0; i < 2; ++i) {
    const std::string joint = "j" + std::to_string(i + 1);
    command_handles.emplace_back(
      joint, hardware_interface::HW_IF_EFFORT, &commands[i]);
    state_handles.emplace_back(
      joint, hardware_interface::HW_IF_POSITION, &positions[i]);
    state_handles.emplace_back(
      joint, hardware_interface::HW_IF_VELOCITY, &velocities[i]);
  }

  std::vector<hardware_interface::LoanedCommandInterface> command_loans;
  std::vector<hardware_interface::LoanedStateInterface> state_loans;
  command_loans.reserve(command_handles.size());
  state_loans.reserve(state_handles.size());
  for (auto & interface : command_handles) {
    command_loans.emplace_back(interface);
  }
  for (auto & interface : state_handles) {
    state_loans.emplace_back(interface);
  }
  controller->assign_interfaces(std::move(command_loans), std::move(state_loans));

  ASSERT_EQ(
    controller->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  velocities[0] = std::numeric_limits<double>::quiet_NaN();
  commands = {6.0, -5.0};
  EXPECT_EQ(
    controller->update(rclcpp::Time(1, 0), rclcpp::Duration::from_seconds(0.002)),
    controller_interface::return_type::ERROR);
  EXPECT_DOUBLE_EQ(commands[0], 0.0);
  EXPECT_DOUBLE_EQ(commands[1], 0.0);

  velocities[0] = 0.0;
  commands = {6.0, -5.0};
  ASSERT_EQ(
    controller->on_deactivate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  EXPECT_DOUBLE_EQ(commands[0], 0.0);
  EXPECT_DOUBLE_EQ(commands[1], 0.0);
  controller->release_interfaces();
}

TEST_F(ControllerLifecycleTest, JointControllersClaimOnlyJointStateInterfaces)
{
  auto jsic = std::make_unique<jsd::JointSpaceImpedanceController>();
  ASSERT_EQ(
    jsic->init("jsic_state_claim_test", "", 500, "", rclcpp::NodeOptions()),
    controller_interface::return_type::OK);
  auto jsic_node = jsic->get_node();
  jsic_node->set_parameter(rclcpp::Parameter("joints", std::vector<std::string>{"j1", "j2"}));
  jsic_node->set_parameter(rclcpp::Parameter("kp_stiffness", std::vector<double>{10.0, 10.0}));
  jsic_node->set_parameter(rclcpp::Parameter("kd_damping", std::vector<double>{1.0, 1.0}));
  jsic_node->set_parameter(rclcpp::Parameter("max_torques", std::vector<double>{20.0, 20.0}));
  ASSERT_EQ(
    jsic->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  const auto jsic_state = jsic->state_interface_configuration();
  ASSERT_EQ(jsic_state.names.size(), 4u);
  EXPECT_EQ(jsic_state.names[0], "j1/position");
  EXPECT_EQ(jsic_state.names[1], "j1/velocity");
  EXPECT_EQ(jsic_state.names[2], "j2/position");
  EXPECT_EQ(jsic_state.names[3], "j2/velocity");
  EXPECT_FALSE(jsic_node->has_parameter("robot_time_interface"));

  auto jspc = std::make_unique<mpc::JointSpacePositionController>();
  ASSERT_EQ(
    jspc->init("jspc_state_claim_test", "", 500, "", rclcpp::NodeOptions()),
    controller_interface::return_type::OK);
  auto jspc_node = jspc->get_node();
  jspc_node->set_parameter(rclcpp::Parameter("joints", std::vector<std::string>{"j1", "j2"}));
  ASSERT_EQ(
    jspc->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  const auto jspc_state = jspc->state_interface_configuration();
  ASSERT_EQ(jspc_state.names.size(), 2u);
  EXPECT_EQ(jspc_state.names[0], "j1/position");
  EXPECT_EQ(jspc_state.names[1], "j2/position");
  EXPECT_FALSE(jspc_node->has_parameter("robot_time_interface"));
}

TEST_F(ControllerLifecycleTest, JointImpedanceRejectsNanGain)
{
  auto controller = std::make_unique<jsd::JointSpaceImpedanceController>();
  ASSERT_EQ(
    controller->init("joint_impedance_nan_test", "", 500, "", rclcpp::NodeOptions()),
    controller_interface::return_type::OK);
  auto node = controller->get_node();
  node->set_parameter(rclcpp::Parameter("joints", std::vector<std::string>{"j1"}));
  node->set_parameter(rclcpp::Parameter(
    "kp_stiffness", std::vector<double>{std::numeric_limits<double>::quiet_NaN()}));
  node->set_parameter(rclcpp::Parameter("kd_damping", std::vector<double>{1.0}));
  node->set_parameter(rclcpp::Parameter("max_torques", std::vector<double>{20.0}));
  EXPECT_EQ(
    controller->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(ControllerLifecycleTest, TaskControllerRejectsUnknownSolverBackend)
{
  auto controller = std::make_unique<mpc::TaskSpaceKinematicPositionController>();
  ASSERT_EQ(
    controller->init("task_solver_name_test", "", 500, "", rclcpp::NodeOptions()),
    controller_interface::return_type::OK);
  auto node = controller->get_node();
  node->set_parameter(rclcpp::Parameter("joints", std::vector<std::string>{"j1"}));
  node->set_parameter(rclcpp::Parameter("base_frame", "base_link"));
  node->set_parameter(rclcpp::Parameter("tip_frame", "tip_link"));
  node->set_parameter(rclcpp::Parameter("input_topic", "/test/pose_reference"));
  node->set_parameter(rclcpp::Parameter("solver.backend", "osqp_typo"));
  EXPECT_EQ(
    controller->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}
