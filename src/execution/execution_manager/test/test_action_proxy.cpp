#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/parallel_gripper_command.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <execution_manager_interfaces/msg/authority_status.hpp>
#include <execution_manager_interfaces/msg/resource_claim.hpp>
#include <execution_manager_interfaces/srv/claim_control.hpp>
#include <execution_manager_interfaces/srv/release_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "execution_manager/node.hpp"

namespace {

using namespace std::chrono_literals;
using FollowTrajectory = control_msgs::action::FollowJointTrajectory;
using GripperCommand = control_msgs::action::ParallelGripperCommand;
using ArmGoalHandle = rclcpp_action::ServerGoalHandle<FollowTrajectory>;
using GripperGoalHandle = rclcpp_action::ServerGoalHandle<GripperCommand>;

template <typename FutureT>
bool wait_ready(FutureT &future, const std::chrono::milliseconds timeout) {
  return future.wait_for(timeout) == std::future_status::ready;
}

class ActionProxyFixture : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    support_ = std::make_shared<rclcpp::Node>("em_action_proxy_support");
    list_service_ =
        support_->create_service<controller_manager_msgs::srv::ListControllers>(
            "/controller_manager/list_controllers",
            [](const std::shared_ptr<
                   controller_manager_msgs::srv::ListControllers::Request>,
               std::shared_ptr<
                   controller_manager_msgs::srv::ListControllers::Response>
                   response) { response->controller.clear(); });
    switch_service_ =
        support_
            ->create_service<controller_manager_msgs::srv::SwitchController>(
                "/controller_manager/switch_controller",
                [](const std::shared_ptr<
                       controller_manager_msgs::srv::SwitchController::Request>,
                   std::shared_ptr<
                       controller_manager_msgs::srv::SwitchController::Response>
                       response) { response->ok = true; });

    arm_server_ = rclcpp_action::create_server<FollowTrajectory>(
        support_, "/execution/arm/follow_joint_trajectory",
        [](const auto &, const auto &) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const auto &) { return rclcpp_action::CancelResponse::ACCEPT; },
        [](const std::shared_ptr<ArmGoalHandle> goal) {
          std::thread([goal]() {
            const auto deadline = std::chrono::steady_clock::now() + 800ms;
            while (std::chrono::steady_clock::now() < deadline) {
              if (goal->is_canceling()) {
                auto result = std::make_shared<FollowTrajectory::Result>();
                goal->canceled(result);
                return;
              }
              std::this_thread::sleep_for(10ms);
            }
            auto result = std::make_shared<FollowTrajectory::Result>();
            result->error_code = FollowTrajectory::Result::SUCCESSFUL;
            goal->succeed(result);
          }).detach();
        });
    gripper_server_ = rclcpp_action::create_server<GripperCommand>(
        support_, "/execution/gripper/gripper_command",
        [](const auto &, const auto &) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const auto &) { return rclcpp_action::CancelResponse::ACCEPT; },
        [](const std::shared_ptr<GripperGoalHandle> goal) {
          std::thread([goal]() {
            const auto deadline = std::chrono::steady_clock::now() + 300ms;
            while (std::chrono::steady_clock::now() < deadline) {
              if (goal->is_canceling()) {
                auto result = std::make_shared<GripperCommand::Result>();
                goal->canceled(result);
                return;
              }
              std::this_thread::sleep_for(10ms);
            }
            auto result = std::make_shared<GripperCommand::Result>();
            result->reached_goal = true;
            goal->succeed(result);
          }).detach();
        });

    support_executor_ =
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
            rclcpp::ExecutorOptions(), 2);
    support_executor_->add_node(support_);
    support_thread_ = std::thread([this]() { support_executor_->spin(); });

    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {{"profile", std::string(WORKSPACE_ROOT) +
                         "/src/execution/execution_manager/config/templates/"
                         "execution_manager_profile.template.yaml"}});
    manager_ =
        std::make_shared<execution_manager::ExecutionManagerNode>(options);
    client_ = std::make_shared<rclcpp::Node>("em_action_proxy_client");
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions(), 6);
    executor_->add_node(manager_);
    executor_->add_node(client_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });

    claim_client_ =
        client_->create_client<execution_manager_interfaces::srv::ClaimControl>(
            "/execution_manager/claim");
    release_client_ =
        client_
            ->create_client<execution_manager_interfaces::srv::ReleaseControl>(
                "/execution_manager/release");
    arm_client_ = rclcpp_action::create_client<FollowTrajectory>(
        client_, "/action_sources/planner/arm/follow_joint_trajectory");
    gripper_client_ = rclcpp_action::create_client<GripperCommand>(
        client_, "/action_sources/planner/gripper/gripper_command");
  }

  void TearDown() override {
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    executor_->remove_node(client_);
    executor_->remove_node(manager_);
    client_.reset();
    manager_.reset();
    support_executor_->cancel();
    if (support_thread_.joinable()) {
      support_thread_.join();
    }
    support_executor_->remove_node(support_);
    support_.reset();
  }

  std::string claim_planner() {
    EXPECT_TRUE(claim_client_->wait_for_service(5s));
    auto request = std::make_shared<
        execution_manager_interfaces::srv::ClaimControl::Request>();
    request->source_role = 3; // PLANNER
    request->source_instance = "Planner";
    request->preempt = false;
    execution_manager_interfaces::msg::ResourceClaim arm;
    arm.resource = "arm";
    arm.command_contract = "joint_trajectory";
    request->resources.push_back(arm);
    execution_manager_interfaces::msg::ResourceClaim gripper;
    gripper.resource = "gripper";
    gripper.command_contract = "gripper_command";
    request->resources.push_back(gripper);
    auto future = claim_client_->async_send_request(request);
    if (!wait_ready(future, 10s)) {
      ADD_FAILURE() << "claim service timed out";
      return {};
    }
    const auto response = future.get();
    EXPECT_TRUE(response->success) << response->message;
    return response->lease_id;
  }

  void release(const std::string &lease_id) {
    auto request = std::make_shared<
        execution_manager_interfaces::srv::ReleaseControl::Request>();
    request->lease_id = lease_id;
    auto future = release_client_->async_send_request(request);
    ASSERT_TRUE(wait_ready(future, 5s));
    EXPECT_TRUE(future.get()->success);
  }

  rclcpp::Node::SharedPtr support_;
  rclcpp::Service<controller_manager_msgs::srv::ListControllers>::SharedPtr
      list_service_;
  rclcpp::Service<controller_manager_msgs::srv::SwitchController>::SharedPtr
      switch_service_;
  rclcpp_action::Server<FollowTrajectory>::SharedPtr arm_server_;
  rclcpp_action::Server<GripperCommand>::SharedPtr gripper_server_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> support_executor_;
  std::thread support_thread_;

  std::shared_ptr<execution_manager::ExecutionManagerNode> manager_;
  rclcpp::Node::SharedPtr client_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread executor_thread_;
  rclcpp::Client<execution_manager_interfaces::srv::ClaimControl>::SharedPtr
      claim_client_;
  rclcpp::Client<execution_manager_interfaces::srv::ReleaseControl>::SharedPtr
      release_client_;
  rclcpp_action::Client<FollowTrajectory>::SharedPtr arm_client_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
};

TEST_F(ActionProxyFixture, ScopedLeaseSurvivesShorterGripperAction) {
  const auto lease_id = claim_planner();
  ASSERT_FALSE(lease_id.empty());
  ASSERT_TRUE(arm_client_->wait_for_action_server(5s));
  ASSERT_TRUE(gripper_client_->wait_for_action_server(5s));

  FollowTrajectory::Goal arm_goal;
  arm_goal.trajectory.joint_names = {"joint1", "joint2"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {0.0, 0.0};
  point.time_from_start.sec = 1;
  arm_goal.trajectory.points.push_back(point);
  GripperCommand::Goal gripper_goal;
  gripper_goal.command.name = {"gripper_joint"};
  gripper_goal.command.position = {0.04};

  auto arm_send = arm_client_->async_send_goal(arm_goal);
  auto gripper_send = gripper_client_->async_send_goal(gripper_goal);
  ASSERT_TRUE(wait_ready(arm_send, 5s));
  ASSERT_TRUE(wait_ready(gripper_send, 5s));
  auto arm_handle = arm_send.get();
  auto gripper_handle = gripper_send.get();
  ASSERT_TRUE(arm_handle);
  ASSERT_TRUE(gripper_handle);

  auto gripper_result = gripper_client_->async_get_result(gripper_handle);
  ASSERT_TRUE(wait_ready(gripper_result, 5s));
  EXPECT_EQ(gripper_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);

  auto arm_result = arm_client_->async_get_result(arm_handle);
  ASSERT_TRUE(wait_ready(arm_result, 5s));
  EXPECT_EQ(arm_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  release(lease_id);
}

TEST_F(ActionProxyFixture, GripperCancelPropagatesOnceAndManagerStaysAlive) {
  const auto lease_id = claim_planner();
  ASSERT_FALSE(lease_id.empty());
  ASSERT_TRUE(gripper_client_->wait_for_action_server(5s));
  GripperCommand::Goal goal;
  goal.command.name = {"gripper_joint"};
  goal.command.position = {0.02};
  auto send = gripper_client_->async_send_goal(goal);
  ASSERT_TRUE(wait_ready(send, 5s));
  auto handle = send.get();
  ASSERT_TRUE(handle);
  auto cancel = gripper_client_->async_cancel_goal(handle);
  ASSERT_TRUE(wait_ready(cancel, 5s));
  auto result = gripper_client_->async_get_result(handle);
  ASSERT_TRUE(wait_ready(result, 5s));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_TRUE(claim_client_->service_is_ready());
  release(lease_id);
}

TEST_F(ActionProxyFixture, ArmCancelPropagatesAndManagerStaysAlive) {
  const auto lease_id = claim_planner();
  ASSERT_FALSE(lease_id.empty());
  ASSERT_TRUE(arm_client_->wait_for_action_server(5s));
  FollowTrajectory::Goal goal;
  goal.trajectory.joint_names = {"joint1", "joint2"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {0.0, 0.0};
  point.time_from_start.sec = 1;
  goal.trajectory.points.push_back(point);
  auto send = arm_client_->async_send_goal(goal);
  ASSERT_TRUE(wait_ready(send, 5s));
  auto handle = send.get();
  ASSERT_TRUE(handle);
  auto cancel = arm_client_->async_cancel_goal(handle);
  ASSERT_TRUE(wait_ready(cancel, 5s));
  auto result = arm_client_->async_get_result(handle);
  ASSERT_TRUE(wait_ready(result, 5s));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_TRUE(claim_client_->service_is_ready());
  release(lease_id);
}

} // namespace
