#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <set>
#include <thread>

#include <gtest/gtest.h>

#include <controller_manager_msgs/msg/controller_state.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <execution_manager_interfaces/msg/authority_status.hpp>
#include <execution_manager_interfaces/msg/resource_authority.hpp>
#include <execution_manager_interfaces/srv/release_control.hpp>
#include <moveit_msgs/msg/cartesian_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "execution_manager/node.hpp"

namespace {

using namespace std::chrono_literals;

class ExternalSourceSingleFlightFixture : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    support_ = std::make_shared<rclcpp::Node>("em_single_flight_support");
    list_service_ =
        support_->create_service<controller_manager_msgs::srv::ListControllers>(
            "/controller_manager/list_controllers",
            [this](
                const std::shared_ptr<
                    controller_manager_msgs::srv::ListControllers::Request>,
                std::shared_ptr<
                    controller_manager_msgs::srv::ListControllers::Response>
                    response) {
              ++list_calls_;
              std::this_thread::sleep_for(250ms);
              std::lock_guard lock(controller_mutex_);
              for (const auto &name : active_controllers_) {
                controller_manager_msgs::msg::ControllerState controller;
                controller.name = name;
                controller.state = "active";
                response->controller.push_back(std::move(controller));
              }
            });
    switch_service_ =
        support_->create_service<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller",
            [this](
                const std::shared_ptr<
                    controller_manager_msgs::srv::SwitchController::Request>
                    request,
                std::shared_ptr<
                    controller_manager_msgs::srv::SwitchController::Response>
                    response) {
              std::lock_guard lock(controller_mutex_);
              for (const auto &name : request->deactivate_controllers) {
                active_controllers_.erase(name);
              }
              for (const auto &name : request->activate_controllers) {
                active_controllers_.insert(name);
              }
              response->ok = true;
            });

    support_executor_ =
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
            rclcpp::ExecutorOptions(), 2);
    support_executor_->add_node(support_);
    support_thread_ = std::thread([this]() { support_executor_->spin(); });

    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {{"profile", std::string(WORKSPACE_ROOT) +
                         "/src/execution/execution_manager/test/"
                         "external_source_single_flight.yaml"}});
    manager_ =
        std::make_shared<execution_manager::ExecutionManagerNode>(options);
    client_ = std::make_shared<rclcpp::Node>("em_single_flight_client");
    status_sub_ = client_->create_subscription<
        execution_manager_interfaces::msg::AuthorityStatus>(
        "/execution_manager/authority_status",
        rclcpp::QoS(1).reliable().transient_local(),
        [this](const execution_manager_interfaces::msg::AuthorityStatus::SharedPtr
                   message) {
          std::lock_guard lock(status_mutex_);
          status_ = *message;
        });
    dummy_arm_ = client_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/test_sources/dummy/arm", 10);
    dummy_gripper_ =
        client_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/test_sources/dummy/gripper", 10);
    quest_arm_ = client_->create_publisher<moveit_msgs::msg::CartesianTrajectory>(
        "/test_sources/quest/arm", 10);
    quest_gripper_ =
        client_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/test_sources/quest/gripper", 10);
    arm_output_sub_ =
        client_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/execution/arm/joint_reference", 10,
            [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr message) {
              if (!message->points.empty() &&
                  !message->points.back().positions.empty()) {
                arm_output_marker_ =
                    static_cast<int>(message->points.back().positions.front());
              }
            });
    clutch_ = client_->create_publisher<std_msgs::msg::Bool>(
        "/test_sources/quest/clutch", 10);
    release_client_ =
        client_->create_client<execution_manager_interfaces::srv::ReleaseControl>(
            "/execution_manager/release");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions(), 3);
    executor_->add_node(manager_);
    executor_->add_node(client_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });
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

  bool source_owns_all(const std::string &source) {
    std::lock_guard lock(status_mutex_);
    if (status_.resources.size() != 2) {
      return false;
    }
    for (const auto &resource : status_.resources) {
      if (resource.authority_state !=
              execution_manager_interfaces::msg::ResourceAuthority::OWNED ||
          resource.source_instance != source) {
        return false;
      }
    }
    return true;
  }

  bool all_unowned() {
    std::lock_guard lock(status_mutex_);
    return status_.resources.size() == 2 &&
           std::all_of(status_.resources.begin(), status_.resources.end(),
                       [](const auto &resource) {
                         return resource.authority_state ==
                                execution_manager_interfaces::msg::
                                    ResourceAuthority::UNOWNED;
                       });
  }

  std::string source_lease(const std::string &source) {
    std::lock_guard lock(status_mutex_);
    if (status_.resources.empty()) {
      return {};
    }
    std::string lease;
    for (const auto &resource : status_.resources) {
      if (resource.source_instance != source || resource.lease_id.empty()) {
        return {};
      }
      if (lease.empty()) {
        lease = resource.lease_id;
      } else if (lease != resource.lease_id) {
        return {};
      }
    }
    return lease;
  }

  template <typename Predicate>
  bool wait_until(Predicate predicate, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return predicate();
  }

  rclcpp::Node::SharedPtr support_;
  rclcpp::Service<controller_manager_msgs::srv::ListControllers>::SharedPtr
      list_service_;
  rclcpp::Service<controller_manager_msgs::srv::SwitchController>::SharedPtr
      switch_service_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> support_executor_;
  std::thread support_thread_;
  std::mutex controller_mutex_;
  std::set<std::string> active_controllers_;
  std::atomic_int list_calls_{0};

  std::shared_ptr<execution_manager::ExecutionManagerNode> manager_;
  rclcpp::Node::SharedPtr client_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread executor_thread_;
  rclcpp::Subscription<execution_manager_interfaces::msg::AuthorityStatus>::
      SharedPtr status_sub_;
  std::mutex status_mutex_;
  execution_manager_interfaces::msg::AuthorityStatus status_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      dummy_arm_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      dummy_gripper_;
  rclcpp::Publisher<moveit_msgs::msg::CartesianTrajectory>::SharedPtr quest_arm_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      quest_gripper_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      arm_output_sub_;
  std::atomic_int arm_output_marker_{0};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clutch_;
  rclcpp::Client<execution_manager_interfaces::srv::ReleaseControl>::SharedPtr
      release_client_;
};

TEST_F(ExternalSourceSingleFlightFixture,
       ConcurrentIngressDoesNotStarveControllerResponse) {
  trajectory_msgs::msg::JointTrajectory joints;
  joints.header.stamp = client_->now();
  joints.joint_names = {"joint1", "joint2"};
  dummy_arm_->publish(joints);
  joints.header.stamp = client_->now();
  joints.joint_names = {"gripper_joint"};
  dummy_gripper_->publish(joints);
  ASSERT_TRUE(wait_until([this]() { return source_owns_all("DummyPolicy"); }, 3s));

  std_msgs::msg::Bool clutch;
  clutch.data = true;
  clutch_->publish(clutch);
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < deadline) {
    trajectory_msgs::msg::JointTrajectory arm;
    arm.header.stamp = client_->now();
    arm.joint_names = {"joint1", "joint2"};
    dummy_arm_->publish(arm);
    trajectory_msgs::msg::JointTrajectory gripper;
    gripper.header.stamp = arm.header.stamp;
    gripper.joint_names = {"gripper_joint"};
    dummy_gripper_->publish(gripper);
    moveit_msgs::msg::CartesianTrajectory pose;
    pose.header.stamp = arm.header.stamp;
    quest_arm_->publish(pose);
    quest_gripper_->publish(gripper);
    std::this_thread::sleep_for(2ms);
  }

  EXPECT_TRUE(wait_until(
      [this]() { return source_owns_all("Quest3Teleop"); }, 3s));
  EXPECT_LT(list_calls_.load(), 10);

  clutch.data = false;
  clutch_->publish(clutch);
  EXPECT_TRUE(wait_until([this]() { return source_owns_all("DummyPolicy"); }, 3s));
}

TEST_F(ExternalSourceSingleFlightFixture,
       ReleasedCachedLeaseIsReacquiredOnNextCommand) {
  trajectory_msgs::msg::JointTrajectory arm;
  arm.header.stamp = client_->now();
  arm.joint_names = {"joint1", "joint2"};
  trajectory_msgs::msg::JointTrajectory gripper;
  gripper.header.stamp = arm.header.stamp;
  gripper.joint_names = {"gripper_joint"};
  dummy_arm_->publish(arm);
  dummy_gripper_->publish(gripper);
  ASSERT_TRUE(wait_until([this]() { return source_owns_all("DummyPolicy"); }, 3s));
  const auto old_lease = source_lease("DummyPolicy");
  ASSERT_FALSE(old_lease.empty());

  ASSERT_TRUE(release_client_->wait_for_service(1s));
  auto request = std::make_shared<
      execution_manager_interfaces::srv::ReleaseControl::Request>();
  request->lease_id = old_lease;
  auto future = release_client_->async_send_request(request);
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  ASSERT_TRUE(future.get()->success);

  arm.header.stamp = client_->now();
  gripper.header.stamp = arm.header.stamp;
  dummy_arm_->publish(arm);
  dummy_gripper_->publish(gripper);
  ASSERT_TRUE(wait_until(
      [this, &old_lease]() {
        const auto lease = source_lease("DummyPolicy");
        return !lease.empty() && lease != old_lease;
      },
      3s));
}

TEST_F(ExternalSourceSingleFlightFixture,
       PublisherWithoutRecentCommandsDoesNotKeepDefaultSourceActive) {
  trajectory_msgs::msg::JointTrajectory arm;
  arm.header.stamp = client_->now();
  arm.joint_names = {"joint1", "joint2"};
  trajectory_msgs::msg::JointTrajectory gripper;
  gripper.header.stamp = arm.header.stamp;
  gripper.joint_names = {"gripper_joint"};
  dummy_arm_->publish(arm);
  dummy_gripper_->publish(gripper);
  ASSERT_TRUE(wait_until([this]() { return source_owns_all("DummyPolicy"); }, 3s));

  // The publisher objects intentionally stay alive.  Source liveness follows
  // recent commands, not delayed DDS publisher-discovery teardown.
  EXPECT_TRUE(wait_until([this]() { return all_unowned(); }, 3s));
}

TEST_F(ExternalSourceSingleFlightFixture,
       ZeroStampedStreamingCommandDoesNotReachDownstream) {
  trajectory_msgs::msg::JointTrajectory arm;
  arm.joint_names = {"joint1", "joint2"};
  trajectory_msgs::msg::JointTrajectoryPoint arm_point;
  arm_point.positions = {1.0, 2.0};
  arm.points.push_back(arm_point);

  trajectory_msgs::msg::JointTrajectory gripper;
  gripper.joint_names = {"gripper_joint"};
  trajectory_msgs::msg::JointTrajectoryPoint gripper_point;
  gripper_point.positions = {0.01};
  gripper.points.push_back(gripper_point);

  ASSERT_TRUE(wait_until([this]() { return all_unowned(); }, 1s));
  ASSERT_TRUE(wait_until(
      [this]() {
        return dummy_arm_->get_subscription_count() > 0 &&
               dummy_gripper_->get_subscription_count() > 0;
      },
      1s));
  dummy_arm_->publish(arm);
  dummy_gripper_->publish(gripper);
  std::this_thread::sleep_for(300ms);
  EXPECT_TRUE(all_unowned());
  EXPECT_EQ(arm_output_marker_.load(), 0);

  arm.header.stamp = client_->now();
  gripper.header.stamp = arm.header.stamp;
  dummy_arm_->publish(arm);
  dummy_gripper_->publish(gripper);
  ASSERT_TRUE(wait_until([this]() { return source_owns_all("DummyPolicy"); }, 3s));

  arm.header.stamp = client_->now();
  dummy_arm_->publish(arm);
  ASSERT_TRUE(wait_until([this]() { return arm_output_marker_.load() == 1; }, 1s));

  arm.header.stamp.sec = 0;
  arm.header.stamp.nanosec = 0;
  arm.points.back().positions = {9.0, 9.0};
  dummy_arm_->publish(arm);
  EXPECT_FALSE(wait_until([this]() { return arm_output_marker_.load() == 9; },
                          300ms));
}

} // namespace
