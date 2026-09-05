#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/parallel_gripper_command.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <execution_manager_interfaces/action/leased_follow_joint_trajectory.hpp>
#include <execution_manager_interfaces/action/leased_parallel_gripper_command.hpp>
#include <execution_manager_interfaces/msg/authority_event.hpp>
#include <execution_manager_interfaces/msg/authority_status.hpp>
#include <execution_manager_interfaces/msg/leased_joint_reference.hpp>
#include <execution_manager_interfaces/msg/leased_pose_reference.hpp>
#include <execution_manager_interfaces/msg/leased_twist_reference.hpp>
#include <execution_manager_interfaces/msg/resource_claim.hpp>
#include <execution_manager_interfaces/msg/trajectory_execution_event.hpp>
#include <execution_manager_interfaces/msg/trajectory_execution_feedback.hpp>
#include <execution_manager_interfaces/srv/claim_control.hpp>
#include <execution_manager_interfaces/srv/release_control.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

#include "execution_manager/profile.hpp"
#include "execution_manager/router.hpp"
#include "execution_manager/selector.hpp"

namespace execution_manager {

class ExecutionManagerNode : public rclcpp::Node {
public:
  explicit ExecutionManagerNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~ExecutionManagerNode() override;

private:
  using FollowTrajectory = control_msgs::action::FollowJointTrajectory;
  using LeasedTrajectory =
      execution_manager_interfaces::action::LeasedFollowJointTrajectory;
  using UpstreamGoalHandle = rclcpp_action::ServerGoalHandle<LeasedTrajectory>;
  using DownstreamGoalHandle =
      rclcpp_action::ClientGoalHandle<FollowTrajectory>;
  using ExternalUpstreamGoalHandle =
      rclcpp_action::ServerGoalHandle<FollowTrajectory>;
  using ExternalDownstreamGoalHandle =
      rclcpp_action::ClientGoalHandle<LeasedTrajectory>;
  using GripperCommand = control_msgs::action::ParallelGripperCommand;
  using LeasedGripper =
      execution_manager_interfaces::action::LeasedParallelGripperCommand;
  using GripperUpstreamGoalHandle =
      rclcpp_action::ServerGoalHandle<LeasedGripper>;
  using GripperDownstreamGoalHandle =
      rclcpp_action::ClientGoalHandle<GripperCommand>;
  using ExternalGripperUpstreamGoalHandle =
      rclcpp_action::ServerGoalHandle<GripperCommand>;
  using ExternalGripperDownstreamGoalHandle =
      rclcpp_action::ClientGoalHandle<LeasedGripper>;

  struct ActionRoute;
  struct ExternalActionRoute;
  struct GripperActionRoute;
  struct ExternalGripperActionRoute;
  struct ExternalSourceRuntime;
  struct ExternalSourceLeaseSnapshot {
    std::string lease_id;
    bool owned_by_em{false};
  };

  void build_services();
  void build_routes();
  void build_external_sources();
  void build_action_route(std::uint8_t source_role, const std::string &resource,
                          const CommandCapability &capability);
  void build_gripper_action_route(std::uint8_t source_role,
                                  const std::string &resource,
                                  const CommandCapability &capability);

  template <typename EnvelopeT, typename PayloadT>
  void build_streaming_route(std::uint8_t source_role,
                             const std::string &resource,
                             const CommandCapability &capability);

  template <typename EnvelopeT, typename PayloadT>
  void build_external_streaming_route(
      const std::shared_ptr<ExternalSourceRuntime> &source,
      const SourceInputConfig &input);
  void build_external_action_route(
      const std::shared_ptr<ExternalSourceRuntime> &source,
      const SourceInputConfig &input);
  void build_external_gripper_action_route(
      const std::shared_ptr<ExternalSourceRuntime> &source,
      const SourceInputConfig &input);
  rclcpp_action::GoalResponse
  external_trajectory_goal(const std::shared_ptr<ExternalActionRoute> &route,
                           const rclcpp_action::GoalUUID &,
                           std::shared_ptr<const FollowTrajectory::Goal> goal);
  rclcpp_action::CancelResponse external_trajectory_cancel(
      const std::shared_ptr<ExternalUpstreamGoalHandle> &goal);
  void external_trajectory_accepted(
      const std::shared_ptr<ExternalActionRoute> &route,
      const std::shared_ptr<ExternalUpstreamGoalHandle> &goal);
  void execute_external_trajectory(
      const std::shared_ptr<ExternalActionRoute> &route,
      const std::shared_ptr<ExternalUpstreamGoalHandle> &upstream);
  rclcpp_action::GoalResponse external_gripper_goal(
      const std::shared_ptr<ExternalGripperActionRoute> &route,
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const GripperCommand::Goal> goal);
  rclcpp_action::CancelResponse external_gripper_cancel(
      const std::shared_ptr<ExternalGripperUpstreamGoalHandle> &goal);
  void external_gripper_accepted(
      const std::shared_ptr<ExternalGripperActionRoute> &route,
      const std::shared_ptr<ExternalGripperUpstreamGoalHandle> &goal);
  void execute_external_gripper(
      const std::shared_ptr<ExternalGripperActionRoute> &route,
      const std::shared_ptr<ExternalGripperUpstreamGoalHandle> &upstream);

  bool ensure_external_source_active(
      const std::shared_ptr<ExternalSourceRuntime> &source);
  ExternalSourceLeaseSnapshot external_source_lease_snapshot(
      const std::shared_ptr<ExternalSourceRuntime> &source);
  std::uint64_t request_external_source_state(
      const std::shared_ptr<ExternalSourceRuntime> &source, bool active,
      bool force = false);
  bool wait_external_source_active(
      const std::shared_ptr<ExternalSourceRuntime> &source,
      std::chrono::milliseconds timeout);
  void external_source_worker_loop();
  void reconcile_external_sources();
  void
  release_external_source(const std::shared_ptr<ExternalSourceRuntime> &source);
  void restore_default_sources();

  void claim_request(
      const std::shared_ptr<
          execution_manager_interfaces::srv::ClaimControl::Request>
          request,
      std::shared_ptr<execution_manager_interfaces::srv::ClaimControl::Response>
          response);
  void release_request(
      const std::shared_ptr<
          execution_manager_interfaces::srv::ReleaseControl::Request>
          request,
      std::shared_ptr<
          execution_manager_interfaces::srv::ReleaseControl::Response>
          response);

  void switch_controllers(const ControllerRoutes &routes);
  void refresh_controller_states();
  void cancel_actions(const std::vector<std::string> &resources);

  rclcpp_action::GoalResponse
  trajectory_goal(const std::shared_ptr<ActionRoute> &route,
                  const rclcpp_action::GoalUUID &,
                  std::shared_ptr<const LeasedTrajectory::Goal> goal);
  rclcpp_action::CancelResponse
  trajectory_cancel(const std::shared_ptr<UpstreamGoalHandle> &goal);
  void trajectory_accepted(const std::shared_ptr<ActionRoute> &route,
                           const std::shared_ptr<UpstreamGoalHandle> &goal);
  void execute_trajectory(const std::shared_ptr<ActionRoute> &route,
                          const std::shared_ptr<UpstreamGoalHandle> &upstream);
  rclcpp_action::GoalResponse
  gripper_goal(const std::shared_ptr<GripperActionRoute> &route,
               const rclcpp_action::GoalUUID &,
               std::shared_ptr<const LeasedGripper::Goal> goal);
  rclcpp_action::CancelResponse
  gripper_cancel(const std::shared_ptr<GripperUpstreamGoalHandle> &goal);
  void gripper_accepted(const std::shared_ptr<GripperActionRoute> &route,
                        const std::shared_ptr<GripperUpstreamGoalHandle> &goal);
  void
  execute_gripper(const std::shared_ptr<GripperActionRoute> &route,
                  const std::shared_ptr<GripperUpstreamGoalHandle> &upstream);

  void publish_status();
  void emit_event(std::uint8_t type, const std::string &lease_id,
                  const std::string &related_lease_id,
                  const Allocation *identity,
                  const std::vector<std::string> &resources,
                  const std::string &reason);
  std::string reserve_id(const std::string &prefix);
  void count_drop(const std::string &reason);

  ExecutionProfile profile_;
  AuthorityManager authority_;
  CommandRouter router_;
  bool use_sim_time_{false};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_;

  rclcpp::CallbackGroup::SharedPtr stream_group_;
  rclcpp::CallbackGroup::SharedPtr transition_group_;
  rclcpp::CallbackGroup::SharedPtr controller_group_;
  rclcpp::CallbackGroup::SharedPtr action_group_;
  rclcpp::Service<execution_manager_interfaces::srv::ClaimControl>::SharedPtr
      claim_service_;
  rclcpp::Service<execution_manager_interfaces::srv::ReleaseControl>::SharedPtr
      release_service_;
  rclcpp::Publisher<execution_manager_interfaces::msg::AuthorityStatus>::
      SharedPtr status_pub_;
  rclcpp::Publisher<
      execution_manager_interfaces::msg::AuthorityEvent>::SharedPtr event_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr source_watchdog_timer_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr
      list_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
      switch_client_;

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::vector<rclcpp::PublisherBase::SharedPtr> publishers_;
  std::vector<std::shared_ptr<ActionRoute>> action_routes_;
  std::vector<std::shared_ptr<ExternalActionRoute>> external_action_routes_;
  std::vector<std::shared_ptr<GripperActionRoute>> gripper_action_routes_;
  std::vector<std::shared_ptr<ExternalGripperActionRoute>>
      external_gripper_action_routes_;
  std::vector<std::shared_ptr<ExternalSourceRuntime>> external_sources_;
  std::mutex external_transition_mutex_;
  std::condition_variable external_transition_cv_;
  std::deque<std::shared_ptr<ExternalSourceRuntime>>
      external_transition_queue_;
  bool external_transition_stopping_{false};
  std::thread external_transition_worker_;
  std::mutex active_actions_mutex_;
  std::map<std::string, std::shared_ptr<ActionRoute>> active_actions_;
  std::map<std::string, std::shared_ptr<GripperActionRoute>>
      active_gripper_actions_;
  std::mutex observed_mutex_;
  std::map<std::string, std::vector<std::string>> observed_controllers_;
  std::mutex diagnostics_mutex_;
  std::map<std::string, std::uint64_t> drop_counters_;
  std::atomic<std::uint64_t> id_sequence_{0};
};

} // namespace execution_manager
