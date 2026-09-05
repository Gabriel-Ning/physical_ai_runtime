#include "execution_manager/node.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <iomanip>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <type_traits>

#include <rclcpp_action/exceptions.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace execution_manager {
namespace {

using namespace std::chrono_literals;

double stamp_seconds(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1e-9;
}

rclcpp::QoS command_qos() { return rclcpp::QoS(1).reliable(); }

std::string join_strings(const std::vector<std::string> &values) {
  std::ostringstream stream;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0) {
      stream << ',';
    }
    stream << values[index];
  }
  return stream.str();
}

std::string describe_claim_resources(
    const std::vector<execution_manager_interfaces::msg::ResourceClaim>
        &resources) {
  std::ostringstream stream;
  for (std::size_t index = 0; index < resources.size(); ++index) {
    if (index != 0) {
      stream << ',';
    }
    stream << resources[index].resource << ':'
           << resources[index].command_contract;
  }
  return stream.str();
}

std::vector<std::string> resources_for_lease(const AllocationMap &snapshot,
                                             const std::string &lease_id) {
  std::vector<std::string> resources;
  for (const auto &[resource, allocation] : snapshot) {
    if (allocation.lease_id == lease_id) {
      resources.push_back(resource);
    }
  }
  return resources;
}

const Allocation *identity_for_lease(const AllocationMap &snapshot,
                                     const std::string &lease_id) {
  for (const auto &[resource, allocation] : snapshot) {
    (void)resource;
    if (allocation.lease_id == lease_id) {
      return &allocation;
    }
  }
  return nullptr;
}

template <typename ClientT, typename GoalHandleT>
void cancel_goal_if_known(const typename ClientT::SharedPtr &client,
                          const std::shared_ptr<GoalHandleT> &goal) {
  if (!client || !goal) {
    return;
  }
  try {
    (void)client->async_cancel_goal(goal);
  } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
  }
}

// Real action liveness is wall/steady based. In simulation the action worker
// still polls futures on wall time so cancellation remains responsive while
// paused, but heartbeat cadence follows ROS simulation time.
constexpr auto kTrajectoryGuardHeartbeatPeriod = 50ms;
constexpr auto kSimActionResultPollWall = 5ms;

} // namespace

struct ExecutionManagerNode::ActionRoute {
  std::uint8_t source_role{0};
  std::string resource;
  CommandCapability capability;
  rclcpp_action::Server<LeasedTrajectory>::SharedPtr server;
  rclcpp_action::Client<FollowTrajectory>::SharedPtr client;
  rclcpp::Publisher<
      execution_manager_interfaces::msg::LeasedJointReference>::SharedPtr trace;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heartbeat;
  std::mutex mutex;
  std::shared_ptr<DownstreamGoalHandle> downstream;
  std::thread worker;
  std::atomic_bool goal_active{false};
};

struct ExecutionManagerNode::GripperActionRoute {
  std::uint8_t source_role{0};
  std::string resource;
  CommandCapability capability;
  rclcpp_action::Server<LeasedGripper>::SharedPtr server;
  rclcpp_action::Client<GripperCommand>::SharedPtr client;
  std::mutex mutex;
  std::shared_ptr<GripperDownstreamGoalHandle> downstream;
  std::thread worker;
  std::atomic_bool goal_active{false};
};

struct ExecutionManagerNode::ExternalSourceRuntime {
  enum class State : std::uint8_t {
    Inactive,
    Activating,
    Active,
    Suspended,
    Releasing,
    Fault,
  };

  ExternalSourceConfig config;
  std::mutex authority_mutex;
  std::condition_variable authority_cv;
  std::string lease_id;
  bool owns_lease{false};
  std::atomic<State> state{State::Inactive};
  std::atomic_bool desired_active{false};
  std::atomic_bool transition_queued{false};
  std::atomic_uint64_t transition_generation{0};
  std::uint64_t completed_generation{0};
  std::atomic_bool requested{false};
  std::atomic_bool has_command{false};
  std::atomic<std::int64_t> last_command_ns{0};
  std::atomic_size_t active_actions{0};
  std::vector<std::weak_ptr<rclcpp::SubscriptionBase>> streaming_inputs;
};

struct ExecutionManagerNode::ExternalActionRoute {
  std::shared_ptr<ExternalSourceRuntime> source;
  SourceInputConfig input;
  rclcpp_action::Server<FollowTrajectory>::SharedPtr server;
  rclcpp_action::Client<LeasedTrajectory>::SharedPtr client;
  std::mutex mutex;
  std::shared_ptr<ExternalDownstreamGoalHandle> downstream;
  std::thread worker;
  std::atomic_bool goal_active{false};
};

struct ExecutionManagerNode::ExternalGripperActionRoute {
  std::shared_ptr<ExternalSourceRuntime> source;
  SourceInputConfig input;
  rclcpp_action::Server<GripperCommand>::SharedPtr server;
  rclcpp_action::Client<LeasedGripper>::SharedPtr client;
  std::mutex mutex;
  std::shared_ptr<ExternalGripperDownstreamGoalHandle> downstream;
  std::thread worker;
  std::atomic_bool goal_active{false};
};

ExecutionManagerNode::ExecutionManagerNode(const rclcpp::NodeOptions &options)
    : Node("execution_manager", options),
      profile_(ExecutionProfile::from_yaml(
          declare_parameter<std::string>("profile"))),
      authority_(profile_),
      router_(declare_parameter<double>("max_command_age_s",
                                        profile_.max_command_age_s)) {
  if (!has_parameter("use_sim_time")) {
    declare_parameter<bool>("use_sim_time", false);
  }
  (void)get_parameter("use_sim_time", use_sim_time_);
  parameter_callback_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &parameter : parameters) {
          if (parameter.get_name() == "use_sim_time" &&
              parameter.as_bool() != use_sim_time_) {
            result.successful = false;
            result.reason =
                "use_sim_time is startup-only; restart execution_manager to change it";
            break;
          }
        }
        return result;
      });
  RCLCPP_INFO(
      get_logger(),
      "Time source: %s (command age, status timer, trajectory guard heartbeat)",
      use_sim_time_ ? "ROS simulation clock (/clock)" : "system wall clock");

  stream_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  transition_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  controller_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  action_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  status_pub_ =
      create_publisher<execution_manager_interfaces::msg::AuthorityStatus>(
          "/execution_manager/authority_status",
          rclcpp::QoS(1).reliable().transient_local());
  event_pub_ =
      create_publisher<execution_manager_interfaces::msg::AuthorityEvent>(
          "/execution_manager/authority_events",
          rclcpp::QoS(100).reliable().transient_local());

  const auto manager = profile_.resources.begin()->second.controller_manager;
  list_client_ = create_client<controller_manager_msgs::srv::ListControllers>(
      manager + "/list_controllers", rmw_qos_profile_services_default,
      controller_group_);
  switch_client_ =
      create_client<controller_manager_msgs::srv::SwitchController>(
          manager + "/switch_controller", rmw_qos_profile_services_default,
          controller_group_);

  build_services();
  build_routes();
  build_external_sources();
  external_transition_worker_ =
      std::thread([this]() { external_source_worker_loop(); });
  refresh_controller_states();
  publish_status();
  // Real robot: keep wall timer (steady), identical to pre-sim behavior.
  // Sim: drive status off /clock so telemetry follows simulation time.
  if (use_sim_time_) {
    status_timer_ = rclcpp::create_timer(
        this, get_clock(), 1s, [this]() { publish_status(); });
  } else {
    status_timer_ = create_wall_timer(1s, [this]() { publish_status(); });
  }
  // Publisher liveness uses steady_clock arrival times (see last_command_ns),
  // so this watchdog stays on wall time for both sim and real.
  source_watchdog_timer_ = create_wall_timer(250ms, [this]() {
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
    const auto liveness_ns = static_cast<std::int64_t>(
        std::max(1.0, 4.0 * profile_.max_command_age_s) * 1e9);
    for (const auto &source : external_sources_) {
      const auto lease = external_source_lease_snapshot(source);
      if (source->config.preempt || source->streaming_inputs.empty() ||
          lease.lease_id.empty()) {
        continue;
      }
      bool live = false;
      for (const auto &weak : source->streaming_inputs) {
        if (const auto sub = weak.lock()) {
          if (sub->get_publisher_count() > 0) {
            live = true;
            break;
          }
        }
      }
      const auto last_command_ns = source->last_command_ns.load();
      if (!live || last_command_ns == 0 ||
          now_ns - last_command_ns > liveness_ns) {
        source->has_command = false;
        request_external_source_state(source, false);
      }
    }
  });
  RCLCPP_INFO(get_logger(),
              "Execution Manager ready: profile=%s resources=%zu age=%.3fs",
              profile_.name.c_str(), profile_.resources.size(),
              profile_.max_command_age_s);
}

ExecutionManagerNode::~ExecutionManagerNode() {
  std::vector<std::string> resources;
  for (const auto &[name, config] : profile_.resources) {
    (void)config;
    resources.push_back(name);
  }
  cancel_actions(resources);
  for (auto &route : action_routes_) {
    if (route->worker.joinable()) {
      route->worker.join();
    }
  }
  for (auto &route : gripper_action_routes_) {
    if (route->worker.joinable()) {
      route->worker.join();
    }
  }
  for (auto &route : external_action_routes_) {
    if (route->worker.joinable()) {
      route->worker.join();
    }
  }
  for (auto &route : external_gripper_action_routes_) {
    if (route->worker.joinable()) {
      route->worker.join();
    }
  }
  {
    std::lock_guard lock(external_transition_mutex_);
    external_transition_stopping_ = true;
  }
  external_transition_cv_.notify_all();
  if (external_transition_worker_.joinable()) {
    external_transition_worker_.join();
  }
}

void ExecutionManagerNode::build_services() {
  claim_service_ =
      create_service<execution_manager_interfaces::srv::ClaimControl>(
          "/execution_manager/claim",
          [this](const std::shared_ptr<
                     execution_manager_interfaces::srv::ClaimControl::Request>
                     request,
                 std::shared_ptr<
                     execution_manager_interfaces::srv::ClaimControl::Response>
                     response) { claim_request(request, response); },
          rmw_qos_profile_services_default, transition_group_);
  release_service_ =
      create_service<execution_manager_interfaces::srv::ReleaseControl>(
          "/execution_manager/release",
          [this](
              const std::shared_ptr<
                  execution_manager_interfaces::srv::ReleaseControl::Request>
                  request,
              std::shared_ptr<
                  execution_manager_interfaces::srv::ReleaseControl::Response>
                  response) { release_request(request, response); },
          rmw_qos_profile_services_default, transition_group_);
}

void ExecutionManagerNode::build_routes() {
  for (const auto source_role :
       {std::uint8_t{1}, std::uint8_t{2}, std::uint8_t{3}, std::uint8_t{4}}) {
    for (const auto &[resource, config] : profile_.resources) {
      for (const auto &[command, capability] : config.commands) {
        if (capability.is_action) {
          if (capability.command_contract == "gripper_command") {
            build_gripper_action_route(source_role, resource, capability);
          } else {
            build_action_route(source_role, resource, capability);
          }
        } else if (command == "joint_reference") {
          build_streaming_route<
              execution_manager_interfaces::msg::LeasedJointReference,
              trajectory_msgs::msg::JointTrajectory>(source_role, resource,
                                                     capability);
        } else if (command == "pose_reference") {
          build_streaming_route<
              execution_manager_interfaces::msg::LeasedPoseReference,
              moveit_msgs::msg::CartesianTrajectory>(source_role, resource,
                                                     capability);
        } else if (command == "twist_reference") {
          build_streaming_route<
              execution_manager_interfaces::msg::LeasedTwistReference,
              geometry_msgs::msg::TwistStamped>(source_role, resource,
                                                capability);
        } else {
          throw std::runtime_error("unsupported streaming command contract '" +
                                   command + "'");
        }
      }
    }
  }
}

void ExecutionManagerNode::build_external_sources() {
  for (const auto &[name, config] : profile_.sources) {
    (void)name;
    auto source = std::make_shared<ExternalSourceRuntime>();
    source->config = config;
    if (config.activation_topic.empty()) {
      source->requested = true;
    } else {
      rclcpp::SubscriptionOptions options;
      options.callback_group = transition_group_;
      subscriptions_.push_back(create_subscription<std_msgs::msg::Bool>(
          config.activation_topic, command_qos(),
          [this, source](const std_msgs::msg::Bool::SharedPtr message) {
            source->requested = message->data;
            request_external_source_state(source, message->data);
          },
          options));
    }

    for (const auto &[resource, input] : config.inputs) {
      (void)resource;
      if (input.is_action) {
        if (input.command_contract == "joint_trajectory") {
          build_external_action_route(source, input);
        } else if (input.command_contract == "gripper_command") {
          build_external_gripper_action_route(source, input);
        } else {
          throw std::runtime_error(
              "unsupported external action command contract '" +
              input.command_contract + "'");
        }
      } else if (input.command_contract == "joint_reference") {
        build_external_streaming_route<
            execution_manager_interfaces::msg::LeasedJointReference,
            trajectory_msgs::msg::JointTrajectory>(source, input);
      } else if (input.command_contract == "pose_reference") {
        build_external_streaming_route<
            execution_manager_interfaces::msg::LeasedPoseReference,
            moveit_msgs::msg::CartesianTrajectory>(source, input);
      } else if (input.command_contract == "twist_reference") {
        build_external_streaming_route<
            execution_manager_interfaces::msg::LeasedTwistReference,
            geometry_msgs::msg::TwistStamped>(source, input);
      } else {
        throw std::runtime_error(
            "unsupported external streaming command contract '" +
            input.command_contract + "'");
      }
    }
    external_sources_.push_back(source);
  }
}

template <typename EnvelopeT, typename PayloadT>
void ExecutionManagerNode::build_streaming_route(
    const std::uint8_t source_role, const std::string &resource,
    const CommandCapability &capability) {
  const auto endpoint = ingress_endpoint(source_role, resource,
                                         capability.command_contract, false);
  auto trace = create_publisher<EnvelopeT>(
      execution_trace_topic(source_role, resource, capability.command_contract),
      command_qos());
  publishers_.push_back(trace);

  rclcpp::SubscriptionOptions options;
  options.callback_group = stream_group_;
  if constexpr (std::is_same_v<PayloadT,
                               trajectory_msgs::msg::JointTrajectory>) {
    if (capability.implementation ==
        "forward_command_controller/ForwardCommandController") {
      auto downstream = create_publisher<std_msgs::msg::Float64MultiArray>(
          capability.downstream_endpoint, command_qos());
      publishers_.push_back(downstream);
      subscriptions_.push_back(create_subscription<EnvelopeT>(
          endpoint, command_qos(),
          [this, resource, capability, trace,
           downstream](const typename EnvelopeT::SharedPtr message) {
            auto guard = authority_.route_guard();
            const auto decision = router_.decide(
                message->lease_id, resource, capability.command_contract,
                guard.allocations(), now().seconds(),
                stamp_seconds(message->header.stamp));
            if (!decision.accepted) {
              count_drop(decision.reason);
              return;
            }
            if (message->command.joint_names.empty() ||
                message->command.points.empty() ||
                message->command.points.back().positions.size() !=
                    message->command.joint_names.size()) {
              count_drop("invalid_joint_reference");
              return;
            }
            std_msgs::msg::Float64MultiArray output;
            output.data = message->command.points.back().positions;
            downstream->publish(output);
            trace->publish(*message);
          },
          options));
      return;
    }
  }

  auto downstream =
      create_publisher<PayloadT>(capability.downstream_endpoint, command_qos());
  publishers_.push_back(downstream);
  subscriptions_.push_back(create_subscription<EnvelopeT>(
      endpoint, command_qos(),
      [this, resource, capability, trace,
       downstream](const typename EnvelopeT::SharedPtr message) {
        auto guard = authority_.route_guard();
        const auto decision = router_.decide(
            message->lease_id, resource, capability.command_contract,
            guard.allocations(), now().seconds(),
            stamp_seconds(message->header.stamp));
        if (!decision.accepted) {
          count_drop(decision.reason);
          return;
        }
        downstream->publish(message->command);
        trace->publish(*message);
      },
      options));
}

template <typename EnvelopeT, typename PayloadT>
void ExecutionManagerNode::build_external_streaming_route(
    const std::shared_ptr<ExternalSourceRuntime> &source,
    const SourceInputConfig &input) {
  const auto ingress =
      ingress_endpoint(source->config.source_role, input.resource,
                       input.command_contract, false);
  auto publisher = create_publisher<EnvelopeT>(ingress, command_qos());
  publishers_.push_back(publisher);

  rclcpp::SubscriptionOptions options;
  options.callback_group = stream_group_;
  auto subscription = create_subscription<PayloadT>(
      input.endpoint, command_qos(),
      [this, source, input,
       publisher](const typename PayloadT::SharedPtr message) {
        if (message->header.stamp.sec == 0 &&
            message->header.stamp.nanosec == 0) {
          count_drop("command_has_no_admission_timestamp");
          return;
        }
        source->has_command = true;
        source->last_command_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();
        if (!source->requested) {
          count_drop("external_source_not_requested");
          return;
        }
        const auto lease = external_source_lease_snapshot(source);
        if (lease.lease_id.empty()) {
          request_external_source_state(source, true);
          count_drop("external_source_not_active");
          return;
        }
        EnvelopeT envelope;
        envelope.header = message->header;
        envelope.command = *message;
        envelope.lease_id = lease.lease_id;
        publisher->publish(envelope);
      },
      options);
  source->streaming_inputs.push_back(subscription);
  subscriptions_.push_back(subscription);
}

void ExecutionManagerNode::build_action_route(
    const std::uint8_t source_role, const std::string &resource,
    const CommandCapability &capability) {
  auto route = std::make_shared<ActionRoute>();
  route->source_role = source_role;
  route->resource = resource;
  route->capability = capability;
  route->client = rclcpp_action::create_client<FollowTrajectory>(
      get_node_base_interface(), get_node_graph_interface(),
      get_node_logging_interface(), get_node_waitables_interface(),
      capability.downstream_endpoint, action_group_);
  route->trace =
      create_publisher<execution_manager_interfaces::msg::LeasedJointReference>(
          execution_trace_topic(source_role, resource, "joint_trajectory"),
          command_qos());
  publishers_.push_back(route->trace);
  if (!capability.heartbeat_endpoint.empty()) {
    route->heartbeat = create_publisher<std_msgs::msg::Bool>(
        capability.heartbeat_endpoint, command_qos());
    publishers_.push_back(route->heartbeat);
  }
  const auto endpoint =
      ingress_endpoint(source_role, resource, "joint_trajectory", true);
  route->server = rclcpp_action::create_server<LeasedTrajectory>(
      get_node_base_interface(), get_node_clock_interface(),
      get_node_logging_interface(), get_node_waitables_interface(), endpoint,
      [this, route](const auto &uuid, const auto goal) {
        return trajectory_goal(route, uuid, goal);
      },
      [this](const auto goal) { return trajectory_cancel(goal); },
      [this, route](const auto goal) { trajectory_accepted(route, goal); },
      rcl_action_server_get_default_options(), action_group_);
  action_routes_.push_back(route);
}

void ExecutionManagerNode::build_gripper_action_route(
    const std::uint8_t source_role, const std::string &resource,
    const CommandCapability &capability) {
  auto route = std::make_shared<GripperActionRoute>();
  route->source_role = source_role;
  route->resource = resource;
  route->capability = capability;
  route->client = rclcpp_action::create_client<GripperCommand>(
      get_node_base_interface(), get_node_graph_interface(),
      get_node_logging_interface(), get_node_waitables_interface(),
      capability.downstream_endpoint, action_group_);
  route->server = rclcpp_action::create_server<LeasedGripper>(
      get_node_base_interface(), get_node_clock_interface(),
      get_node_logging_interface(), get_node_waitables_interface(),
      ingress_endpoint(source_role, resource, "gripper_command", true),
      [this, route](const auto &uuid, const auto goal) {
        return gripper_goal(route, uuid, goal);
      },
      [this](const auto goal) { return gripper_cancel(goal); },
      [this, route](const auto goal) { gripper_accepted(route, goal); },
      rcl_action_server_get_default_options(), action_group_);
  gripper_action_routes_.push_back(route);
}

void ExecutionManagerNode::build_external_action_route(
    const std::shared_ptr<ExternalSourceRuntime> &source,
    const SourceInputConfig &input) {
  auto route = std::make_shared<ExternalActionRoute>();
  route->source = source;
  route->input = input;
  const auto ingress = ingress_endpoint(
      source->config.source_role, input.resource, "joint_trajectory", true);
  route->client = rclcpp_action::create_client<LeasedTrajectory>(
      get_node_base_interface(), get_node_graph_interface(),
      get_node_logging_interface(), get_node_waitables_interface(), ingress,
      action_group_);
  route->server = rclcpp_action::create_server<FollowTrajectory>(
      get_node_base_interface(), get_node_clock_interface(),
      get_node_logging_interface(), get_node_waitables_interface(),
      input.endpoint,
      [this, route](const auto &uuid, const auto goal) {
        return external_trajectory_goal(route, uuid, goal);
      },
      [this](const auto goal) { return external_trajectory_cancel(goal); },
      [this, route](const auto goal) {
        external_trajectory_accepted(route, goal);
      },
      rcl_action_server_get_default_options(), action_group_);
  external_action_routes_.push_back(route);
}

void ExecutionManagerNode::build_external_gripper_action_route(
    const std::shared_ptr<ExternalSourceRuntime> &source,
    const SourceInputConfig &input) {
  auto route = std::make_shared<ExternalGripperActionRoute>();
  route->source = source;
  route->input = input;
  route->client = rclcpp_action::create_client<LeasedGripper>(
      get_node_base_interface(), get_node_graph_interface(),
      get_node_logging_interface(), get_node_waitables_interface(),
      ingress_endpoint(source->config.source_role, input.resource,
                       input.command_contract, true),
      action_group_);
  route->server = rclcpp_action::create_server<GripperCommand>(
      get_node_base_interface(), get_node_clock_interface(),
      get_node_logging_interface(), get_node_waitables_interface(),
      input.endpoint,
      [this, route](const auto &uuid, const auto goal) {
        return external_gripper_goal(route, uuid, goal);
      },
      [this](const auto goal) { return external_gripper_cancel(goal); },
      [this, route](const auto goal) {
        external_gripper_accepted(route, goal);
      },
      rcl_action_server_get_default_options(), action_group_);
  external_gripper_action_routes_.push_back(route);
}

bool ExecutionManagerNode::ensure_external_source_active(
    const std::shared_ptr<ExternalSourceRuntime> &source) {
  std::unique_lock lock(source->authority_mutex);
  if (!source->lease_id.empty()) {
    const auto snapshot = authority_.snapshot();
    if (identity_for_lease(snapshot, source->lease_id) != nullptr) {
      source->state = ExternalSourceRuntime::State::Active;
      source->authority_cv.notify_all();
      return true;
    }
    source->lease_id.clear();
  }

  {
    const auto snapshot = authority_.snapshot();
    std::string adopted;
    bool can_adopt = !source->config.inputs.empty();
    for (const auto &[resource, input] : source->config.inputs) {
      (void)resource;
      const auto item = snapshot.find(input.resource);
      if (item == snapshot.end() ||
          item->second.state != AuthorityState::Owned ||
          item->second.source_instance != source->config.name ||
          item->second.source_role != source->config.source_role ||
          item->second.command_contract != input.command_contract) {
        can_adopt = false;
        break;
      }
      if (adopted.empty()) {
        adopted = item->second.lease_id;
      } else if (adopted != item->second.lease_id) {
        can_adopt = false;
        break;
      }
    }
    if (can_adopt && !adopted.empty()) {
      source->lease_id = adopted;
      source->owns_lease = false;
      source->state = ExternalSourceRuntime::State::Active;
      source->authority_cv.notify_all();
      RCLCPP_INFO(get_logger(), "External source %s active: lease=%s",
                  source->config.name.c_str(), source->lease_id.c_str());
      return true;
    }

    // A displaced default source keeps publishing while a preempting source
    // owns the resources.  Do not turn every frame into a doomed claim (or
    // block behind an in-flight transition); restore_default_sources() will
    // claim it once the resources are unowned again.
    if (!source->config.preempt) {
      for (const auto &[resource, input] : source->config.inputs) {
        (void)resource;
        const auto item = snapshot.find(input.resource);
        if (item != snapshot.end() &&
            item->second.state != AuthorityState::Unowned) {
          count_drop("external_source_waiting_for_resources");
          source->state = ExternalSourceRuntime::State::Suspended;
          source->authority_cv.notify_all();
          return false;
        }
      }
    }
  }

  auto request = std::make_shared<
      execution_manager_interfaces::srv::ClaimControl::Request>();
  auto response = std::make_shared<
      execution_manager_interfaces::srv::ClaimControl::Response>();
  request->source_role = source->config.source_role;
  request->source_instance = source->config.name;
  request->preempt = source->config.preempt;
  for (const auto &[resource, input] : source->config.inputs) {
    (void)resource;
    execution_manager_interfaces::msg::ResourceClaim claim;
    claim.resource = input.resource;
    claim.command_contract = input.command_contract;
    request->resources.push_back(std::move(claim));
  }
  claim_request(request, response);
  if (!response->success) {
    source->state = response->message.find("fault") != std::string::npos ||
                            response->message.find("timed out") !=
                                std::string::npos ||
                            response->message.find("unavailable") !=
                                std::string::npos ||
                            response->message.find("ok=false") !=
                                std::string::npos
                        ? ExternalSourceRuntime::State::Fault
                        : ExternalSourceRuntime::State::Suspended;
    source->authority_cv.notify_all();
    RCLCPP_WARN(get_logger(), "Failed to activate external source %s: %s",
                source->config.name.c_str(), response->message.c_str());
    return false;
  }
  source->lease_id = response->lease_id;
  source->owns_lease = true;
  source->state = ExternalSourceRuntime::State::Active;
  source->authority_cv.notify_all();
  RCLCPP_INFO(get_logger(), "External source %s active: lease=%s",
              source->config.name.c_str(), source->lease_id.c_str());
  return true;
}

void ExecutionManagerNode::release_external_source(
    const std::shared_ptr<ExternalSourceRuntime> &source) {
  std::lock_guard lock(source->authority_mutex);
  if (source->lease_id.empty()) {
    source->state = ExternalSourceRuntime::State::Inactive;
    source->authority_cv.notify_all();
    return;
  }
  auto request = std::make_shared<
      execution_manager_interfaces::srv::ReleaseControl::Request>();
  auto response = std::make_shared<
      execution_manager_interfaces::srv::ReleaseControl::Response>();
  request->lease_id = source->lease_id;
  release_request(request, response);
  source->lease_id.clear();
  source->owns_lease = false;
  source->state = response->success ? ExternalSourceRuntime::State::Inactive
                                    : ExternalSourceRuntime::State::Fault;
  source->authority_cv.notify_all();
}

ExecutionManagerNode::ExternalSourceLeaseSnapshot
ExecutionManagerNode::external_source_lease_snapshot(
    const std::shared_ptr<ExternalSourceRuntime> &source) {
  if (source->state.load() != ExternalSourceRuntime::State::Active) {
    return {};
  }
  std::lock_guard lock(source->authority_mutex);
  if (source->state.load() != ExternalSourceRuntime::State::Active) {
    return {};
  }
  // A source lease can also be released through the public authority API
  // (for example, by an RMI NodeActivation scope).  Never keep routing with a
  // locally cached lease after authority no longer recognizes it.  Marking the
  // source inactive lets the next command enqueue one source-level activation
  // and either adopt the caller's replacement lease or claim a new one.
  const auto allocations = authority_.snapshot();
  if (source->lease_id.empty() ||
      identity_for_lease(allocations, source->lease_id) == nullptr) {
    source->lease_id.clear();
    source->owns_lease = false;
    source->state = ExternalSourceRuntime::State::Inactive;
    source->authority_cv.notify_all();
    return {};
  }
  return {source->lease_id, source->owns_lease};
}

std::uint64_t ExecutionManagerNode::request_external_source_state(
    const std::shared_ptr<ExternalSourceRuntime> &source, const bool active,
    const bool force) {
  // Joining an in-flight transition is a lock-free operation.  In particular,
  // streaming callbacks must never wait on authority_mutex while the lifecycle
  // worker is synchronously waiting for controller-manager responses.
  if (source->desired_active.load() == active &&
      source->transition_queued.load()) {
    return source->transition_generation.load();
  }
  bool enqueue = false;
  std::uint64_t generation = 0;
  {
    std::lock_guard lock(source->authority_mutex);
    const bool previous = source->desired_active.load();
    const auto state = source->state.load();
    const bool transition_running = source->transition_queued.load();

    if (previous == active && transition_running) {
      return source->transition_generation.load();
    }
    if (!force && previous == active &&
        ((active && (state == ExternalSourceRuntime::State::Active ||
                     state == ExternalSourceRuntime::State::Suspended ||
                     state == ExternalSourceRuntime::State::Fault)) ||
         (!active && state == ExternalSourceRuntime::State::Inactive))) {
      return source->transition_generation.load();
    }

    source->desired_active = active;
    generation = source->transition_generation.fetch_add(1) + 1;
    if (!transition_running) {
      source->transition_queued = true;
      enqueue = true;
    }
  }
  if (enqueue) {
    {
      std::lock_guard lock(external_transition_mutex_);
      external_transition_queue_.push_back(source);
    }
    external_transition_cv_.notify_one();
  }
  return generation;
}

bool ExecutionManagerNode::wait_external_source_active(
    const std::shared_ptr<ExternalSourceRuntime> &source,
    const std::chrono::milliseconds timeout) {
  if (!external_source_lease_snapshot(source).lease_id.empty()) {
    return true;
  }
  // All action workers for this source join the same generation.  Only the
  // lifecycle worker writes transition states and performs claim/release.
  const auto generation = request_external_source_state(source, true, true);
  std::unique_lock lock(source->authority_mutex);
  return source->authority_cv.wait_for(lock, timeout, [&source, generation]() {
           return source->completed_generation >= generation;
         }) &&
         source->state.load() == ExternalSourceRuntime::State::Active;
}

void ExecutionManagerNode::external_source_worker_loop() {
  while (true) {
    std::shared_ptr<ExternalSourceRuntime> source;
    {
      std::unique_lock lock(external_transition_mutex_);
      external_transition_cv_.wait(lock, [this]() {
        return external_transition_stopping_ ||
               !external_transition_queue_.empty();
      });
      if (external_transition_stopping_ &&
          external_transition_queue_.empty()) {
        return;
      }
      source = external_transition_queue_.front();
      external_transition_queue_.pop_front();
    }

    std::uint64_t generation = 0;
    bool desired = false;
    {
      std::lock_guard lock(source->authority_mutex);
      generation = source->transition_generation.load();
      desired = source->desired_active.load();
      source->state = desired ? ExternalSourceRuntime::State::Activating
                              : ExternalSourceRuntime::State::Releasing;
    }
    if (desired) {
      (void)ensure_external_source_active(source);
      reconcile_external_sources();
    } else {
      release_external_source(source);
      restore_default_sources();
    }

    bool requeue = false;
    {
      std::lock_guard lock(source->authority_mutex);
      source->completed_generation =
          std::max(source->completed_generation, generation);
      source->transition_queued = false;
      if (source->transition_generation.load() != generation) {
        source->transition_queued = true;
        requeue = true;
      }
      source->authority_cv.notify_all();
    }
    if (requeue) {
      {
        std::lock_guard lock(external_transition_mutex_);
        external_transition_queue_.push_back(source);
      }
      external_transition_cv_.notify_one();
    }
  }
}

void ExecutionManagerNode::reconcile_external_sources() {
  const auto allocations = authority_.snapshot();
  for (const auto &source : external_sources_) {
    std::lock_guard lock(source->authority_mutex);
    if (source->lease_id.empty() ||
        identity_for_lease(allocations, source->lease_id) != nullptr) {
      continue;
    }
    source->lease_id.clear();
    source->owns_lease = false;
    source->state = source->desired_active.load()
                        ? ExternalSourceRuntime::State::Suspended
                        : ExternalSourceRuntime::State::Inactive;
    source->authority_cv.notify_all();
  }
}

void ExecutionManagerNode::restore_default_sources() {
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          std::chrono::steady_clock::now().time_since_epoch())
                          .count();
  const auto liveness_ns = static_cast<std::int64_t>(
      std::max(1.0, 4.0 * profile_.max_command_age_s) * 1e9);
  for (const auto &source : external_sources_) {
    if (source->config.preempt || !source->requested || !source->has_command) {
      continue;
    }
    const auto last_command_ns = source->last_command_ns.load();
    if (last_command_ns == 0 || now_ns - last_command_ns > liveness_ns) {
      source->has_command = false;
      continue;
    }
    bool live = source->streaming_inputs.empty();
    for (const auto &weak : source->streaming_inputs) {
      if (const auto subscription = weak.lock()) {
        if (subscription->get_publisher_count() > 0) {
          live = true;
          break;
        }
      }
    }
    if (live) {
      source->desired_active = false;
      request_external_source_state(source, true);
    }
  }
}

void ExecutionManagerNode::claim_request(
    const std::shared_ptr<
        execution_manager_interfaces::srv::ClaimControl::Request>
        request,
    std::shared_ptr<execution_manager_interfaces::srv::ClaimControl::Response>
        response) {
  LeaseRequest claim;
  claim.lease_id = reserve_id("lease");
  claim.source_role = request->source_role;
  claim.source_instance = request->source_instance;
  claim.preempt = request->preempt;
  for (const auto &item : request->metadata) {
    claim.metadata[item.key] = item.value;
  }
  for (const auto &resource : request->resources) {
    claim.resources.push_back({resource.resource, resource.command_contract});
  }
  const auto before = authority_.snapshot();
  TransitionResult result;
  try {
    result = authority_.claim(
        claim, [this](const auto &resources) { cancel_actions(resources); },
        [this](const auto &routes) { switch_controllers(routes); });
  } catch (const std::exception &error) {
    response->success = false;
    response->message = error.what();
    return;
  }

  for (const auto &old_lease : result.displaced_lease_ids) {
    const auto resources = resources_for_lease(before, old_lease);
    emit_event(execution_manager_interfaces::msg::AuthorityEvent::PREEMPTED,
               old_lease, claim.lease_id, identity_for_lease(before, old_lease),
               resources,
               result.success ? "explicit_preempt" : "preempt_switch_failed");
  }
  response->success = result.success;
  response->message = result.message;
  if (!result.success) {
    refresh_controller_states();
    RCLCPP_ERROR(
        get_logger(),
        "Authority claim transition failed: lease=%s source_role=%u "
        "source_instance=%s resources=[%s] affected=[%s] reason=%s",
        claim.lease_id.c_str(), static_cast<unsigned>(claim.source_role),
        claim.source_instance.c_str(),
        describe_claim_resources(request->resources).c_str(),
        join_strings(result.affected_resources).c_str(), result.message.c_str());
    Allocation attempted_identity;
    attempted_identity.source_role = claim.source_role;
    attempted_identity.source_instance = claim.source_instance;
    attempted_identity.metadata = claim.metadata;
    emit_event(
        execution_manager_interfaces::msg::AuthorityEvent::TRANSITION_FAILED,
        claim.lease_id, "", &attempted_identity, result.affected_resources,
        result.message);
    publish_status();
    return;
  }

  response->lease_id = claim.lease_id;
  for (const auto &requested : claim.resources) {
    const auto &capability = profile_.resources.at(requested.resource)
                                 .commands.at(requested.command_contract);
    execution_manager_interfaces::msg::CommandEndpoint endpoint;
    endpoint.resource = requested.resource;
    endpoint.command_contract = requested.command_contract;
    endpoint.endpoint =
        ingress_endpoint(claim.source_role, requested.resource,
                         requested.command_contract, capability.is_action);
    endpoint.is_action = capability.is_action;
    response->endpoints.push_back(std::move(endpoint));
  }
  const auto after = authority_.snapshot();
  emit_event(execution_manager_interfaces::msg::AuthorityEvent::CLAIMED,
             claim.lease_id, "", identity_for_lease(after, claim.lease_id),
             resources_for_lease(after, claim.lease_id), "claimed");
  publish_status();
}

void ExecutionManagerNode::release_request(
    const std::shared_ptr<
        execution_manager_interfaces::srv::ReleaseControl::Request>
        request,
    std::shared_ptr<execution_manager_interfaces::srv::ReleaseControl::Response>
        response) {
  const auto before = authority_.snapshot();
  const auto identity = identity_for_lease(before, request->lease_id);
  if (identity == nullptr) {
    // A late release after preemption is a normal distributed-system race.
    // Make release idempotent instead of turning it into a false FAULT fact.
    response->success = true;
    response->message = "lease_not_active";
    response->authority_state =
        execution_manager_interfaces::msg::ResourceAuthority::UNOWNED;
    return;
  }
  const auto result =
      authority_.release(request->lease_id, [this](const auto &resources) {
        cancel_actions(resources);
      });
  response->success = result.success;
  response->message = result.message;
  response->authority_state =
      result.success
          ? execution_manager_interfaces::msg::ResourceAuthority::UNOWNED
          : execution_manager_interfaces::msg::ResourceAuthority::FAULT;
  emit_event(result.success
                 ? execution_manager_interfaces::msg::AuthorityEvent::RELEASED
                 : execution_manager_interfaces::msg::AuthorityEvent::
                       TRANSITION_FAILED,
             request->lease_id, "", identity, result.affected_resources,
             result.message);
  if (!result.success) {
    refresh_controller_states();
    RCLCPP_ERROR(get_logger(),
                 "Authority release transition failed: lease=%s source_role=%u "
                 "source_instance=%s affected=[%s] reason=%s",
                 request->lease_id.c_str(),
                 static_cast<unsigned>(identity->source_role),
                 identity->source_instance.c_str(),
                 join_strings(result.affected_resources).c_str(),
                 result.message.c_str());
  }
  publish_status();
}

void ExecutionManagerNode::switch_controllers(const ControllerRoutes &routes) {
  if (!switch_client_->wait_for_service(5s) ||
      !list_client_->wait_for_service(5s)) {
    throw std::runtime_error("controller_manager services unavailable");
  }
  auto list_future = list_client_->async_send_request(
      std::make_shared<
          controller_manager_msgs::srv::ListControllers::Request>());
  if (list_future.wait_for(5s) != std::future_status::ready) {
    throw std::runtime_error("list_controllers timed out");
  }
  const auto listed = list_future.get();
  std::set<std::string> active;
  for (const auto &controller : listed->controller) {
    if (controller.state == "active") {
      active.insert(controller.name);
    }
  }
  std::set<std::string> activate;
  std::set<std::string> deactivate;
  for (const auto &[resource, selected] : routes) {
    if (!active.count(selected.controller)) {
      activate.insert(selected.controller);
    }
    for (const auto &[command, capability] :
         profile_.resources.at(resource).commands) {
      (void)command;
      if (capability.controller != selected.controller &&
          active.count(capability.controller)) {
        deactivate.insert(capability.controller);
      }
    }
  }
  auto request = std::make_shared<
      controller_manager_msgs::srv::SwitchController::Request>();
  request->activate_controllers.assign(activate.begin(), activate.end());
  request->deactivate_controllers.assign(deactivate.begin(), deactivate.end());
  request->strictness =
      controller_manager_msgs::srv::SwitchController::Request::STRICT;
  request->activate_asap = true;
  request->timeout.sec = 5;
  auto future = switch_client_->async_send_request(request);
  if (future.wait_for(6s) != std::future_status::ready) {
    refresh_controller_states();
    throw std::runtime_error("switch_controller timed out");
  }
  if (!future.get()->ok) {
    refresh_controller_states();
    throw std::runtime_error("switch_controller returned ok=false");
  }
  refresh_controller_states();
}

void ExecutionManagerNode::refresh_controller_states() {
  if (!list_client_->wait_for_service(500ms)) {
    return;
  }
  auto future = list_client_->async_send_request(
      std::make_shared<
          controller_manager_msgs::srv::ListControllers::Request>());
  if (future.wait_for(1s) != std::future_status::ready) {
    return;
  }
  const auto response = future.get();
  std::lock_guard lock(observed_mutex_);
  observed_controllers_.clear();
  for (const auto &[resource, config] : profile_.resources) {
    std::set<std::string> known;
    for (const auto &[command, capability] : config.commands) {
      (void)command;
      known.insert(capability.controller);
    }
    for (const auto &controller : response->controller) {
      if (known.count(controller.name) && controller.state == "active") {
        observed_controllers_[resource].push_back(controller.name);
      }
    }
  }
}

void ExecutionManagerNode::cancel_actions(
    const std::vector<std::string> &resources) {
  std::vector<std::shared_ptr<ActionRoute>> routes;
  std::vector<std::shared_ptr<GripperActionRoute>> gripper_routes;
  {
    std::lock_guard lock(active_actions_mutex_);
    for (const auto &resource : resources) {
      const auto item = active_actions_.find(resource);
      if (item != active_actions_.end()) {
        routes.push_back(item->second);
      }
      const auto gripper = active_gripper_actions_.find(resource);
      if (gripper != active_gripper_actions_.end()) {
        gripper_routes.push_back(gripper->second);
      }
    }
  }
  for (const auto &route : routes) {
    std::shared_ptr<DownstreamGoalHandle> downstream;
    {
      std::lock_guard lock(route->mutex);
      downstream = route->downstream;
    }
    // Cancellation acknowledgement is deliberately not awaited: the
    // following STRICT controller switch is the hard fencing backstop. This
    // keeps takeover latency bounded even when a JTC action server is slow.
    cancel_goal_if_known<rclcpp_action::Client<FollowTrajectory>,
                         DownstreamGoalHandle>(route->client, downstream);
  }
  for (const auto &route : gripper_routes) {
    std::shared_ptr<GripperDownstreamGoalHandle> downstream;
    {
      std::lock_guard lock(route->mutex);
      downstream = route->downstream;
    }
    cancel_goal_if_known<rclcpp_action::Client<GripperCommand>,
                         GripperDownstreamGoalHandle>(route->client,
                                                      downstream);
  }
}

rclcpp_action::GoalResponse ExecutionManagerNode::trajectory_goal(
    const std::shared_ptr<ActionRoute> &route, const rclcpp_action::GoalUUID &,
    std::shared_ptr<const LeasedTrajectory::Goal> goal) {
  if (goal->resource != route->resource) {
    count_drop("trajectory_resource_mismatch");
    return rclcpp_action::GoalResponse::REJECT;
  }
  auto guard = authority_.route_guard();
  const auto decision = router_.decide(
      goal->lease_id, route->resource, "joint_trajectory", guard.allocations(),
      now().seconds(), stamp_seconds(goal->header.stamp));
  if (!decision.accepted) {
    count_drop(decision.reason);
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool expected = false;
  if (!route->goal_active.compare_exchange_strong(expected, true)) {
    count_drop("trajectory_already_active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ExecutionManagerNode::trajectory_cancel(
    const std::shared_ptr<UpstreamGoalHandle> &) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ExecutionManagerNode::trajectory_accepted(
    const std::shared_ptr<ActionRoute> &route,
    const std::shared_ptr<UpstreamGoalHandle> &goal) {
  if (route->worker.joinable()) {
    route->worker.join();
  }
  route->worker =
      std::thread([this, route, goal]() { execute_trajectory(route, goal); });
}

void ExecutionManagerNode::execute_trajectory(
    const std::shared_ptr<ActionRoute> &route,
    const std::shared_ptr<UpstreamGoalHandle> &upstream) {
  const auto cleanup = [this, &route]() {
    if (route->heartbeat) {
      std_msgs::msg::Bool message;
      message.data = false;
      route->heartbeat->publish(message);
    }
    {
      std::lock_guard lock(route->mutex);
      route->downstream.reset();
    }
    {
      std::lock_guard lock(active_actions_mutex_);
      const auto item = active_actions_.find(route->resource);
      if (item != active_actions_.end() && item->second == route) {
        active_actions_.erase(item);
      }
    }
    route->goal_active.store(false);
  };
  auto fail = [&upstream](const std::string &message) {
    auto result = std::make_shared<LeasedTrajectory::Result>();
    result->error_code = FollowTrajectory::Result::INVALID_GOAL;
    result->error_string = message;
    upstream->abort(result);
  };

  if (!route->client->wait_for_action_server(5s)) {
    fail("downstream JTC action unavailable");
    cleanup();
    return;
  }
  const auto leased = upstream->get_goal();
  FollowTrajectory::Goal goal;
  goal.trajectory = leased->trajectory;
  goal.path_tolerance = leased->path_tolerance;
  goal.goal_tolerance = leased->goal_tolerance;
  goal.goal_time_tolerance = leased->goal_time_tolerance;
  auto options = rclcpp_action::Client<FollowTrajectory>::SendGoalOptions();
  options.feedback_callback = [upstream](const auto &, const auto feedback) {
    auto output = std::make_shared<LeasedTrajectory::Feedback>();
    output->header = feedback->header;
    output->joint_names = feedback->joint_names;
    output->desired = feedback->desired;
    output->actual = feedback->actual;
    output->error = feedback->error;
    upstream->publish_feedback(output);
  };
  auto send_future = route->client->async_send_goal(goal, options);
  if (send_future.wait_for(5s) != std::future_status::ready) {
    fail("downstream JTC goal request timed out");
    cleanup();
    return;
  }
  auto downstream = send_future.get();
  if (!downstream) {
    fail("downstream JTC rejected goal");
    cleanup();
    return;
  }
  {
    std::lock_guard lock(route->mutex);
    route->downstream = downstream;
  }
  {
    std::lock_guard lock(active_actions_mutex_);
    active_actions_[route->resource] = route;
  }
  // Close the dispatch race with takeover. If takeover happened before this
  // route was visible in active_actions_, the lease check catches it here. If
  // it happens after registration, cancel_actions() sees the route. Use a fresh
  // stamp because this second check is authority fencing, not command-age
  // admission (waiting for the downstream action server may legitimately take
  // longer than max_command_age_s).
  {
    const auto route_guard = authority_.route_guard();
    const auto now_s = now().seconds();
    const auto decision =
        router_.decide(leased->lease_id, route->resource, "joint_trajectory",
                       route_guard.allocations(), now_s, now_s);
    if (!decision.accepted) {
      count_drop("trajectory_post_dispatch_" + decision.reason);
      cancel_goal_if_known<rclcpp_action::Client<FollowTrajectory>,
                           DownstreamGoalHandle>(route->client, downstream);
      fail("lease lost while dispatching downstream trajectory");
      cleanup();
      return;
    }
  }
  execution_manager_interfaces::msg::LeasedJointReference trace;
  trace.header = leased->header;
  trace.lease_id = leased->lease_id;
  trace.command = leased->trajectory;
  route->trace->publish(trace);

  auto result_future = route->client->async_get_result(downstream);
  if (!use_sim_time_) {
    // Real-robot protection path: wall-paced 50 ms heartbeat — unchanged.
    while (rclcpp::ok() &&
           result_future.wait_for(kTrajectoryGuardHeartbeatPeriod) !=
               std::future_status::ready) {
      if (route->heartbeat) {
        std_msgs::msg::Bool message;
        message.data = true;
        route->heartbeat->publish(message);
      }
      if (upstream->is_canceling()) {
        cancel_goal_if_known<rclcpp_action::Client<FollowTrajectory>,
                             DownstreamGoalHandle>(route->client, downstream);
      }
    }
  } else {
    auto last_heartbeat = now();
    while (rclcpp::ok()) {
      if (result_future.wait_for(kSimActionResultPollWall) ==
          std::future_status::ready) {
        break;
      }
      if (upstream->is_canceling()) {
        cancel_goal_if_known<rclcpp_action::Client<FollowTrajectory>,
                             DownstreamGoalHandle>(route->client, downstream);
      }
      const auto current = now();
      const bool clock_reset = current < last_heartbeat;
      const bool heartbeat_due =
          clock_reset || current - last_heartbeat >=
                             rclcpp::Duration(kTrajectoryGuardHeartbeatPeriod);
      if (route->heartbeat && heartbeat_due) {
        std_msgs::msg::Bool message;
        message.data = true;
        route->heartbeat->publish(message);
        last_heartbeat = current;
      }
    }
  }
  if (!rclcpp::ok()) {
    cleanup();
    return;
  }
  const auto wrapped = result_future.get();
  auto result = std::make_shared<LeasedTrajectory::Result>();
  if (wrapped.result) {
    result->error_code = wrapped.result->error_code;
    result->error_string = wrapped.result->error_string;
  }
  if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
    upstream->succeed(result);
  } else if (wrapped.code == rclcpp_action::ResultCode::CANCELED &&
             upstream->is_canceling()) {
    upstream->canceled(result);
  } else {
    upstream->abort(result);
  }
  cleanup();
}

rclcpp_action::GoalResponse ExecutionManagerNode::gripper_goal(
    const std::shared_ptr<GripperActionRoute> &route,
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const LeasedGripper::Goal> goal) {
  if (goal->resource != route->resource) {
    count_drop("gripper_resource_mismatch");
    return rclcpp_action::GoalResponse::REJECT;
  }
  auto guard = authority_.route_guard();
  const auto decision = router_.decide(
      goal->lease_id, route->resource, "gripper_command", guard.allocations(),
      now().seconds(), stamp_seconds(goal->header.stamp));
  if (!decision.accepted) {
    count_drop(decision.reason);
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool expected = false;
  if (!route->goal_active.compare_exchange_strong(expected, true)) {
    count_drop("gripper_already_active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ExecutionManagerNode::gripper_cancel(
    const std::shared_ptr<GripperUpstreamGoalHandle> &) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ExecutionManagerNode::gripper_accepted(
    const std::shared_ptr<GripperActionRoute> &route,
    const std::shared_ptr<GripperUpstreamGoalHandle> &goal) {
  if (route->worker.joinable()) {
    route->worker.join();
  }
  route->worker =
      std::thread([this, route, goal]() { execute_gripper(route, goal); });
}

void ExecutionManagerNode::execute_gripper(
    const std::shared_ptr<GripperActionRoute> &route,
    const std::shared_ptr<GripperUpstreamGoalHandle> &upstream) {
  const auto cleanup = [this, &route]() {
    {
      std::lock_guard lock(route->mutex);
      route->downstream.reset();
    }
    {
      std::lock_guard lock(active_actions_mutex_);
      const auto item = active_gripper_actions_.find(route->resource);
      if (item != active_gripper_actions_.end() && item->second == route) {
        active_gripper_actions_.erase(item);
      }
    }
    route->goal_active.store(false);
  };
  auto fail = [&upstream](const std::string &message) {
    (void)message;
    auto result = std::make_shared<LeasedGripper::Result>();
    upstream->abort(result);
  };

  if (!route->client->wait_for_action_server(5s)) {
    fail("downstream gripper action unavailable");
    cleanup();
    return;
  }
  const auto leased = upstream->get_goal();
  GripperCommand::Goal goal;
  goal.command = leased->command;
  auto options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
  options.feedback_callback = [upstream](const auto &, const auto feedback) {
    auto output = std::make_shared<LeasedGripper::Feedback>();
    output->state = feedback->state;
    upstream->publish_feedback(output);
  };
  auto send_future = route->client->async_send_goal(goal, options);
  if (send_future.wait_for(5s) != std::future_status::ready) {
    fail("downstream gripper goal request timed out");
    cleanup();
    return;
  }
  auto downstream = send_future.get();
  if (!downstream) {
    fail("downstream gripper rejected goal");
    cleanup();
    return;
  }
  {
    std::lock_guard lock(route->mutex);
    route->downstream = downstream;
  }
  {
    std::lock_guard lock(active_actions_mutex_);
    active_gripper_actions_[route->resource] = route;
  }
  {
    const auto route_guard = authority_.route_guard();
    const auto now_s = now().seconds();
    const auto decision =
        router_.decide(leased->lease_id, route->resource, "gripper_command",
                       route_guard.allocations(), now_s, now_s);
    if (!decision.accepted) {
      count_drop("gripper_post_dispatch_" + decision.reason);
      cancel_goal_if_known<rclcpp_action::Client<GripperCommand>,
                           GripperDownstreamGoalHandle>(route->client,
                                                        downstream);
      fail("lease lost while dispatching downstream gripper");
      cleanup();
      return;
    }
  }

  auto result_future = route->client->async_get_result(downstream);
  while (rclcpp::ok() &&
         result_future.wait_for(50ms) != std::future_status::ready) {
    if (upstream->is_canceling()) {
      cancel_goal_if_known<rclcpp_action::Client<GripperCommand>,
                           GripperDownstreamGoalHandle>(route->client,
                                                        downstream);
    }
  }
  if (!rclcpp::ok()) {
    cleanup();
    return;
  }
  const auto wrapped = result_future.get();
  auto result = std::make_shared<LeasedGripper::Result>();
  if (wrapped.result) {
    result->state = wrapped.result->state;
    result->stalled = wrapped.result->stalled;
    result->reached_goal = wrapped.result->reached_goal;
  }
  if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
    upstream->succeed(result);
  } else if (wrapped.code == rclcpp_action::ResultCode::CANCELED &&
             upstream->is_canceling()) {
    upstream->canceled(result);
  } else {
    upstream->abort(result);
  }
  cleanup();
}

rclcpp_action::GoalResponse ExecutionManagerNode::external_trajectory_goal(
    const std::shared_ptr<ExternalActionRoute> &route,
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FollowTrajectory::Goal>) {
  route->source->has_command = true;
  if (!route->source->requested) {
    count_drop("external_source_not_requested");
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool expected = false;
  if (!route->goal_active.compare_exchange_strong(expected, true)) {
    count_drop("external_trajectory_already_active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ExecutionManagerNode::external_trajectory_cancel(
    const std::shared_ptr<ExternalUpstreamGoalHandle> &) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ExecutionManagerNode::external_trajectory_accepted(
    const std::shared_ptr<ExternalActionRoute> &route,
    const std::shared_ptr<ExternalUpstreamGoalHandle> &goal) {
  if (route->worker.joinable()) {
    route->worker.join();
  }
  route->worker = std::thread(
      [this, route, goal]() { execute_external_trajectory(route, goal); });
}

void ExecutionManagerNode::execute_external_trajectory(
    const std::shared_ptr<ExternalActionRoute> &route,
    const std::shared_ptr<ExternalUpstreamGoalHandle> &upstream) {
  route->source->active_actions.fetch_add(1);
  const auto cleanup = [this, &route]() {
    {
      std::lock_guard lock(route->mutex);
      route->downstream.reset();
    }
    route->goal_active.store(false);
    if (route->source->active_actions.fetch_sub(1) == 1 &&
        route->source->config.preempt &&
        route->source->config.activation_topic.empty() &&
        external_source_lease_snapshot(route->source).owned_by_em) {
      request_external_source_state(route->source, false);
    }
  };
  auto fail = [&upstream](const std::string &message) {
    auto result = std::make_shared<FollowTrajectory::Result>();
    result->error_code = FollowTrajectory::Result::INVALID_GOAL;
    result->error_string = message;
    upstream->abort(result);
  };

  route->source->has_command = true;
  if (!route->source->requested ||
      !wait_external_source_active(route->source, 10s)) {
    fail("failed to activate external source");
    cleanup();
    return;
  }
  if (!route->client->wait_for_action_server(5s)) {
    fail("leased trajectory ingress unavailable");
    cleanup();
    return;
  }

  const auto plain = upstream->get_goal();
  const auto lease = external_source_lease_snapshot(route->source);
  if (lease.lease_id.empty()) {
    fail("external source lease disappeared before trajectory dispatch");
    cleanup();
    return;
  }
  LeasedTrajectory::Goal goal;
  goal.header.stamp = now();
  goal.lease_id = lease.lease_id;
  goal.resource = route->input.resource;
  goal.trajectory = plain->trajectory;
  goal.path_tolerance = plain->path_tolerance;
  goal.goal_tolerance = plain->goal_tolerance;
  goal.goal_time_tolerance = plain->goal_time_tolerance;
  auto options = rclcpp_action::Client<LeasedTrajectory>::SendGoalOptions();
  options.feedback_callback = [upstream](const auto &, const auto feedback) {
    auto output = std::make_shared<FollowTrajectory::Feedback>();
    output->header = feedback->header;
    output->joint_names = feedback->joint_names;
    output->desired = feedback->desired;
    output->actual = feedback->actual;
    output->error = feedback->error;
    upstream->publish_feedback(output);
  };
  auto send_future = route->client->async_send_goal(goal, options);
  if (send_future.wait_for(5s) != std::future_status::ready) {
    fail("leased trajectory goal request timed out");
    cleanup();
    return;
  }
  auto downstream = send_future.get();
  if (!downstream) {
    fail("leased trajectory ingress rejected goal");
    cleanup();
    return;
  }
  {
    std::lock_guard lock(route->mutex);
    route->downstream = downstream;
  }

  auto result_future = route->client->async_get_result(downstream);
  while (rclcpp::ok() &&
         result_future.wait_for(50ms) != std::future_status::ready) {
    if (upstream->is_canceling()) {
      cancel_goal_if_known<rclcpp_action::Client<LeasedTrajectory>,
                           ExternalDownstreamGoalHandle>(route->client,
                                                         downstream);
    }
  }
  if (!rclcpp::ok()) {
    cleanup();
    return;
  }
  const auto wrapped = result_future.get();
  auto result = std::make_shared<FollowTrajectory::Result>();
  if (wrapped.result) {
    result->error_code = wrapped.result->error_code;
    result->error_string = wrapped.result->error_string;
  }
  if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
    upstream->succeed(result);
  } else if (wrapped.code == rclcpp_action::ResultCode::CANCELED &&
             upstream->is_canceling()) {
    upstream->canceled(result);
  } else {
    upstream->abort(result);
  }
  cleanup();
}

rclcpp_action::GoalResponse ExecutionManagerNode::external_gripper_goal(
    const std::shared_ptr<ExternalGripperActionRoute> &route,
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GripperCommand::Goal> goal) {
  if (goal->command.name.empty() || goal->command.position.empty() ||
      goal->command.name.size() != goal->command.position.size()) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool expected = false;
  if (!route->goal_active.compare_exchange_strong(expected, true)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ExecutionManagerNode::external_gripper_cancel(
    const std::shared_ptr<ExternalGripperUpstreamGoalHandle> &) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ExecutionManagerNode::external_gripper_accepted(
    const std::shared_ptr<ExternalGripperActionRoute> &route,
    const std::shared_ptr<ExternalGripperUpstreamGoalHandle> &goal) {
  if (route->worker.joinable()) {
    route->worker.join();
  }
  route->worker = std::thread(
      [this, route, goal]() { execute_external_gripper(route, goal); });
}

void ExecutionManagerNode::execute_external_gripper(
    const std::shared_ptr<ExternalGripperActionRoute> &route,
    const std::shared_ptr<ExternalGripperUpstreamGoalHandle> &upstream) {
  route->source->active_actions.fetch_add(1);
  const auto cleanup = [this, &route]() {
    {
      std::lock_guard lock(route->mutex);
      route->downstream.reset();
    }
    route->goal_active.store(false);
    if (route->source->active_actions.fetch_sub(1) == 1 &&
        route->source->config.preempt &&
        route->source->config.activation_topic.empty() &&
        external_source_lease_snapshot(route->source).owned_by_em) {
      request_external_source_state(route->source, false);
    }
  };
  const auto abort = [&upstream]() {
    upstream->abort(std::make_shared<GripperCommand::Result>());
  };
  route->source->has_command = true;
  if (!route->source->requested ||
      !wait_external_source_active(route->source, 10s) ||
      !route->client->wait_for_action_server(5s)) {
    abort();
    cleanup();
    return;
  }
  const auto lease = external_source_lease_snapshot(route->source);
  if (lease.lease_id.empty()) {
    abort();
    cleanup();
    return;
  }
  LeasedGripper::Goal goal;
  goal.header.stamp = now();
  goal.lease_id = lease.lease_id;
  goal.resource = route->input.resource;
  goal.command = upstream->get_goal()->command;
  auto options = rclcpp_action::Client<LeasedGripper>::SendGoalOptions();
  options.feedback_callback = [upstream](const auto &, const auto feedback) {
    auto output = std::make_shared<GripperCommand::Feedback>();
    output->state = feedback->state;
    upstream->publish_feedback(output);
  };
  auto send_future = route->client->async_send_goal(goal, options);
  if (send_future.wait_for(5s) != std::future_status::ready) {
    abort();
    cleanup();
    return;
  }
  auto downstream = send_future.get();
  if (!downstream) {
    abort();
    cleanup();
    return;
  }
  {
    std::lock_guard lock(route->mutex);
    route->downstream = downstream;
  }
  auto result_future = route->client->async_get_result(downstream);
  bool cancel_sent = false;
  while (rclcpp::ok() &&
         result_future.wait_for(50ms) != std::future_status::ready) {
    if (upstream->is_canceling() && !cancel_sent) {
      cancel_sent = true;
      cancel_goal_if_known<rclcpp_action::Client<LeasedGripper>,
                           ExternalGripperDownstreamGoalHandle>(route->client,
                                                                downstream);
    }
  }
  if (!rclcpp::ok()) {
    cleanup();
    return;
  }
  const auto wrapped = result_future.get();
  auto result = std::make_shared<GripperCommand::Result>();
  if (wrapped.result) {
    result->state = wrapped.result->state;
    result->stalled = wrapped.result->stalled;
    result->reached_goal = wrapped.result->reached_goal;
  }
  if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
    upstream->succeed(result);
  } else if (wrapped.code == rclcpp_action::ResultCode::CANCELED &&
             upstream->is_canceling()) {
    upstream->canceled(result);
  } else {
    upstream->abort(result);
  }
  cleanup();
}

void ExecutionManagerNode::publish_status() {
  execution_manager_interfaces::msg::AuthorityStatus status;
  status.header.stamp = now();
  const auto snapshot = authority_.snapshot();
  std::lock_guard observed_lock(observed_mutex_);
  for (const auto &[resource, allocation] : snapshot) {
    execution_manager_interfaces::msg::ResourceAuthority item;
    item.resource = resource;
    item.authority_state = static_cast<std::uint8_t>(allocation.state);
    item.lease_id = allocation.lease_id;
    item.source_role = allocation.source_role;
    item.source_instance = allocation.source_instance;
    item.command_contract = allocation.command_contract;
    item.requested_controller = allocation.requested_controller;
    const auto observed = observed_controllers_.find(resource);
    if (observed != observed_controllers_.end()) {
      item.observed_controllers = observed->second;
    }
    status.resources.push_back(std::move(item));
  }
  status_pub_->publish(status);
}

void ExecutionManagerNode::emit_event(const std::uint8_t type,
                                      const std::string &lease_id,
                                      const std::string &related_lease_id,
                                      const Allocation *identity,
                                      const std::vector<std::string> &resources,
                                      const std::string &reason) {
  execution_manager_interfaces::msg::AuthorityEvent event;
  event.header.stamp = now();
  event.event_id = reserve_id("event");
  event.type = type;
  event.lease_id = lease_id;
  event.related_lease_id = related_lease_id;
  if (identity) {
    event.source_role = identity->source_role;
    event.source_instance = identity->source_instance;
    for (const auto &[key, value] : identity->metadata) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = value;
      event.metadata.push_back(std::move(item));
    }
  }
  event.resources = resources;
  event.reason = reason;
  event_pub_->publish(event);
}

std::string ExecutionManagerNode::reserve_id(const std::string &prefix) {
  static std::random_device random;
  std::ostringstream stream;
  stream << prefix << '-' << std::hex << now().nanoseconds() << '-' << random()
         << '-' << id_sequence_.fetch_add(1);
  return stream.str();
}

void ExecutionManagerNode::count_drop(const std::string &reason) {
  std::lock_guard lock(diagnostics_mutex_);
  ++drop_counters_[reason];
}

} // namespace execution_manager
