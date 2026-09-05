#include "execution_manager/profile.hpp"

#include <set>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace execution_manager {
namespace {

constexpr std::uint8_t kPolicy = 1;
constexpr std::uint8_t kTeleop = 2;
constexpr std::uint8_t kPlanner = 3;
constexpr std::uint8_t kMemory = 4;

void add_capability(ResourceConfig &resource, const std::string &command,
                    const YAML::Node &controller, const std::string &endpoint,
                    const bool is_action) {
  if (resource.commands.count(command)) {
    throw std::runtime_error("resource '" + resource.name +
                             "' has duplicate command contract '" + command +
                             "'");
  }
  CommandCapability capability;
  capability.command_contract = command;
  capability.controller = controller["name"].as<std::string>();
  capability.implementation = controller["implementation"].as<std::string>("");
  capability.downstream_endpoint = endpoint;
  capability.is_action = is_action;
  const auto topics = controller["ros_topics"];
  if (topics && topics["trajectory_guard_heartbeat"]) {
    capability.heartbeat_endpoint =
        topics["trajectory_guard_heartbeat"].as<std::string>();
  }
  resource.commands.emplace(command, std::move(capability));
}

} // namespace

ExecutionProfile ExecutionProfile::from_yaml(const std::string &path) {
  const auto root = YAML::LoadFile(path);
  ExecutionProfile profile;
  profile.name = root["metadata"]["name"].as<std::string>();
  profile.max_command_age_s = root["max_command_age_s"].as<double>(0.25);
  if (profile.max_command_age_s <= 0.0) {
    throw std::runtime_error("max_command_age_s must be positive");
  }

  const auto groups = root["groups"];
  if (!groups || !groups.IsMap()) {
    throw std::runtime_error("profile has no groups map: " + path);
  }
  std::set<std::string> managers;
  for (const auto &item : groups) {
    ResourceConfig resource;
    resource.name = item.first.as<std::string>();
    const auto group = item.second;
    resource.controller_manager =
        group["controller_manager"].as<std::string>("/controller_manager");
    managers.insert(resource.controller_manager);
    const auto controllers = group["controllers"];
    if (!controllers || !controllers.IsMap()) {
      throw std::runtime_error("resource '" + resource.name +
                               "' has no controllers map");
    }
    for (const auto &controller_item : controllers) {
      const auto controller = controller_item.second;
      const auto topics = controller["ros_topics"];
      if (topics && topics.IsMap()) {
        for (const auto &topic : topics) {
          const auto command = topic.first.as<std::string>();
          if (command == "trajectory_guard_heartbeat") {
            continue;
          }
          add_capability(resource, command, controller,
                         topic.second.as<std::string>(), false);
        }
      }
      const auto actions = controller["ros_actions"];
      if (actions && actions["follow_joint_trajectory"]) {
        add_capability(resource, "joint_trajectory", controller,
                       actions["follow_joint_trajectory"].as<std::string>(),
                       true);
      }
      if (actions && actions["gripper_command"]) {
        add_capability(resource, "gripper_command", controller,
                       actions["gripper_command"].as<std::string>(), true);
      }
    }
    if (resource.commands.empty()) {
      throw std::runtime_error("resource '" + resource.name +
                               "' exposes no command capability");
    }
    profile.resources.emplace(resource.name, std::move(resource));
  }
  if (managers.size() != 1) {
    throw std::runtime_error(
        "execution_manager currently supports exactly one controller_manager");
  }

  const auto sources = root["sources"];
  if (sources) {
    if (!sources.IsMap()) {
      throw std::runtime_error("profile sources must be a map");
    }
    for (const auto &item : sources) {
      ExternalSourceConfig source;
      source.name = item.first.as<std::string>();
      const auto config = item.second;
      source.source_role =
          source_role_value(config["source_role"].as<std::string>());
      source.preempt = config["preempt"].as<bool>(false);
      source.activation_topic = config["activation_topic"].as<std::string>("");
      const auto inputs = config["inputs"];
      if (!inputs || !inputs.IsMap()) {
        throw std::runtime_error("source '" + source.name +
                                 "' has no inputs map");
      }
      for (const auto &input_item : inputs) {
        SourceInputConfig input;
        input.resource = input_item.first.as<std::string>();
        const auto input_config = input_item.second;
        input.command_contract =
            input_config["command_contract"].as<std::string>();
        const auto topic = input_config["topic"];
        const auto action = input_config["action"];
        if (static_cast<bool>(topic) == static_cast<bool>(action)) {
          throw std::runtime_error(
              "source '" + source.name + "' input '" + input.resource +
              "' must declare exactly one of topic or action");
        }
        input.is_action = static_cast<bool>(action);
        input.endpoint = (input.is_action ? action : topic).as<std::string>();
        const auto resource = profile.resources.find(input.resource);
        if (resource == profile.resources.end() ||
            !resource->second.commands.count(input.command_contract)) {
          throw std::runtime_error(
              "source '" + source.name + "' has unsupported input '" +
              input.resource + ":" + input.command_contract + "'");
        }
        const auto &capability =
            resource->second.commands.at(input.command_contract);
        if (capability.is_action != input.is_action) {
          throw std::runtime_error(
              "source '" + source.name + "' input '" + input.resource +
              "' transport does not match command capability");
        }
        source.inputs.emplace(input.resource, std::move(input));
      }
      if (!source.activation_topic.empty() && !source.preempt) {
        throw std::runtime_error("source '" + source.name +
                                 "' activation_topic requires preempt=true");
      }
      profile.sources.emplace(source.name, std::move(source));
    }
  }
  return profile;
}

std::string source_role_token(const std::uint8_t source_role) {
  if (source_role == kPolicy) {
    return "policy";
  }
  if (source_role == kTeleop) {
    return "teleop";
  }
  if (source_role == kPlanner) {
    return "planner";
  }
  if (source_role == kMemory) {
    return "memory";
  }
  throw std::invalid_argument("unknown source role");
}

std::uint8_t source_role_value(const std::string &source_role) {
  if (source_role == "POLICY") {
    return kPolicy;
  }
  if (source_role == "TELEOP") {
    return kTeleop;
  }
  if (source_role == "PLANNER") {
    return kPlanner;
  }
  if (source_role == "MEMORY") {
    return kMemory;
  }
  throw std::invalid_argument("unknown source role '" + source_role + "'");
}

std::string ingress_endpoint(const std::uint8_t source_role,
                             const std::string &resource,
                             const std::string &command_contract,
                             const bool is_action) {
  const auto suffix = is_action && command_contract == "joint_trajectory"
                          ? "follow_joint_trajectory"
                          : command_contract;
  return "/execution_manager/ingress/" + source_role_token(source_role) + "/" +
         resource + "/" + suffix;
}

} // namespace execution_manager
