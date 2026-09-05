#pragma once

#include <cstdint>
#include <map>
#include <string>

namespace execution_manager {

struct CommandCapability {
  std::string command_contract;
  std::string controller;
  std::string implementation;
  std::string downstream_endpoint;
  std::string heartbeat_endpoint;
  bool is_action{false};
};

struct ResourceConfig {
  std::string name;
  std::string controller_manager{"/controller_manager"};
  std::map<std::string, CommandCapability> commands;
};

struct SourceInputConfig {
  std::string resource;
  std::string command_contract;
  std::string endpoint;
  bool is_action{false};
};

struct ExternalSourceConfig {
  std::string name;
  std::uint8_t source_role{0};
  bool preempt{false};
  std::string activation_topic;
  std::map<std::string, SourceInputConfig> inputs;
};

struct ExecutionProfile {
  std::string name;
  double max_command_age_s{0.25};
  std::map<std::string, ResourceConfig> resources;
  std::map<std::string, ExternalSourceConfig> sources;

  static ExecutionProfile from_yaml(const std::string &path);
};

std::string source_role_token(std::uint8_t source_role);
std::uint8_t source_role_value(const std::string &source_role);
std::string ingress_endpoint(std::uint8_t source_role,
                             const std::string &resource,
                             const std::string &command_contract,
                             bool is_action);

} // namespace execution_manager
