#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "execution_manager/profile.hpp"

namespace execution_manager {

enum class AuthorityState : std::uint8_t {
  Unowned = 0,
  Transitioning = 1,
  Owned = 2,
  Fault = 3,
};

struct RequestedResource {
  std::string resource;
  std::string command_contract;
};

struct LeaseRequest {
  std::string lease_id;
  std::uint8_t source_role{0};
  std::string source_instance;
  std::vector<RequestedResource> resources;
  bool preempt{false};
  std::map<std::string, std::string> metadata;
};

struct Allocation {
  AuthorityState state{AuthorityState::Unowned};
  std::string lease_id;
  std::uint8_t source_role{0};
  std::string source_instance;
  std::string command_contract;
  std::string requested_controller;
  std::map<std::string, std::string> metadata;
};

using AllocationMap = std::map<std::string, Allocation>;
using ControllerRoutes = std::map<std::string, CommandCapability>;
using CancelHook = std::function<void(const std::vector<std::string> &)>;
using SwitchHook = std::function<void(const ControllerRoutes &)>;

struct TransitionResult {
  bool success{false};
  std::string message;
  std::vector<std::string> displaced_lease_ids;
  std::vector<std::string> affected_resources;
};

class AuthorityManager {
public:
  class RouteGuard {
  public:
    RouteGuard(std::mutex &mutex, const AllocationMap &allocations);
    const AllocationMap &allocations() const { return allocations_; }

  private:
    std::unique_lock<std::mutex> lock_;
    const AllocationMap &allocations_;
  };

  explicit AuthorityManager(const ExecutionProfile &profile);

  TransitionResult claim(const LeaseRequest &request, const CancelHook &cancel,
                         const SwitchHook &switch_controllers);
  TransitionResult release(const std::string &lease_id,
                           const CancelHook &cancel);
  AllocationMap snapshot() const;
  RouteGuard route_guard() const;

private:
  ControllerRoutes validate_request(const LeaseRequest &request) const;
  void set_state(const std::vector<std::string> &resources,
                 AuthorityState state);

  const ExecutionProfile &profile_;
  mutable std::mutex transition_mutex_;
  mutable std::mutex route_mutex_;
  AllocationMap allocations_;
};

} // namespace execution_manager
