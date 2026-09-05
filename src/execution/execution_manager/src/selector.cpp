#include "execution_manager/selector.hpp"

#include <algorithm>
#include <set>
#include <stdexcept>

namespace execution_manager {
namespace {

std::vector<std::string> sorted(std::set<std::string> values) {
  return {values.begin(), values.end()};
}

} // namespace

AuthorityManager::RouteGuard::RouteGuard(std::mutex &mutex,
                                         const AllocationMap &allocations)
    : lock_(mutex), allocations_(allocations) {}

AuthorityManager::AuthorityManager(const ExecutionProfile &profile)
    : profile_(profile) {
  for (const auto &[name, resource] : profile.resources) {
    (void)resource;
    allocations_.emplace(name, Allocation{});
  }
}

ControllerRoutes
AuthorityManager::validate_request(const LeaseRequest &request) const {
  if (request.lease_id.empty()) {
    throw std::invalid_argument("lease_id must not be empty");
  }
  (void)source_role_token(request.source_role);
  if (request.source_instance.empty()) {
    throw std::invalid_argument("source_instance must not be empty");
  }
  if (request.resources.empty()) {
    throw std::invalid_argument("claim must contain at least one resource");
  }
  ControllerRoutes routes;
  for (const auto &requested : request.resources) {
    const auto resource = profile_.resources.find(requested.resource);
    if (resource == profile_.resources.end()) {
      throw std::invalid_argument("unknown resource '" + requested.resource +
                                  "'");
    }
    const auto capability =
        resource->second.commands.find(requested.command_contract);
    if (capability == resource->second.commands.end()) {
      throw std::invalid_argument("resource '" + requested.resource +
                                  "' does not support command contract '" +
                                  requested.command_contract + "'");
    }
    if (!routes.emplace(requested.resource, capability->second).second) {
      throw std::invalid_argument("duplicate requested resource '" +
                                  requested.resource + "'");
    }
  }
  return routes;
}

void AuthorityManager::set_state(const std::vector<std::string> &resources,
                                 const AuthorityState state) {
  for (const auto &resource : resources) {
    auto &allocation = allocations_.at(resource);
    allocation = Allocation{};
    allocation.state = state;
  }
}

TransitionResult AuthorityManager::claim(
    const LeaseRequest &request, const CancelHook &cancel,
    const SwitchHook &switch_controllers) {
  std::lock_guard transition_lock(transition_mutex_);
  const auto routes = validate_request(request);
  std::set<std::string> requested_resources;
  std::set<std::string> displaced_leases;
  std::set<std::string> affected_resources;
  {
    std::lock_guard route_lock(route_mutex_);
    for (const auto &[resource, capability] : routes) {
      (void)capability;
      requested_resources.insert(resource);
      const auto &allocation = allocations_.at(resource);
      if (allocation.state == AuthorityState::Fault && !request.preempt) {
        return {false, "resource_fault_requires_explicit_preempt", {},
                {resource}};
      }
      if (allocation.state == AuthorityState::Owned) {
        displaced_leases.insert(allocation.lease_id);
      }
    }
    if (!displaced_leases.empty() && !request.preempt) {
      return {false, "resources_busy_preempt_required", {},
              sorted(requested_resources)};
    }
    for (const auto &[resource, allocation] : allocations_) {
      if (displaced_leases.count(allocation.lease_id)) {
        affected_resources.insert(resource);
      }
    }
    affected_resources.insert(requested_resources.begin(),
                              requested_resources.end());
    set_state(sorted(affected_resources), AuthorityState::Transitioning);
  }

  const auto affected = sorted(affected_resources);
  const auto displaced = sorted(displaced_leases);
  try {
    if (!displaced.empty() && cancel) {
      cancel(affected);
    }
    if (switch_controllers) {
      switch_controllers(routes);
    }
  } catch (const std::exception &error) {
    std::lock_guard route_lock(route_mutex_);
    set_state(affected, AuthorityState::Fault);
    return {false, error.what(), displaced, affected};
  }

  {
    std::lock_guard route_lock(route_mutex_);
    set_state(affected, AuthorityState::Unowned);
    for (const auto &[resource, capability] : routes) {
      allocations_[resource] = Allocation{
          AuthorityState::Owned, request.lease_id, request.source_role,
          request.source_instance, capability.command_contract,
          capability.controller, request.metadata};
    }
  }
  return {true, "claimed", displaced, affected};
}

TransitionResult AuthorityManager::release(const std::string &lease_id,
                                           const CancelHook &cancel) {
  std::lock_guard transition_lock(transition_mutex_);
  if (lease_id.empty()) {
    return {false, "lease_id must not be empty", {}, {}};
  }
  std::vector<std::string> resources;
  {
    std::lock_guard route_lock(route_mutex_);
    for (const auto &[resource, allocation] : allocations_) {
      if (allocation.state == AuthorityState::Owned &&
          allocation.lease_id == lease_id) {
        resources.push_back(resource);
      }
    }
    if (resources.empty()) {
      return {false, "lease_not_active", {}, {}};
    }
    set_state(resources, AuthorityState::Transitioning);
  }
  try {
    if (cancel) {
      cancel(resources);
    }
  } catch (const std::exception &error) {
    std::lock_guard route_lock(route_mutex_);
    set_state(resources, AuthorityState::Fault);
    return {false, error.what(), {}, resources};
  }
  {
    std::lock_guard route_lock(route_mutex_);
    set_state(resources, AuthorityState::Unowned);
  }
  return {true, "released", {}, resources};
}

AllocationMap AuthorityManager::snapshot() const {
  std::lock_guard route_lock(route_mutex_);
  return allocations_;
}

AuthorityManager::RouteGuard AuthorityManager::route_guard() const {
  return RouteGuard(route_mutex_, allocations_);
}

} // namespace execution_manager
