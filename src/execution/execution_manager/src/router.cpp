#include "execution_manager/router.hpp"

#include <stdexcept>

namespace execution_manager {

CommandRouter::CommandRouter(const double max_command_age_s)
    : max_command_age_s_(max_command_age_s) {
  if (max_command_age_s <= 0.0) {
    throw std::invalid_argument("max_command_age_s must be positive");
  }
}

CommandDecision CommandRouter::decide(
    const std::string &lease_id, const std::string &resource,
    const std::string &command_contract, const AllocationMap &allocations,
    const double now_s, const std::optional<double> stamp_s) const {
  const auto item = allocations.find(resource);
  if (item == allocations.end()) {
    return {false, "unknown_resource"};
  }
  const auto &allocation = item->second;
  if (allocation.state != AuthorityState::Owned) {
    return {false, allocation.state == AuthorityState::Fault
                       ? "resource_fault"
                       : "resource_unowned_or_transitioning"};
  }
  if (allocation.lease_id != lease_id) {
    return {false, "stale_or_foreign_lease"};
  }
  if (allocation.command_contract != command_contract) {
    return {false, "command_contract_not_claimed"};
  }
  if (!stamp_s || *stamp_s <= 0.0) {
    return {false, "command_has_no_admission_timestamp"};
  }
  const auto age_s = now_s - *stamp_s;
  if (age_s > max_command_age_s_) {
    return {false, "stale_command"};
  }
  if (age_s < -max_command_age_s_) {
    return {false, "future_command"};
  }
  return {true, "accepted"};
}

std::string execution_trace_topic(const std::uint8_t source_role,
                                  const std::string &resource,
                                  const std::string &command_contract) {
  return "/execution_trace/" + source_role_token(source_role) + "/" + resource +
         "/" + command_contract;
}

} // namespace execution_manager
