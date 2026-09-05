#pragma once

#include <optional>
#include <string>

#include "execution_manager/selector.hpp"

namespace execution_manager {

struct CommandDecision {
  bool accepted{false};
  std::string reason;
};

class CommandRouter {
public:
  explicit CommandRouter(double max_command_age_s);

  CommandDecision decide(const std::string &lease_id,
                         const std::string &resource,
                         const std::string &command_contract,
                         const AllocationMap &allocations, double now_s,
                         std::optional<double> stamp_s) const;

private:
  double max_command_age_s_;
};

std::string execution_trace_topic(std::uint8_t source_role,
                                  const std::string &resource,
                                  const std::string &command_contract);

} // namespace execution_manager
