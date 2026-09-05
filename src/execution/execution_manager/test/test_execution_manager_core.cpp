#include <filesystem>
#include <stdexcept>

#include <gtest/gtest.h>

#include "execution_manager/profile.hpp"
#include "execution_manager/router.hpp"
#include "execution_manager/selector.hpp"

namespace em = execution_manager;

namespace {

std::string
profile_path(const std::string &embodiment,
             const std::string &filename = "execution_manager.yaml") {
  return (std::filesystem::path(WORKSPACE_ROOT) / "src" / "bringup" /
          embodiment / "workstation_launch" / "config" / filename)
      .string();
}

std::string test_profile_path(const std::string &filename) {
  return (std::filesystem::path(WORKSPACE_ROOT) / "src" / "execution" /
          "execution_manager" / "test" / filename)
      .string();
}

em::ExecutionProfile test_profile() {
  em::ExecutionProfile profile;
  profile.name = "test";
  for (const auto &resource_name : {"left_arm", "right_arm"}) {
    const std::string name{resource_name};
    em::ResourceConfig resource;
    resource.name = name;
    resource.commands.emplace(
        "joint_reference",
        em::CommandCapability{"joint_reference", name + "_jspc", "test",
                              "/execution/" + name + "/joint_reference", "",
                              false});
    resource.commands.emplace(
        "joint_trajectory",
        em::CommandCapability{"joint_trajectory", name + "_jtc", "test",
                              "/execution/" + name + "/follow_joint_trajectory",
                              "", true});
    profile.resources.emplace(name, std::move(resource));
  }
  return profile;
}

em::LeaseRequest request(std::string lease_id, std::uint8_t role,
                         std::vector<std::string> resources,
                         const bool preempt = false) {
  em::LeaseRequest result;
  result.lease_id = std::move(lease_id);
  result.source_role = role;
  result.source_instance = "test-source";
  result.preempt = preempt;
  for (auto &resource : resources) {
    result.resources.push_back({std::move(resource), "joint_reference"});
  }
  return result;
}

} // namespace

TEST(Profile, LoadsExecutionCapabilitiesWithoutProviders) {
  const std::vector<std::string> paths{profile_path("marvin_manipulation"),
                                       profile_path("franka_manipulation")};
  for (const auto &path : paths) {
    const auto profile = em::ExecutionProfile::from_yaml(path);
    EXPECT_FALSE(profile.name.empty());
    EXPECT_FALSE(profile.resources.empty());
    EXPECT_GT(profile.max_command_age_s, 0.0);
  }
  EXPECT_EQ(em::ingress_endpoint(1, "left_arm", "joint_reference", false),
            "/execution_manager/ingress/policy/left_arm/joint_reference");
  EXPECT_EQ(
      em::ingress_endpoint(3, "left_arm", "joint_trajectory", true),
      "/execution_manager/ingress/planner/left_arm/follow_joint_trajectory");
  EXPECT_EQ(em::ingress_endpoint(3, "left_gripper", "gripper_command", true),
            "/execution_manager/ingress/planner/left_gripper/gripper_command");
  EXPECT_EQ(em::source_role_value("MEMORY"), 4);
  EXPECT_EQ(em::source_role_token(4), "memory");
}

TEST(Profile, LoadsTrustedExternalSources) {
  const auto profile = em::ExecutionProfile::from_yaml(
      test_profile_path("external_source_single_flight.yaml"));
  const auto &policy = profile.sources.at("DummyPolicy");
  EXPECT_EQ(policy.source_role, 1);
  EXPECT_FALSE(policy.preempt);
  EXPECT_TRUE(policy.activation_topic.empty());
  EXPECT_EQ(policy.inputs.at("arm").command_contract, "joint_reference");

  const auto &teleop = profile.sources.at("Quest3Teleop");
  EXPECT_EQ(teleop.source_role, 2);
  EXPECT_TRUE(teleop.preempt);
  EXPECT_EQ(teleop.activation_topic, "/test_sources/quest/clutch");
  EXPECT_EQ(teleop.inputs.at("arm").command_contract, "pose_reference");
  EXPECT_EQ(teleop.inputs.at("gripper").command_contract,
            "joint_reference");
}

TEST(Authority, BusyResourceRequiresExplicitPreempt) {
  const auto profile = test_profile();
  em::AuthorityManager manager(profile);
  EXPECT_TRUE(manager.claim(request("P1", 1, {"left_arm"}), {}, {}).success);
  const auto rejected = manager.claim(request("T1", 2, {"left_arm"}), {}, {});
  EXPECT_FALSE(rejected.success);
  EXPECT_EQ(rejected.message, "resources_busy_preempt_required");
  EXPECT_EQ(manager.snapshot().at("left_arm").lease_id, "P1");
}

TEST(Authority, PartialPreemptInvalidatesEntireOldLease) {
  const auto profile = test_profile();
  em::AuthorityManager manager(profile);
  EXPECT_TRUE(manager.claim(request("P1", 1, {"left_arm", "right_arm"}), {}, {})
                  .success);
  std::vector<std::string> canceled;
  const auto takeover = manager.claim(
      request("T1", 2, {"left_arm"}, true),
      [&canceled](const auto &resources) { canceled = resources; }, {});
  ASSERT_TRUE(takeover.success);
  EXPECT_EQ(canceled, (std::vector<std::string>{"left_arm", "right_arm"}));
  const auto state = manager.snapshot();
  EXPECT_EQ(state.at("left_arm").lease_id, "T1");
  EXPECT_EQ(state.at("left_arm").state, em::AuthorityState::Owned);
  EXPECT_EQ(state.at("right_arm").state, em::AuthorityState::Unowned);
  EXPECT_TRUE(state.at("right_arm").lease_id.empty());
}

TEST(Authority, DisjointLeasesCanRemainActive) {
  const auto profile = test_profile();
  em::AuthorityManager manager(profile);
  EXPECT_TRUE(manager.claim(request("P1", 1, {"left_arm"}), {}, {}).success);
  EXPECT_TRUE(manager.claim(request("T1", 2, {"right_arm"}), {}, {}).success);
  const auto state = manager.snapshot();
  EXPECT_EQ(state.at("left_arm").lease_id, "P1");
  EXPECT_EQ(state.at("right_arm").lease_id, "T1");
}

TEST(Authority, SwitchFailureFencesOldAndFaultsAffectedResources) {
  const auto profile = test_profile();
  em::AuthorityManager manager(profile);
  EXPECT_TRUE(manager.claim(request("P1", 1, {"left_arm", "right_arm"}), {}, {})
                  .success);
  const auto failed =
      manager.claim(request("T1", 2, {"left_arm"}, true), {}, [](const auto &) {
        throw std::runtime_error("switch failed");
      });
  EXPECT_FALSE(failed.success);
  const auto state = manager.snapshot();
  EXPECT_EQ(state.at("left_arm").state, em::AuthorityState::Fault);
  EXPECT_EQ(state.at("right_arm").state, em::AuthorityState::Fault);
  EXPECT_TRUE(state.at("left_arm").lease_id.empty());
  EXPECT_TRUE(state.at("right_arm").lease_id.empty());
}

TEST(Authority, ReleaseUsesLeaseIdentity) {
  const auto profile = test_profile();
  em::AuthorityManager manager(profile);
  EXPECT_TRUE(manager.claim(request("P1", 1, {"left_arm"}), {}, {}).success);
  EXPECT_FALSE(manager.release("old-policy-lease", {}).success);
  EXPECT_EQ(manager.snapshot().at("left_arm").lease_id, "P1");
  EXPECT_TRUE(manager.release("P1", {}).success);
  EXPECT_EQ(manager.snapshot().at("left_arm").state,
            em::AuthorityState::Unowned);
}

TEST(Router, RejectsOldLeaseAfterSameRoleReacquires) {
  em::CommandRouter router(0.25);
  em::AllocationMap allocations{{
      "left_arm",
      {em::AuthorityState::Owned,
       "P2",
       1,
       "policy-v2",
       "joint_reference",
       "left_arm_jspc",
       {}},
  }};
  EXPECT_TRUE(
      router
          .decide("P2", "left_arm", "joint_reference", allocations, 10.1, 10.0)
          .accepted);
  EXPECT_EQ(
      router
          .decide("P1", "left_arm", "joint_reference", allocations, 10.1, 10.0)
          .reason,
      "stale_or_foreign_lease");
  EXPECT_EQ(
      router
          .decide("P2", "left_arm", "joint_reference", allocations, 10.5, 10.0)
          .reason,
      "stale_command");
}
