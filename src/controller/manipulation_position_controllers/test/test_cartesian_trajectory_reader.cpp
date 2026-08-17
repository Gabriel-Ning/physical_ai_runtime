// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/cartesian_point.hpp>
#include <moveit_msgs/msg/cartesian_trajectory.hpp>
#include <moveit_msgs/msg/cartesian_trajectory_point.hpp>

#include "manipulation_position_controllers/task_space/kinematics/cartesian_trajectory_reader.hpp"

namespace ts = manipulation_position_controllers::task_space;

namespace
{

moveit_msgs::msg::CartesianTrajectoryPoint make_point(
  double x, double y, double z, double t_sec)
{
  moveit_msgs::msg::CartesianTrajectoryPoint pt;
  pt.point.pose.position.x = x;
  pt.point.pose.position.y = y;
  pt.point.pose.position.z = z;
  pt.point.pose.orientation.w = 1.0;
  const int64_t ns = static_cast<int64_t>(t_sec * 1e9);
  pt.time_from_start.sec = static_cast<int32_t>(ns / 1000000000LL);
  pt.time_from_start.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return pt;
}

}  // namespace

TEST(CartesianTrajectoryReader, UntimedUsesFrameDt)
{
  moveit_msgs::msg::CartesianTrajectory msg;
  msg.header.stamp.sec = 10;
  msg.points.push_back(make_point(0.1, 0.0, 0.0, 0.0));
  msg.points.push_back(make_point(0.2, 0.0, 0.0, 0.0));

  ts::PoseChunk chunk;
  ts::CartesianTrajectoryReaderConfig cfg;
  cfg.receive_time_s = 10.0;
  cfg.untimed_frame_dt_s = 0.05;
  ASSERT_TRUE(ts::read_cartesian_trajectory(msg, cfg, chunk));
  ASSERT_EQ(chunk.size, 2u);
  EXPECT_DOUBLE_EQ(chunk.frames[0].time_s, 10.0);
  EXPECT_DOUBLE_EQ(chunk.frames[1].time_s, 10.05);
  EXPECT_DOUBLE_EQ(chunk.frames[1].px, 0.2);
}

TEST(CartesianTrajectoryReader, TimedUsesHeaderPlusTimeFromStart)
{
  moveit_msgs::msg::CartesianTrajectory msg;
  msg.header.stamp.sec = 5;
  msg.points.push_back(make_point(0.0, 0.0, 0.0, 0.0));
  msg.points.push_back(make_point(0.1, 0.0, 0.0, 0.1));
  msg.points.push_back(make_point(0.2, 0.0, 0.0, 0.25));

  ts::PoseChunk chunk;
  ts::CartesianTrajectoryReaderConfig cfg;
  cfg.receive_time_s = 5.0;
  ASSERT_TRUE(ts::read_cartesian_trajectory(msg, cfg, chunk));
  ASSERT_EQ(chunk.size, 3u);
  EXPECT_DOUBLE_EQ(chunk.frames[0].time_s, 5.0);
  EXPECT_DOUBLE_EQ(chunk.frames[1].time_s, 5.1);
  EXPECT_DOUBLE_EQ(chunk.frames[2].time_s, 5.25);
}

TEST(CartesianTrajectoryReader, RejectsNonIncreasingTimed)
{
  moveit_msgs::msg::CartesianTrajectory msg;
  msg.header.stamp.sec = 1;
  msg.points.push_back(make_point(0.0, 0.0, 0.0, 0.2));
  msg.points.push_back(make_point(0.1, 0.0, 0.0, 0.1));

  ts::PoseChunk chunk;
  ts::CartesianTrajectoryReaderConfig cfg;
  cfg.receive_time_s = 1.0;
  EXPECT_FALSE(ts::read_cartesian_trajectory(msg, cfg, chunk));
}

TEST(CartesianTrajectoryReader, RespectsMaxPosePoints)
{
  moveit_msgs::msg::CartesianTrajectory msg;
  msg.header.stamp.sec = 1;
  for (int i = 0; i < 5; ++i) {
    msg.points.push_back(make_point(0.1 * i, 0.0, 0.0, 0.0));
  }
  ts::PoseChunk chunk;
  ts::CartesianTrajectoryReaderConfig cfg;
  cfg.receive_time_s = 0.0;
  cfg.max_pose_points = 2;
  ASSERT_TRUE(ts::read_cartesian_trajectory(msg, cfg, chunk));
  EXPECT_EQ(chunk.size, 2u);
}

TEST(CartesianTrajectoryReader, RejectsZeroStampByDefault)
{
  moveit_msgs::msg::CartesianTrajectory msg;
  msg.points.push_back(make_point(0.1, 0.0, 0.0, 0.0));

  ts::PoseChunk chunk;
  ts::CartesianTrajectoryReaderConfig cfg;
  cfg.receive_time_s = 10.0;
  EXPECT_FALSE(ts::read_cartesian_trajectory(msg, cfg, chunk));
}

TEST(CartesianTrajectoryReader, AllowsZeroStampWhenExplicitlyConfigured)
{
  moveit_msgs::msg::CartesianTrajectory msg;
  msg.points.push_back(make_point(0.1, 0.0, 0.0, 0.0));

  ts::PoseChunk chunk;
  ts::CartesianTrajectoryReaderConfig cfg;
  cfg.receive_time_s = 10.0;
  cfg.reject_zero_stamped_references = false;
  ASSERT_TRUE(ts::read_cartesian_trajectory(msg, cfg, chunk));
  EXPECT_DOUBLE_EQ(chunk.frames[0].time_s, 10.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
