// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// Unit tests for the pure pose-chunk helpers (chunk_is_fresh, sample_pose_chunk).
// These cover the staleness contract added on this branch: a chunk stays active
// through its own time horizon plus a stale-timeout grace window, then expires.

#include <gtest/gtest.h>

#include <cmath>

#include "manipulation_position_controllers/task_space/kinematics/pose_chunk_buffer.hpp"

namespace ts = manipulation_position_controllers::task_space;

namespace
{
/// Build a chunk of `n` frames starting at t0 spaced by dt, with a position
/// that ramps along +x so interpolation is observable.
ts::PoseChunk make_chunk(size_t n, double t0, double dt)
{
  ts::PoseChunk chunk;
  chunk.size = n;
  chunk.sequence = 1;
  chunk.receive_time_s = t0;
  for (size_t i = 0; i < n; ++i) {
    auto & f = chunk.frames[i];
    f.time_s = t0 + static_cast<double>(i) * dt;
    f.px = static_cast<double>(i);
    f.py = 0.0;
    f.pz = 0.0;
    f.qx = 0.0; f.qy = 0.0; f.qz = 0.0; f.qw = 1.0;
  }
  return chunk;
}
}  // namespace

TEST(ChunkIsFresh, EmptyChunkIsNeverFresh)
{
  ts::PoseChunk chunk;  // size == 0
  EXPECT_FALSE(ts::chunk_is_fresh(chunk, 0.0, 0.2));
  EXPECT_FALSE(ts::chunk_is_fresh(chunk, 100.0, 0.2));
}

TEST(ChunkIsFresh, FreshWithinHorizon)
{
  const auto chunk = make_chunk(/*n=*/5, /*t0=*/10.0, /*dt=*/0.02);  // ends 10.08
  EXPECT_TRUE(ts::chunk_is_fresh(chunk, 10.00, 0.2));
  EXPECT_TRUE(ts::chunk_is_fresh(chunk, 10.05, 0.2));
  EXPECT_TRUE(ts::chunk_is_fresh(chunk, 10.08, 0.2));
}

TEST(ChunkIsFresh, FreshWithinGraceWindowPastHorizon)
{
  const auto chunk = make_chunk(5, 10.0, 0.02);  // ends 10.08
  // Within stale_timeout grace past the last frame.
  EXPECT_TRUE(ts::chunk_is_fresh(chunk, 10.08 + 0.10, 0.2));
  EXPECT_TRUE(ts::chunk_is_fresh(chunk, 10.08 + 0.20, 0.2));
}

TEST(ChunkIsFresh, StaleBeyondGraceWindow)
{
  const auto chunk = make_chunk(5, 10.0, 0.02);  // ends 10.08
  EXPECT_FALSE(ts::chunk_is_fresh(chunk, 10.08 + 0.2001, 0.2));
  EXPECT_FALSE(ts::chunk_is_fresh(chunk, 100.0, 0.2));
}

TEST(SamplePoseChunk, EmptyReturnsFalse)
{
  ts::PoseChunk chunk;
  double px, py, pz, qx, qy, qz, qw;
  EXPECT_FALSE(ts::sample_pose_chunk(chunk, 0.0, px, py, pz, qx, qy, qz, qw));
}

TEST(SamplePoseChunk, HoldsFirstFrameBeforeWindow)
{
  const auto chunk = make_chunk(5, 10.0, 0.02);
  double px, py, pz, qx, qy, qz, qw;
  ASSERT_TRUE(ts::sample_pose_chunk(chunk, 9.0, px, py, pz, qx, qy, qz, qw));
  EXPECT_DOUBLE_EQ(px, 0.0);  // first frame px
}

TEST(SamplePoseChunk, HoldsLastFrameAfterWindow)
{
  const auto chunk = make_chunk(5, 10.0, 0.02);
  double px, py, pz, qx, qy, qz, qw;
  ASSERT_TRUE(ts::sample_pose_chunk(chunk, 999.0, px, py, pz, qx, qy, qz, qw));
  EXPECT_DOUBLE_EQ(px, 4.0);  // last frame px (index 4)
}

TEST(SamplePoseChunk, InterpolatesWithinWindow)
{
  const auto chunk = make_chunk(5, 10.0, 0.02);
  double px, py, pz, qx, qy, qz, qw;
  // Halfway between frame 0 (t=10.00, px=0) and frame 1 (t=10.02, px=1).
  ASSERT_TRUE(ts::sample_pose_chunk(chunk, 10.01, px, py, pz, qx, qy, qz, qw));
  EXPECT_NEAR(px, 0.5, 1e-9);
  // Quaternion stays unit-norm.
  EXPECT_NEAR(std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw), 1.0, 1e-9);
}
