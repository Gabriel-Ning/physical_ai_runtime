# Dataset tools

This toolbox converts episodes from the workspace's independent C++
`episode_recorder` into training datasets:

```text
episode_recorder (C++ MCAP)
  -> McapReader (rosbag2_py MCAP access used by the validated converter)
  -> ProfileEpisodeReader (indexed timeline + lazy per-camera decoding)
  -> native LeRobotDataset writer
```

`DatasetContract` and policy inference both resolve names, shapes, camera
sources, action groups, and frequency from the same embodiment Profile. New
datasets include `policy_contract.json` so checkpoints can later prove vector
element order rather than validating dimensions alone.

The recorder preserves each ROS stream at its native frequency. Conversion is
the only synchronization boundary: the output timeline is the common camera
time range, images and state use nearest samples, and actions use causal latest
commands with measured-state hold before the first command. Numeric samples and
image timestamps are indexed once per episode; selected camera frames are then
decoded lazily and released after LeRobot writes each frame.

Offline conversion:

```bash
python apps/convert_lerobot.py \
  --profile apps/profiles/piper_bimanual.yaml \
  --episode data/episodes/episode_000001 \
  --output data/lerobot/piper_pick \
  --repo-id local/piper_pick \
  --task "pick up the object"
```

## Future work

The v0 boundary is intentionally small: deterministic Profile-driven mapping,
causal synchronization, lazy image decoding, and native LeRobot output. Future
work should extend those boundaries rather than introduce a second dataset
schema or writer:

1. Emit a per-episode synchronization report with nearest-sample offsets,
   causal action ages, initial measured-state holds, and stale tail holds.
2. Add optional Profile-driven quality gates for missing streams, maximum
   camera/state offset, stale actions, dropped messages, and usable duration.
   Start with report-only warnings before enabling conversion failures.
3. Make episode boundary handling explicit: preserve, trim, or separately tag
   the idle regions before the first command and after the final command.
4. Preserve source/header timestamps and timestamp-domain metadata where
   available. Recorder receive time remains the common fallback; it must not be
   presented as hardware exposure synchronization.
5. Validate recorder artifacts before conversion, including checksums,
   finalizer state, capture-health errors, and Profile/stream snapshots.
6. Benchmark long episodes and multi-episode conversion. Only expose native
   LeRobot video encoder concurrency or streaming settings when measured memory
   or throughput requires it.

These additions should keep `DatasetContract` as the single semantic contract,
`ProfileEpisodeReader` as the synchronization boundary, and
`LeRobotDataset` as the writer.
