# Co-location validation — RT host (8× Hik + Marvin + episode_recorder)

Host: `beta` / Ubuntu 24.04 / `6.8.1-1056-realtime` / i9-11900KB / 30 GiB / NVMe.

Target topology: all three stacks on this PC. Camera payload stays raw
`bayer_rggb8` (offline demosaic later). Profiles already accepted upstream and
re-verified here: `1080×720@30` and `640×480@60`.

**Status (2026-07-21):** Gate0–5 **stage validation passed** on this host
(see [Verdict](#verdict--this-rt-host-2026-07-21)). **Deferred burn-in:**
[multi-hour soak](#deferred-multi-hour-soak-required-before-calling-this-fully-burn-in).
Improvement and time-align notes: [backlog](#improvement-backlog),
[time sync](#time-sync--embodiment-dataset-alignment).

## NIC roles

| NIC | Speed / MTU | Address | Role |
| --- | --- | --- | --- |
| `enp1s0` | 10 GbE / 9000 | `192.168.1.100/24` | Hik GigE Vision / MVS |
| `enp89s0` | 2.5 GbE / 9000 | `192.168.10.100/24` | ROS DDS (raw Bayer + control) |
| `enx6c1ff764508f` | 1 GbE / 1500 | `10.19.0.100/24` | Marvin SDK (`10.19.0.191` reachable) |

CycloneDDS binds `enp89s0` via [`.config/cyclonedds_default.xml`](../.config/cyclonedds_default.xml)
(10 MiB socket buffers). Host `net.core.rmem_max` / `wmem_max` set to 16 MiB
for this session (`/etc/sysctl.d/99-ros-dds-buffers.conf` recommended for
persist).

RT CPU isolation used `RT_ISOL_CPUS=14,15`. It was applied for Gate4+
(`isolcpus=14,15` after reboot), with controller-manager affinity pinned to
CPUs 14 and 15. The host-specific profile and GRUB helper used during the test
were not retained in this repository; the evidence below stands alone.

## Repository scripts used

| Path | Role |
| --- | --- |
| `scripts/setup.sh` / `scripts/stop_ros.sh` | Workspace bringup / teardown helpers |

One-off gate runners, the RT CPU profile, the GRUB helper, and the local MCAP
conversion helper were intentionally not retained.

## Gate log

### Gate0 — machine / network — pass (2026-07-21)

- Three NICs up at expected speeds; Hik and ROS MTU 9000.
- Marvin ping `10.19.0.191` OK (~1.3 ms).
- DDS URI active; socket buffer sysctl raised.
- `isolcpus` deferred until control gates.

Command (historical): NIC/IP/sysctl checks were done via a one-off gate0 script
(removed). Re-check manually with `ip`, `ping 10.19.0.191`, and `sysctl net.core.rmem_max`.

### Gate1 — build / contracts — pass (2026-07-21)

- `episode_recorder` Release build OK.
- The tests used site-specific camera-only, co-location, and full-multimodal
  stream contracts plus a 60 Hz camera profile. Those exact site files were
  intentionally not retained in this repository. Recreate them from the
  generic examples under `episode_recorder/config` and
  `hik_camera_bringup/config`, using the deployed serial numbers and topics.

### Gate2 — 8× Hik only — pass (2026-07-21)

**Profile A** `1080×720@30` (`hik_8cam.launch.py` default):

- `cam_0` ROS hz ≈ 29.9 Hz
- `/statistics`: `effective_fps≈30`, `burst_ok_rate=100%`, PTP 1 master + 7 slaves
- sync `spread_us` p95 ≈ 12.7 µs; all cameras connected, `grab_failures=0`

**Profile B** `640×480@60` (`config_file:=.../hik_8cam_60.yaml`):

- Driver stats over ≥75 s: `action=60.0`, `eff≈60`, `bursts ok=100%`
- `spread_p50≈10.8 µs`, `host_skew=0`, per-cam ≈60–61 fps

### Gate3 — Hik + episode_recorder (camera-only) — stage pass (2026-07-21)

Profile B `640×480@60`. Recorder: `hik_8cam_60_camera_streams.yaml`.

**Accepted artifact:** `data/episodes/colocation_gate3/episode_000004`

| Metric | Value |
| --- | --- |
| `result` | **PASS** (strict validator; incomplete is hard FAIL) |
| `incomplete_bursts` | **0** / 3761 |
| `total_recorder_drops` | **0** |
| duration | ~62.7 s (≥6× `action_clock_refresh_sec=10`) |
| camera `max_record_gap_s` | ≤ **0.033 s** |
| stream statuses | all **PASS** |
| `ring_ovf` during record window | **Δ=0** (startup warmup may leave a non-zero baseline) |
| MCAP size | ~9.3 GiB |

**Fixes that made complete finalize possible:**

1. Per-camera fixed **ring buffer** (`pendingRing`, capacity 16; overflow drops oldest + `ring_ovf` in stats)
2. PTP clock refresh **after** ROS publish, and **1 LatchPtp sample per main-loop iteration** (5 samples spread) so refresh no longer stalls a whole frame period

The ring is an interim mitigation, not the final lossless design. The current
grab path makes two full-frame copies before ROS message construction, and
independent per-camera rings cannot preserve burst atomicity if one camera
overflows. The production design should write once into preallocated storage
owned by a bounded pool keyed by `trigger_index`; only complete bursts should
be published. Pool exhaustion must apply trigger backpressure or explicitly
invalidate the active episode rather than silently dropping one camera frame.

**Stage decision:** Gate3 camera-only path is **pass for this phase**. Longer soak and
repeated start/stop across multiple episodes are **deferred** to joint recording
(Gate5 / full multimodal), where the same strict finalize criteria apply to each
episode.

**Deferred stress (do not skip later):**

- Long-duration co-load soak (tens of minutes) with Hik + recorder (+ Marvin)
- Intermittent `start_recording` / `stop_recording` cycles; every episode must
  finalize with `incomplete_bursts=0` and 0 recorder drops
- Confirm `ring_ovf` Δ remains 0 across the whole session (not only one episode)

**Earlier FAIL episodes** (`000000`–`000003`) remain as regression evidence before
the publish/refresh fixes. `episode_000002` was temporarily finalized under a
rejected loose rule and is **not** a quality baseline.

Recording used `ros2 launch episode_recorder …` with
`hik_8cam_60_camera_streams.yaml` (one-off gate helper scripts removed).

### Gate4 — Marvin controller — fake + real stationary pass (2026-07-21)

`isolcpus=14,15` confirmed after reboot. The controller-manager launch was
pinned with `taskset -c 14,15`.

**Fake hardware (`use_fake_hardware:=true`) — pass:**

| Check | Result |
| --- | --- |
| `/joint_states` | ~**500 Hz** |
| controllers | JSB + left/right TSKPC all **active** |
| `ros2_control_node` affinity | **14,15** |
| plugin | `mock_components/GenericSystem` |

**Real hardware (`use_fake_hardware:=false robot_ip:=10.19.0.191`) — stationary pass:**

| Check | Result |
| --- | --- |
| SDK / HW | connected; no teleop goals published |
| `/joint_states` | ~**500 Hz** |
| controllers | all **active** |
| affinity | **14,15** |
| Steady-state CM overrun (≈45–60 s after activate) | **0** |
| Mode-switch overruns | 2× ~650 ms during `perform_command_mode_switch` L/R (one-time, not steady) |

Overrun judgment: with isolcpus + taskset, **steady loop looks clean (0 overrun/min in the
observation window)**. No side-by-side non-isolated baseline was captured in this
session, so improvement vs “before” is qualitative only.

Logs: `/tmp/gate4_marvin_fake.log`, `/tmp/gate4_marvin_real.log`

#### Known non-blocker: `write() — dispatch staging error (2 ms)`

**Symptom (Gate4 real, stationary):** periodic
`[WARN] write() — dispatch staging error (2 ms accumulated)` from
`MarvinBimanualArmHardware`. Not counted as CM overrun. Gate4 window: ~25 WARN lines,
**0** `tag busy`, **0** sustained ERROR. Accumulator always resets at **2 ms** (= one
500 Hz cycle).

**Root cause:** Marvin SDK single-buffer send tag race, not camera/DDS/CM.

1. `write()` → `marvin_sdk_bridge::dispatch_position_commands()` →
   `OnClearSet()` → `OnSetJointCmdPos_*` → `OnSetSend()`.
2. `OnSetSend()` sets an internal busy tag (`0x64`); the SDK ~1 kHz UDP timer thread
   clears it after send.
3. While the tag is still set, **both** `OnClearSet()` and `OnSetSend()` return false.
4. Because dispatch always calls `OnClearSet()` first, the busy case surfaces as
   `DispatchResult::StagingError` (“staging error”), not `SendBusy` (“tag busy”) —
   same race, different API hit first.
5. HI treats this as recoverable: skip this cycle, re-baseline next; escalate to
   ERROR only if skip time accumulates to `stale_error_ms` (default 100 ms).

`RCLCPP_WARN_THROTTLE(..., 2000)` makes sparse single-cycle misses look “periodic”.

**Impact:** Stationary hold — negligible (hold last command +2 ms). Fast motion —
occasional one-cycle miss then catch-up within velocity guard. Closed-source SDK
single buffer: race cannot be fully eliminated from our side.

**Improvement plan (deferred code change; do not block Gate5):**

| Priority | Change | Why |
| --- | --- | --- |
| 1 | Demote single-cycle skips to DEBUG; WARN only when accumulated skip ≥ warn threshold (e.g. `stale_warn_ms` / 20 ms) | Stops log noise without changing control behavior |
| 2 | Map `OnClearSet` failure-while-busy → `SendBusy` (or shared “transient busy”); reserve `StagingError` for real `OnSetJointCmdPos` failures | Correct semantics / triage |
| 3 | Optional: count busy skips in a light metric; never spin-retry inside RT `write()` | Observability without lengthening CM period |

Acceptance for a later HI patch: Gate4-equivalent stationary run shows no WARN for
isolated 2 ms skips; sustained busy ≥ error threshold still ERROR; unit tests for
busy vs real staging still pass.

### Gate5 — full co-location — stage pass (2026-07-21)

Hik + recorder + Marvin co-load. Staging-error WARNs (Gate4 note) are **expected
noise** until the HI logging patch lands — fail Gate5 only on sustained dispatch
ERROR, CM overrun under load, or recorder finalize failures.

**Phasing (stationary first):**

| Phase | Contract | Why |
| --- | --- | --- |
| **5a** | Cameras + `/joint_states` (+ optional TF/stats): `hik_8cam_60_coloc_streams.yaml` | Idle EM does not publish `pose_reference`; marker teleop has **0 publishers** without RViz — full multimodal `required: true` cannot start-gate |
| **5b** | Full multimodal `hik_8cam_60_full_streams.yaml` (markers + EM pose refs) | Needs intentional teleop / marker publishers; motion risk on real HW — operator present |

#### Gate5a evidence (Hik 60 + Marvin real stationary + coloc recorder)

Co-load smoke before record: `/joint_states` ~500 Hz, `burst_ok_rate=100%`, cam ~60 Hz;
CM overrun count during Gate5 window so far: **0** (staging WARNs continue, non-blocking).

| Episode | Duration | Result | Notes |
| --- | --- | --- | --- |
| `colocation_gate5/episode_000000` | ~62.8 s | **PASS** | coloc; incomplete=0; TF non-monotonic listed in `errors` but overall PASS |
| `colocation_gate5/episode_000001` | ~22.8 s | **PASS** | start/stop |
| `colocation_gate5/episode_000002.partial` | ~22.8 s | **FAIL** | incomplete=0; **cam_7** `max_gap≈51.7 ms > 50 ms`; TF non-monotonic |

#### Gate5b evidence (full multimodal + high-level marker teleop)

High-level PC interactive marker publishers confirmed (~50 Hz L/R). Contract:
`hik_8cam_60_full_streams.yaml`. Stress was driven with repeated
`ros2 launch episode_recorder` + `start_recording`/`stop_recording` (gate helper
scripts since removed). Schedule: 5×30 s + 3×60 s + 1×120 s.

| Episode | Duration | Result | Notes |
| --- | --- | --- | --- |
| `episode_000003` | ~32.8 s | **PASS** | first full+markers smoke (pre-stress) |
| `episode_000005`–`000009` | ~33 s each | **PASS** ×5 | start/stop stress |
| `episode_000010`–`000012` | ~63 s each | **PASS** ×3 | `000010` has TF non-monotonic in `errors` (still PASS) |
| `episode_000013` | ~122.8 s | **PASS** | long soak; markers ~50 Hz, JS ~493 Hz, cams 60 Hz, incomplete=0, drops=0 |

**Stress aggregate:** **9/9 PASS**, `incomplete_bursts=0`, `drops=0` on every round.
Disk: started ~455 GiB free → ended ~390 GiB free; experiment dir ~90 GiB.
CM overrun during Gate5: **0**. Staging WARN count grew (non-blocking).

`episode_000004.partial`: aborted mid-run during script fix (no health) — ignore.

**Interpretation:** With markers live, full multimodal co-location finalize is
stable across repeated start/stop and a 2 min soak. Occasional `/tf`
non-monotonic MCAP timestamps appear as soft `errors` without flipping
overall result; cam_7 gap FAIL from Gate5a did not recur in this full stress.

Recording: `ros2 launch episode_recorder` + service start/stop (no gate helper
scripts in-tree).

## Verdict — this RT host (2026-07-21)

**Stage validation passed** for co-location on this PC for the four stacks
below. This is not a production certification: the multi-hour soak and listed
follow-ups remain open.

| Stack | Role | Gate evidence |
| --- | --- | --- |
| 1. RT `ros2_control` (Marvin) | 500 Hz CM on `isolcpus` 14,15 | Gate4–5: steady CM overrun **0** under Hik+recorder co-load |
| 2. 8× sync Hik Bayer | PTP Scheduled Action @60 | Gate2–5: `burst_ok≈100%`, `incomplete_bursts=0` |
| 3. Action / marker command | High-level PC → DDS → EM/TSKPC | Gate5b: ~50 Hz L/R recorded, strict finalize PASS |
| 4. `episode_recorder` | Service-driven MCAP episodes | Gate5b: **9/9** full contract PASS, drops=0 |

MCAP `fastwrite` (no chunks) is a **post-process / preset** issue, not a capture
integrity failure. It remains the high-throughput recording default; create an
indexed/compressed derivative offline when Foxglove-compatible output is
needed.

### Deferred: multi-hour soak (required before calling this fully burn-in)

Not yet run. Keep as an explicit gate; do not skip when claiming long-duty
production burn-in.

| Item | Target | Pass criteria |
| --- | --- | --- |
| Duration | **≥ 2–4 hours** continuous co-load (Hik 60 + Marvin real + markers + recorder) | Process stay-up; no unexplained restart |
| Recording | Repeated `start`/`stop` across many episodes **and/or** long episodes | **Every** episode: `result=PASS`, `incomplete_bursts=0`, recorder drops=0 |
| Control | CM overrun rate under load | Steady-state overrun **≈0** (mode-switch one-shots excluded) |
| Cameras | `/statistics` + `ring_ovf` | `burst_ok` stays ~100%; `ring_ovf` Δ ≈0 over the window |
| Disk | Free space floor (e.g. 80 GiB) | Stop cleanly before floor; no corrupt partials from ENOSPC |
| Suggested entry | Loop `ros2 launch episode_recorder` + start/stop under disk floor | Log summary + CM/Hik snippets under `/tmp/gate5_soak_*` |

Shortest Gate5 soak so far: **~2 min** (`episode_000013`). Multi-hour remains **open**.

## Time sync & embodiment dataset alignment

Recorder policy: **preserve raw time evidence; do not invent online clock
conversion**. Alignment is an offline recipe (see
`episode_recorder` architecture time model).

### What each stream carries

| Stream | Strong anchors | Clock domain |
| --- | --- | --- |
| 8× cameras | `trigger_index`, `device_timestamp_ns` (+ PTP flags); image header ≈ host receive→ROS | Camera PTP → mapped on **RT PC** |
| `/joint_states` | `header.stamp` (CM/JSB) | **RT PC** ROS |
| Marker `pose_target` (high-level PC) | `PoseStamped.header.stamp` = remote `now()` | **Remote PC** ROS |
| All streams in MCAP | `log_time` = host system time at subscription receipt | **Recorder host** (this RT PC) |

Manifest also stores per-stream `source_time_domain` and
`record_timestamp_semantics: host_system_time_at_subscription_receipt`.

Gate5b example (`episode_000013`) mean `record − header` skew: joint ~0.2 ms,
camera ~1.9 ms, marker/pose_reference ~**67–68 ms** (remote clock offset + DDS
latency combined — headers are **not** the same physical clock).

### Cross-PC marker ↔ local joint/camera: what we already keep for offline align

**Enough for soft dataset alignment:**

1. Common axis: MCAP **`log_time`** on this PC for every stream (including remote markers).
2. Remote publish time: marker **`header.stamp`** (unchanged).
3. Local robot/camera times: joint headers + `FrameMetadata` device/trigger/host fields.
4. Contract labels: `source_time_domain` in `episode_manifest.json`.

**Recommended offline order:** camera group by `trigger_index` /
`device_timestamp` → map to RT ROS via image headers / receive → align joints on
RT ROS or `log_time` → align markers on **`log_time`** (or header after chrony
calibration).

**Not retained today (improvement candidates):** chrony/PTP offset snapshot at
record start; remote host/clock identity in manifest; RT-side receive stamp
written into the marker message body; hardware-common trigger between marker and
exposure.

## Improvement backlog

| Priority | Item | Notes |
| --- | --- | --- |
| P0 | **Multi-hour soak** (above) | Burn-in before “full production” language |
| P0 | Keep `fastwrite` for capture; generate indexed/compressed derivatives offline | Foxglove / indexed MCAP |
| P0 | Replace per-camera rings with a preallocated, `trigger_index`-keyed burst pool | One-copy ownership, burst atomicity, explicit backpressure |
| P1 | Marvin `dispatch staging error` WARN demote / busy semantics | Gate4 note; non-blocking |
| P1 | Cross-PC clock: chrony (or PTP) high-level ↔ RT; log offset into episode manifest | Tightens marker↔joint/camera header align |
| P2 | Optional: stamp marker receive time on RT (extra field or related topic) | Stronger than `log_time` alone for debugging |
| P2 | `ring_ovf` on ROS `CameraStatistics` + session Δ in soak logs | Catch grab overrun under load |
| P2 | TF non-monotonic MCAP soft errors | Optional stream; triage if TF becomes required |
| P3 | Bringup-native `cpu_affinity`; optional soak automation script if needed again | Ops convenience |

## Foxglove / MCAP indexing

Gate5 recordings used `storage_preset_profile: fastwrite`, which writes **no
chunks**. Foxglove Studio refuses unindexed MCAPs larger than **1 GiB** with:

`Unable to open unindexed MCAP file; unindexed files are limited to 1GB`

Post-process conversion is acceptable after the raw bag has finalized and its
checksums and stream contract pass. The site-local conversion helper used for
these tests was not retained; use a separately versioned MCAP conversion tool
and keep the original capture immutable.

**Future recordings:** keep `fastwrite` as the capture default. Generate a
separate indexed/compressed MCAP after successful finalization when required
for Foxglove or archival access; do not spend compression CPU on the live
capture path by default.
