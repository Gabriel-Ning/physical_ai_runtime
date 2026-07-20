# Recording Architecture

## Product boundary

The runtime records independently timed ROS streams and the evidence needed to
interpret them later. It does not assemble online observation/action rows,
interpolate state, resize images, or emit a training schema.

```text
ROS/device publishers
  -> episode_recorder capture and MCAP writer
  -> episode finalizer and validator
  -> immutable raw experience bundle
  -> temporal dataset compiler (future)
  -> LeRobot / RLDS / Open X-Embodiment / custom datasets
```

This follows the useful separation in Project Aria: raw streams, explicit time
domains, stream configuration and calibration, query/alignment, and derived
products are distinct layers. MCAP remains the raw container; VRS is not
reimplemented.

The source of truth is the complete finalized episode bundle, not MCAP alone.
MCAP is authoritative for serialized payloads and their record timeline;
manifests, calibration, clock observations, health evidence, and checksums are
equally required to interpret that payload.

## Product invariants

- Serialized ROS payloads are written unchanged.
- The recorder is loss-accounted, not magically lossless: upstream sequence
  gaps, DDS loss, recorder-side drops, and storage failures remain observable.
- Streams remain independent; differing rates, latency, and gaps are expected.
- The MCAP timestamp is sampled when the recorder callback receives a message,
  not later when the writer thread drains its queue.
- `header.stamp` is not assumed to be capture time. Its meaning comes from the
  stream contract.
- Missing required streams, incompatible types or QoS, recorder-side drops,
  and storage failures cannot silently produce a finalized episode.
- Capture memory is bounded and queue pressure is observable.
- Only a successfully verified directory is named as a finalized episode.
- Automatic finalization is the only operation that mutates an episode bundle.
- A finalized episode is immutable. Standalone validation is read-only and may
  write a report only outside the episode directory.

## Stream contract

Each stream has a stable semantic ID in addition to its ROS topic:

```yaml
streams:
  - id: overhead_left_image
    label: camera.overhead_left.image
    role: camera_image
    topic: /cameras/overhead_left/image_raw
    expected_type: sensor_msgs/msg/Image
    required: true
    qos:
      mode: auto
      depth: 10
    time:
      source_domain: header_declared_host_receive
    health:
      expected_rate_hz: 30.0
      min_rate_ratio: 0.98
      max_gap_s: 0.08
      recorder_drop_policy: fail
```

Supported roles are descriptive, not a closed type hierarchy: camera image,
camera info, frame metadata, robot state, raw teleoperation, normalized intent,
execution command/status, hardware feedback, IMU, force/torque, and diagnostic
streams all use the same raw recording path.

Legacy `topics` configuration remains accepted. It is converted to required
streams whose IDs and labels equal the topic and whose source time is
`record_only`.

## Time model

| Name | Meaning | Recording behavior |
| --- | --- | --- |
| Record time | Recorder host system time at subscription receipt | Stored as MCAP timestamp |
| Receive steady time | Recorder `steady_clock` receipt time | Used for online age/gap statistics |
| Header time | Timestamp embedded by a ROS publisher | Interpreted only when declared |
| Device time | Native sensor/controller clock | Preserved in the original message or metadata stream |
| Common time | Validated PTP/TAI/UTC or another shared clock | Claimed only with clock identity, domain, mapping validity, and lock evidence |

The recorder never invents a conversion between these domains. Future provider
queries use explicit domains and `Before`, `After`, `Closest`, and `Bracket`
semantics. Alignment policy belongs to offline conversion.

Three different notions of time must remain separate:

1. **Raw time** is evidence retained by the recorder: record, receive, header,
   device, and validated common time.
2. **Alignment time** is a compiler-selected timeline. An alignment recipe may
   choose a camera exposure as an anchor, interpolate robot state, bracket IMU
   samples, and select the last command before or first command after an anchor.
3. **Dataset time** is the timestamp or sample index emitted by an exporter.
   It is a derived representation and is never written back into the raw
   episode.

Clock support is split across two boundaries. The recorder captures clock
observations: identity, domain, synchronization mode, lock state, offset,
uncertainty, and validity interval when available. A future `ClockMapper`
performs explicit domain conversion and returns mapped time together with
uncertainty, validity, mapping revision, and provenance. Linux realtime, PTP,
TAI, gPTP, and OpenHarmony distributed clocks fit this model without making
the recorder claim an unverified common timeline.

## Capture data plane

The lifecycle node owns subscriptions and episode control. High-rate callbacks
sample receipt clocks, assign a monotonic receive sequence, update low-cost
statistics, and enqueue a serialized message into a bounded MPSC queue.

A single worker owns `rosbag2_cpp::Writer`. It drains accepted messages in
receive order and writes the callback receipt timestamp. Queue capacity,
high-water mark, enqueue failures, and writer errors are part of episode
health. Required-stream queue loss makes the episode fail.

The recorder is non-real-time. It performs no file I/O in robot controller
threads and runs below hardware/control priorities.

## Session and episode metadata

A session manifest contains relatively static context:

- session/experiment/operator identity;
- robot and device identity;
- recording profile and source configuration hashes;
- host, boot ID, ROS domain, RMW, clock source, and synchronization snapshot;
- software and Git revisions.

An episode manifest contains:

- schema version, session reference, episode ID/index, task and state;
- start/end time, storage profile and recorder version;
- requested and resolved stream contracts, types, QoS, and publishers;
- configuration, runtime parameter, calibration, `CameraInfo`, and `/tf_static`
  snapshot references;
- online counters, finalizer result, and file inventory.

Application-specific teleop, retargeter, controller, and calibration metadata is
passed as structured manifest context. The recorder preserves it without
interpreting policy semantics.

## Transactional lifecycle

```text
IDLE -> ARMING -> RECORDING -> FINALIZING -> FINALIZED
                  |              |
                  +-> DISCARDING  +-> INCOMPLETE
                  +-> ERROR --------> INCOMPLETE
```

Recording starts in `episode_NNNNNN.partial`. Start performs stream, path,
writer, and disk-space preflight. Stop disables capture, drains the queue,
closes the writer, and invokes finalization. Finalization reopens MCAP, verifies
its inventory, writes health and SHA-256 checksums, updates the manifest last,
then atomically renames the directory on the same filesystem.

After the rename, neither the validator nor a future compiler may modify the
episode. Validation prints a report or writes it to an explicitly external
path. Derived indexes, aligned samples, videos, and datasets live in separate
artifact directories and refer back to the source bundle by content hash.

An interrupted `.partial` directory is never treated as finalized. Startup
marks it incomplete and preserves it for explicit validation, recovery, or
deletion. Discard removes bulk data and writes an audit entry at session level.

## Health model

Online health is intentionally cheap:

- count, first/last receipt, rate and maximum receipt gap;
- queue depth/high-water mark and recorder-side drops;
- topic type, publisher and QoS changes;
- writer failures, bytes observed, disk free space and write throughput;
- missing or stale required streams.

The finalizer performs expensive checks after close:

- MCAP readability and topic/type/count inventory;
- declared source timestamp monotonicity, duplicates, jumps and skew;
- profile-specific metadata checks;
- numeric and boolean PASS/WARN/FAIL rules;
- duration-dependent threshold overrides;
- checksums covering bag, rosbag metadata, snapshots, health and manifest.

The standalone validator and automatic finalizer use the same implementation.
They use different mutation policies: automatic finalization may write only
inside a `.partial` directory, while standalone validation of a finalized
episode is strictly read-only.

## Hikrobot coverage floor

The generic product must retain all information recorded by the Hik reference
tool: image, `CameraInfo`, `FrameMetadata`, statistics/burst streams, camera
configuration, runtime parameters, types, Git revision, storage settings and
checksums.

When the Hik validation profile is enabled, offline validation additionally
checks stable identity, image/info/metadata association, capture and device
timestamp monotonicity, the closed common trigger interval, continuous trigger
indices, incomplete burst rate, device-time spread and effective rate.

These checks are a camera profile, not the core data model. The core has no hard
dependency on a Hik-specific image wrapper and does not use callback arrival
order for physical synchronization.

### Verified seven-camera throughput gate

On 2026-07-18, a clean isolated build recorded seven Hikrobot GigE cameras at
`1080x720 bayer_rggb8 @ 30 Hz` for 10 seconds. Every camera produced 300 image,
CameraInfo, and FrameMetadata records; all 300 Scheduled Action bursts were
complete; recorder drops and writer failures were zero. Maximum device
timestamp spread was 13.233 us, the queue high-water mark was 17 messages /
3.89 MB, offline validation passed, and all SHA-256 entries verified.

The resulting episode was approximately 1.6 GB, so this establishes a short
throughput gate rather than long-duration production qualification. The test
also reported `device_ros_time_invalid_frames` for every frame: PTP lock and
native device timestamps were preserved, but no validated device-to-ROS/common
time mapping was available.

## Future temporal dataset compiler

The offline compiler is not part of the recorder process:

```text
immutable episode bundle
  -> EpisodeProvider / Stream Query Engine
  -> ClockMapper
  -> Alignment Engine + versioned recipe
  -> modality materializers
  -> DatasetExporter
```

The initial temporal query contract should remain value-agnostic:

```text
list_streams()
get_stream_info(stream)
get_timestamps(stream, time_domain)
get_by_index(stream, index)
query(stream, time, time_domain, Before | After | Closest, max_delta)
bracket(stream, time, time_domain, max_delta)
```

Generic query does not interpolate values. Interpolation requires message-type
and policy semantics and therefore belongs to the alignment engine. Every
alignment result records selected source indices/timestamps, deltas,
interpolation method, validity, and rejection reason.

### Indexing

MCAP chunk/message indexes are used first. Some high-throughput profiles, such
as `fastwrite`, may omit message indexes; in that case the provider may build a
derived SQLite/DuckDB or equivalent timestamp index. An external index is a
disposable cache: it is stored outside the episode, keyed by source hashes, and
can always be rebuilt. Large timestamp arrays are not duplicated into a JSON
source-of-truth file.

### Materialization and export

Image/video conversion is offline. The recorder stores exactly the ROS image,
compressed image, or hardware-encoded stream supplied by the publisher and
does not transcode it. A materializer may later select frames, resize, and
encode H.264/AV1 with explicit FPS, resolution, keyframe, and codec settings.

Exporters adapt aligned materializations to LeRobot, RLDS, Open
X-Embodiment, HDF5, or a custom VLA schema. They do not read recorder internals
directly; they consume the provider and alignment result. Every derived
artifact records:

- source episode ID, manifest checksum, and MCAP SHA-256;
- compiler/exporter version and Git revision;
- alignment recipe and recipe hash;
- calibration and clock-mapping revisions;
- stream mapping and per-sample alignment evidence;
- video/image transformation parameters.

The same immutable episode can therefore produce multiple datasets without
changing or recapturing the physical experience.

## Deferred layers

- Implementation of the read-only provider and derived index cache.
- Clock mapping, observation/action alignment, and interpolation recipes.
- Video materializers and LeRobot/RLDS/OpenX/custom exporters.
- Online multimodal frame assembly.
- Online calibration drift models.
- Distributed multi-host transaction coordination.

