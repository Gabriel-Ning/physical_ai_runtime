# Execution Manager (`execution_manager`)

`execution_manager` is the authoritative C++ execution selection, command routing, and JTC action proxy daemon in the Physical AI Runtime.

It runs persistently on the Workstation, maintaining single-authority control over downstream `ros2_control` hardware endpoints.

---

## 1. Architecture & Responsibilities

```text
Policy / Teleop / Planner application
        -> ordinary ROS topic or FollowJointTrajectory Action
        -> profile-declared trusted source ingress
        -> C++ EM authority gate + controller switch
        -> /execution/<resource>/...
        -> ros2_control controller + RT guard
        -> hardware
```

- **Atomic Multi-resource Claim**: Atomically claim control over multiple resources (e.g. `left_arm`, `right_arm`) with explicit preemption rules (`preempt=true`).
- **Partial Preemption Fencing**: If a multi-resource lease is partially preempted by another client, the entire original lease is invalidated and remaining resources transition to `UNOWNED` to prevent uncoordinated motions.
- **Application-transparent authority**: Policy, teleop, and planner nodes do not
  acquire leases. EM maps their profile-declared input and activation signals to
  internally fenced commands, performs preemption, and restores the default
  policy after a transient planner/teleop source releases control.
- **Typed internal fencing**: Internal streaming envelopes and JTC goals carry
  lease identity and timestamps. These contracts do not leak into application
  code.
- **Action proxy responses**: JTC and parallel-gripper feedback, terminal result,
  cancellation, and error details are relayed through EM to the originating
  Action client. Recorder-visible Action lifecycle traces are future work.
- **Stamped streaming admission**: External `joint_reference`, `pose_reference`,
  and `twist_reference` messages require a non-zero `header.stamp` in the EM ROS
  clock domain. Zero-stamped streaming commands are rejected before source
  activation or downstream forwarding. This does not change the standard JTC
  meaning of a zero `trajectory.header.stamp` inside an Action goal: execute now.
- **ROS 2 Interfaces**: Wire contracts are defined in `execution_manager_interfaces`.

## Clock behavior

`use_sim_time` is false by default and is fixed at process startup. On real
hardware, source liveness, service/action waits, and trajectory-guard heartbeat
pacing use steady/wall time. Message age is compared in the node ROS clock
domain. In simulation, message age, status publication, and trajectory-guard
heartbeat cadence follow `/clock`, while wall polling remains available for
cancellation and shutdown responsiveness. Restart the Execution Manager to
change clock domains.

---

## 2. Usage

### Launching the Daemon

```bash
ros2 launch execution_manager execution_manager.launch.py \
  profile:=/path/to/execution_manager.yaml
```

### Prefix.dev Binary Installation

```toml
[dependencies]
ros-jazzy-execution-manager = ">=0.2.0"
```

---

## 3. Building from Source

```bash
colcon build --packages-select execution_manager
```

---

## 4. Future Work

### Recorder-visible Action lifecycle traces

Live Action behavior is complete: EM returns goal acceptance, periodic feedback,
terminal `SUCCEEDED`, `CANCELED`, or `ABORTED` status, controller result fields,
and cancellation to the originating client. The current recorder can still
capture long executions through configured command/state streams, but that is not
an exact log of the ROS Action transaction.

A future release will publish the already-defined
`TrajectoryExecutionEvent`, `TrajectoryExecutionFeedback`,
`GripperExecutionEvent`, and `GripperExecutionFeedback` trace messages. The trace
will contain an accepted event with the full goal, periodic feedback, and one
terminal event with the controller result so an Action execution can be audited
or replayed with its original lifecycle semantics.

---

## License

Apache-2.0 License.
