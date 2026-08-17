# Joint Trajectory Controller Guard

Small RT-host watchdog for a
`control_msgs/action/FollowJointTrajectory` server. It does not proxy goals and
does not run inside the controller-manager update loop.

The workstation publishes `std_msgs/Bool` on the configured heartbeat
topic while its JTC goal is owned by a live session:

- `true` arms or refreshes the guard;
- missing `true` messages for `heartbeat_timeout_s` requests
  `async_cancel_all_goals()` from the configured local JTC action server;
- `false` disarms the guard after normal completion/cancel;
- after a timeout fault, the fault stays latched and a new `true` is ignored
  until an explicit `false` starts a new session.

The guard publishes a standard diagnostic entry on `/diagnostics`, including
`guard_state`, `fault_code`, `fault_sequence`, the action name, and heartbeat
topic. It distinguishes heartbeat timeout, accepted cancel, no active goal,
unavailable action server, rejected cancel, and cancel-response timeout.

The guard only requests cancel. Smooth stopping is owned by JTC and must be
enabled in the robot profile:

```yaml
arm_jtc:
  ros__parameters:
    state_interfaces: [position, velocity]
    constraints:
      decelerate_on_cancel: true
      arm_joint1: {max_deceleration_on_cancel: 3.0}
```

Every controlled joint needs a positive `max_deceleration_on_cancel`, and all
joints need a velocity state interface. Values are robot-specific safety
parameters and require attended hardware validation.

Example:

```bash
ros2 launch joint_trajectory_controller_guard jtc_guard.launch.py \
  action_name:=/arm_jtc/follow_joint_trajectory
```

The default private heartbeat topic is `/jtc_guard/heartbeat`. Robot bringup
must override `heartbeat_topic` to the execution-namespace endpoint used by
the RMI profile, for example `/execution/arm/trajectory_guard_heartbeat`.

```yaml
joint_trajectory:
  ros_actions:
    follow_joint_trajectory: /execution/arm/follow_joint_trajectory
  ros_topics:
    trajectory_guard_heartbeat: /execution/arm/trajectory_guard_heartbeat
```

RMI pre-arms the 10 Hz heartbeat immediately before dispatching the downstream
goal, closing the race between RT goal arrival and the acceptance response. A
confirmed rejection, terminal result, or cancel response sends `false`;
ambiguous dispatch, communication, or cancel-path failure stops heartbeat
without sending `false`, deliberately driving the guard through its timeout
path.
