# piper_leader_teleop

Homomorphic Piper leader source for ROS 2 Jazzy. The first version provides:

- local libpiper 500 Hz MIT gravity compensation;
- teaching-pendant finger free move;
- fresh, coherent arm references on
  `/action_sources/piper_leader/arm/joint_reference`
  (`trajectory_msgs/JointTrajectory`);
- raw pendant state (`sensor_msgs/JointState`) and an EM-contract gripper
  reference on `/action_sources/piper_leader/end_effector/joint_reference`
  (`trajectory_msgs/JointTrajectory`, single named position-only point;
  joint position is one-finger travel = pendant opening width / 2);
- JSON source diagnostics on `/teleop/piper_leader/status`.

Haptic feedback is intentionally deferred. The follower remains independently
owned by `ros2_control`; this node only publishes leader source topics.

## RMI dual-arm (piper_bimanual)

This node is a workstation source. It does **not** talk to
`controller_manager`. Under LocalEM the follower JSPC listens on
`/execution/<side>_arm/joint_reference`, so a small RMI process must admit
`TeleopJoint_{Left,Right}` and relay `action_sources` onto that ingress.

Build on the workstation (this package is not in the RT colcon set until
`piper_leader_teleop` is selected):

```bash
colcon build --symlink-install --packages-select piper_leader_teleop
source install/setup.bash
```

RT follower stack must already be up. Then, in order:

```bash
# 1. Admit TeleopJoint and start the relay (keeps JSPC + gripper_fwd active)
python examples/14_piper_leader_teleop.py --profile piper_bimanual.yaml --side both

# 2. Other terminals: start leaders disabled. Do not enable yet.
#    Override can_interface in the yaml if the pendant is not on can0/can1.
ros2 launch piper_leader_teleop piper_leader.launch.py \
  config:=$(ros2 pkg prefix piper_leader_teleop)/share/piper_leader_teleop/config/piper_leader_left.yaml \
  node_name:=piper_leader_left autostart:=false

ros2 launch piper_leader_teleop piper_leader.launch.py \
  config:=$(ros2 pkg prefix piper_leader_teleop)/share/piper_leader_teleop/config/piper_leader_right.yaml \
  node_name:=piper_leader_right autostart:=false

# 3. Gate: status JSON has active=false and no joint_reference traffic.
ros2 topic echo /teleop/piper_leader_left/status --once

# 4. Enable only after the leader is supported and CAN is live.
ros2 service call /piper_leader_left/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /piper_leader_right/enable std_srvs/srv/SetBool '{data: true}'
```

Stop before killing the nodes:

```bash
ros2 service call /piper_leader_left/enable std_srvs/srv/SetBool '{data: false}'
ros2 service call /piper_leader_right/enable std_srvs/srv/SetBool '{data: false}'
```

Then Ctrl+C the example so TeleopJoint releases. Do not run Policy / marker
teleop against the same arm while a leader is enabled.

Left yaml defaults `can0` and publishes follower joint names `left_joint*`;
right yaml defaults `can1` and `right_joint*`. Follower SocketCAN on the RT
host (`piper0` / `piper1`) is a different pair of adapters.

## Leader model

The launch file expands
`piper_description/urdf/piper_with_teach.urdf.xacro` and passes it through the
private `leader_robot_description` parameter. This is the leader gravity model
only:

- it includes the teaching-pendant inertial model;
- it contains no follower `ros2_control` hardware block;
- it is not the follower controller manager's `robot_description`;
- it is not published as the global `/robot_description` topic.

The model always includes the identity `flange_link -> flange` alias required
by `libpiper::Model`. It is canonical and unprefixed because it is private to
the teleop node rather than part of the global ROS robot description.

libpiper currently loads models from a file path, so the node materializes this
expanded parameter into a private temporary URDF while the leader is active.

## Safety and lifecycle

The packaged config sets `autostart: false`. In that disabled state the ROS
node and follower-state subscription are live, but CAN hardware and control
loops stay inactive until the explicit service call.

```bash
ros2 launch piper_leader_teleop piper_leader.launch.py
ros2 service call /piper_leader/enable std_srvs/srv/SetBool '{data: true}'
```

To initialize immediately in non-preempting shadow mode instead:

```bash
ros2 launch piper_leader_teleop piper_leader.launch.py \
  autostart:=true default_mode:=shadow
```

`autostart` is an optional launch override: an empty value uses the YAML/node
default, `false` starts disabled, and `true` enters `default_mode`. Shadow and
passive modes never publish action references; only `active_preempt` does.

Stop before shutting down:

```bash
ros2 service call /piper_leader/enable std_srvs/srv/SetBool '{data: false}'
```

`restore_pendant_servo_on_stop` defaults to false: stopping leaves the pendant
finger driver disabled rather than issuing an enable command with an implicit
width target.

## Fake follower validation

The left-side validation configs form this exact path:

`can0 leader -> /action_sources/piper_leader_left/arm/joint_reference (+ end_effector/joint_reference) -> em_left -> left_arm_jspc + left_gripper_fwd -> fake left follower`

Use `config/piper_leader_left.yaml` and `config/piper_leader_right.yaml` for
the two leaders and
`piper_manipulation_controller_bringup/config/leader_fake_left.yaml` for the
follower. The leader config publishes the follower's prefixed joint names; the
leader gravity model itself remains the independent, unprefixed
`piper_with_teach` model.

Left and right leader nodes may load the same canonical model independently.
Their CAN interfaces, node namespaces, output topics, and published follower
joint names provide the ROS-side identity; libpiper model names stay unprefixed.

## Validation gates

1. Launch only. Expect a status topic with `active=false`; no action target is
   published and no hardware command is sent.
2. With the leader supported and CAN configured, enable it. Expect coherent
   arm/pendant sequences in the status JSON and moving joint references.
3. Only after gate 2 passes, configure the execution manager to consume the
   `piper_leader` source and test the follower at conservative limits.
