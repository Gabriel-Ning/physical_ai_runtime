# LeRobot policy

The LeRobot integration is an ordinary policy object. It owns model inference
and action buffering, but it does not own an RMI Context, Node, authority lease,
ROS executor, or control loop:

```text
Robot.get_observation()
  -> LeRobotPolicy.select_action()
  -> RMI Node.submit()
  -> Execution Manager
```

The RMI profile supplies ordered state/action features, camera sources and the
named `Policy` Node binding. The application attaches the profile cameras to its
Context, constructs the RMI Node and controls its authority scope.

```python
layout = context.profile.policy_layout("Policy")
for camera_name in layout.camera_sources.values():
    context.make_camera(
        camera_name,
        converter=ros_image_to_numpy,
        history_size=1,
    )

policy = LeRobotPolicy(layout, checkpoint, task=task, device=device)
policy_node = context.make_node("Policy", policy)

try:
    with policy_node.activate():
        while running:
            observation = context.robot["dual_manipulator"].get_observation()
            actions = policy.select_action(observation)
            policy_node["dual_manipulator"].submit(actions)
finally:
    policy.close()
```

Returning `None` during model warm-up is a valid no-candidate cycle. A remote
policy can implement the same `select_action(observation)` boundary without
changing this application loop.
