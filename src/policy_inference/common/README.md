# Policy inference contract

Policy inference is a pure producer in the Action Node Framework:

```text
Robot.get_observation()
  -> policy.select_action(observation)
  -> Node.submit(action | actions | None)
  -> Execution Manager arbitration
```

`PolicyIOContract.from_profile(..., node_name="Policy")` fixes joint ordering,
camera features and action slices before model loading. Context-created cameras
are included in Robot observations. Policies do not know ROS topics, leases,
controllers, preemption or recorder internals.
