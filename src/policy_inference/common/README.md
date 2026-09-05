# Policy inference features

Policy inference is a pure producer in the Action Node Framework:

```text
Robot.get_observation()
  -> policy.select_action(observation)
  -> Node.submit(action | actions | None)
  -> Execution Manager arbitration
```

RMI's `profile.policy_layout(...)` fixes joint ordering, state/action feature
names, camera features and action slices before model loading. Context-created
cameras are included in Robot observations. Policies do not know leases,
controllers, preemption or recorder internals.

The application owns the RMI Node and its activation scope. Local and remote
policies expose the same `select_action(observation)` call.

This directory is reserved for fallback inference mechanics shared by policy
backends that do not provide their own action-chunk runtime. Backends such as
LeRobot keep using their native preprocessing, postprocessing and inference
engines instead.
