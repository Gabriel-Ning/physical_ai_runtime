# Policy Inference Examples

These are ordinary Python scripts, not ROS packages. Source the workspace and
run them directly; no additional `colcon build` is needed.

## 1. VLA / ACT / diffusion / flow-matching example

[`examples/vla_policy_example.py`](examples/vla_policy_example.py) keeps the
whole path visible in one file:

```text
JointState + RGB Image + task
  -> NumPy observation dictionary
  -> optional PyTorch tensor conversion
  -> model action [T,D]
  -> JointTrajectory
  -> /action_sources/policy/joint_chunk
```

Run the safe hold example:

```bash
source install/setup.bash
python src/policy_inference/examples/vla_policy_example.py --ros-args \
  -p joint_names:="[joint1,joint2,joint3]" \
  -p image_topic:=/camera/color/image_raw
```

These policy families share the same runtime shape: state, images, and optional
language enter the model; the model returns a short receding-horizon action
chunk `[T,D]`. Users replace `ExampleChunkPolicy.predict()` and adjust the
observation keys required by their model.

## 2. Complete trajectory example

[`examples/trajectory_policy_example.py`](examples/trajectory_policy_example.py)
shows a learning-based planner that returns:

```python
positions, times = policy.predict(observation)
# positions: [T,D]
# times:     [T], strictly increasing
```

Run it directly:

```bash
python src/policy_inference/examples/trajectory_policy_example.py --ros-args \
  -p joint_names:="[joint1,joint2,joint3]"
```

The example publishes one complete trajectory and then waits.

## 3. Diffusion planner (start/goal -> full trajectory)

[`examples/diffusion_planner_example.py`](examples/diffusion_planner_example.py)
is the migration template for a diffusion planner with this contract:

```text
q_pos/vel/acc start+goal  ->  DiffusionPlanner.plan(request)
                          ->  positions/velocities/accelerations [T,D]
                              time_from_start [T]
                          ->  /action_sources/policy/joint_trajectory_goal
```

`T` is the number of points in the trajectory (chosen by the model).
Only replace `DiffusionPlanner.plan()`. Keep request/result keys; shapes must
be `[T,D]` / `[T]` with matching `joint_names`.

```bash
python src/policy_inference/examples/diffusion_planner_example.py --ros-args \
  -p joint_names:="[panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7]" \
  -p q_pos_goal:="[0.2,-0.3,0.1,-1.8,0.2,1.6,0.1]"
```

## Execution Manager inputs

VLA chunks publish to:

```text
/action_sources/policy/joint_chunk
```

Complete learned trajectories publish to:

```text
/action_sources/policy/joint_trajectory_goal
```

The complete-trajectory input is opt-in. The application-owned Execution
Manager configuration must enable it:

```yaml
execution_manager:
  ros__parameters:
    sources: >-
      {"policy":{"priority":50,
                  "inactive_timeout_s":1.0,
                  "goal_contracts":["joint_trajectory_goal"]}}
```

EM forwards accepted complete trajectories to its configured
`FollowJointTrajectory` action server. The examples default to hold or linear
interpolation; validate model-specific motion first with fake hardware.
