# Usage

## Build and test

Inside Physical AI Runtime (recommended):

```bash
pixi install --locked
eval "$(pixi shell-hook --frozen)"   # or direnv
colcon build --symlink-install --packages-select isaacteleop_toolbox \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select isaacteleop_toolbox --event-handlers console_direct+
```

## CloudXR assets (once per machine / workspace)

```bash
ros2 run isaacteleop_toolbox isaacteleop-cloudxr-setup
```

Requires `$CLOUDXR_DIR` (set by Physical AI Runtime activation) or
`--cloudxr-dir /path/to/ws/.cloudxr`. See [CloudXR](CLOUDXR.md).

## Source-only live (no robot)

```bash
ros2 launch isaacteleop_toolbox bimanual_target_live.launch.py
```

RViz starts by default after a short delay (`use_rviz:=false` to disable).

On the Quest browser open `https://<pc-lan-ip>:48322/client/` (see CloudXR doc).
Without a headset the node stays up and retries XR connection.

## MCAP replay

```bash
ros2 launch isaacteleop_toolbox bimanual_target_live.launch.py \
  mcap_replay_path:=/absolute/path/to/controllers.mcap
```

## Published topics (default profile)

Node name: `quest3_bimanual_target`. Profile:
`configs/quest3_bimanual_relative.yaml`.

| Topic | Type | Role |
|---|---|---|
| `~/left_pose_target` | `geometry_msgs/PoseStamped` | Left EE pose target |
| `~/right_pose_target` | `geometry_msgs/PoseStamped` | Right EE pose target |
| `~/left_gripper_target` | `trajectory_msgs/JointTrajectory` | Left continuous gripper reference |
| `~/right_gripper_target` | `trajectory_msgs/JointTrajectory` | Right continuous gripper reference |
| `~/status` | `std_msgs/String` (JSON) | Versioned pipeline status |
| `/teleop/quest3_bimanual/*/snapshot/*` | `PoseStamped` | Latched clutch snapshots |

Robot apps should remap the private targets into their execution stack.
Snapshot names are absolute for easy recording.

## Validation order

1. Unit tests  
2. CloudXR setup  
3. Source-only live or MCAP replay  
4. Fake hardware in the **robot app**  
5. Conservative real motion in the **robot app**

Steps 4–5 are intentionally outside this package.

## Package layout

```text
configs/                 parameter profiles
launch/                  source-only launches
rviz/                    visualization configs
python/isaacteleop_toolbox/
  cloudxr_*.py           host-client assets + setup CLI
  runtime.py             live/replay TeleopSession loop
  session_builders.py    IsaacTeleop graph construction
  retargeters/           application retargeters
  nodes/                 ROS nodes
test/                    unit tests
docs/                    detailed documentation
```
