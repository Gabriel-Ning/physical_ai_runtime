# franka_manipulation_workstation_launch

Workstation bringup for Franka FR3: Execution Manager and episode recorder.

Package path: `src/bringup/franka_manipulation/workstation_launch`  
ROS package name: `franka_manipulation_workstation_launch`

RT Host stack: sibling package `franka_manipulation_rt_launch`.

## Config

- `config/recording/rmi_fr3_policy.yaml` — MCAP stream contract

Defaults for launch args come from `apps/profiles/fr3_pika_single_arm.yaml`.

## Launch

```bash
ros2 launch franka_manipulation_workstation_launch franka_workstation.launch.py
```

EM only:

```bash
ros2 launch franka_manipulation_workstation_launch franka_workstation.launch.py \
  with_recorder:=false
```
