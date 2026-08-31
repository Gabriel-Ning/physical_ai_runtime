# piper_manipulation_workstation_launch

Workstation bringup for Piper: Execution Manager, episode recorder, Orbbec Femto Bolt,
dual wrist RealSense D435i, and optional leader teleop arms.

Package path: `src/bringup/piper_manipulation/workstation_launch`  
ROS package name: `piper_manipulation_workstation_launch`

RT Host stack: sibling package `piper_manipulation_rt_launch`.

## Config

- `apps/recording/piper_bimanual.yaml` — application-selected MCAP stream contract
- `config/execution_manager.yaml` — controller routing and source admission
- `config/camera/femto_bolt.yaml` — static Orbbec cell camera
- `config/camera/d435i_dual.yaml` — left/right wrist RealSense streams
- `config/teleop/piper_leaders.yaml` — leader CAN defaults

Defaults for launch args come from `apps/profiles/piper_bimanual.yaml`.

## Launch

Full workstation stack:

```bash
ros2 launch piper_manipulation_workstation_launch piper_workstation.launch.py
```

Fake hardware、无相机验证（两个终端；默认加载双臂和双夹爪）：

```bash
# Terminal 1: RT host
source install/setup.bash
ros2 launch piper_manipulation_rt_launch rt_stack.launch.py \
  use_fake_hardware:=true use_rviz:=true load_gripper_hardware:=true \
  cpu_affinity:=none
```

```bash
# Terminal 2: workstation（无 Orbbec、无 RealSense、无真实 leader）
source install/setup.bash
ros2 launch piper_manipulation_workstation_launch piper_workstation.launch.py \
  with_orbbec:=false with_realsense:=false with_leaders:=false
```

随后运行 leader relay API 示例时，需要另外启动真实 leader；纯 fake hardware
检查可先验证 homing profile、EM、recorder 和控制器：

```bash
ros2 control list_controllers
ros2 action list | grep -E 'left_gripper|right_gripper'
ros2 topic echo /joint_states --once
```

当前 fake hardware 启动姿态已经写入 `apps/profiles/piper_bimanual.yaml`
的 `homing.joint_positions`：双臂 12 个关节和双夹爪均为 `0.0`。
`recorder.rate_hz` 为 `50.0`，`max_duration_s` 为 `60.0`；相机流在录制
contract 中为 optional，因此上述无相机启动不会阻塞录制 start gate。

Peripheral entrypoints:

```bash
ros2 launch piper_manipulation_workstation_launch piper_orbbec.launch.py
ros2 launch piper_manipulation_workstation_launch piper_realsense.launch.py
ros2 launch piper_manipulation_workstation_launch piper_leaders.launch.py
```

## Camera validation (key test)

After RT + workstation are up, verify observation topics match `apps/profiles/piper_bimanual.yaml`:

| Sensor | Topic | Check |
|--------|-------|-------|
| Static Orbbec | `/observation/static_orbbec/color/image_raw` | `ros2 topic hz` ~30 Hz, stable frame_id |
| Left wrist RS | `/observation/left_hand_realsense/color/image_raw` | non-zero rate, correct serial |
| Right wrist RS | `/observation/right_hand_realsense/color/image_raw` | non-zero rate, correct serial |

Quick smoke:

```bash
ros2 topic list | grep observation
ros2 topic hz /observation/static_orbbec/color/image_raw
ros2 topic hz /observation/left_hand_realsense/color/image_raw
ros2 topic hz /observation/right_hand_realsense/color/image_raw
```

Record with cameras enabled:

```bash
ros2 launch piper_manipulation_workstation_launch piper_workstation.launch.py \
  with_orbbec:=true with_realsense:=true with_recorder:=true
```

Then inspect `episode_health.json` for `static_orbbec`, `left_wrist_cam`, and `right_wrist_cam`
stream counts and `recorder_drops == 0`.
