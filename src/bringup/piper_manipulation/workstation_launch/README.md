# piper_manipulation_workstation_launch

Workstation bringup for Piper: Execution Manager, episode recorder, Orbbec Femto Bolt,
dual wrist RealSense D435i, and optional leader teleop arms.

Package path: `src/bringup/piper_manipulation/workstation_launch`  
ROS package name: `piper_manipulation_workstation_launch`

RT Host stack: sibling package `piper_manipulation_rt_launch`.

## Config

- `config/recording/rmi_piper_bimanual.yaml` — MCAP stream contract
- `config/camera/femto_bolt.yaml` — static Orbbec cell camera
- `config/camera/d435i_dual.yaml` — left/right wrist RealSense streams
- `config/teleop/piper_leaders.yaml` — leader CAN defaults

Defaults for launch args come from `apps/profiles/piper_bimanual.yaml`.

## Launch

Full workstation stack:

```bash
ros2 launch piper_manipulation_workstation_launch piper_workstation.launch.py
```

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
