# Marvin Workstation

默认机位：**Marvin 双臂 + 双 Pika**。每只 Pika 带 **鱼眼 + D405**（RT / gamma）。工作站 AA 上是 **两路 D435**（head + 第三人称），以及常驻的 Execution Manager 和 episode recorder。

Package path: `src/bringup/marvin_manipulation/workstation_launch`  
ROS package name: `marvin_manipulation_workstation_launch`

## 配置

- `config/execution_manager.yaml` — 唯一路由表，也是 EM launch 的 `config`
- `apps/recording/marvin_manipulation.yaml` — 启用相机时的 recorder 契约，相机流参与 `start_gate`
- `apps/recording/marvin_manipulation_no_cam.yaml` — 禁用相机时的 recorder 契约，保留相机流定义但不要求相机在线。RGB-D depth 均录 `aligned_depth_to_color/image_raw`。`root_dir` / `experiment_name` / `task` 在 RMI profile 里改
- `config/camera/workstation_realsense.yaml` — head D435I（`243222071293`）+ 侧面第三人称 D435I（`405622076349`）
- `apps/profiles/marvin_bimanual.yaml` — RMI 应用 API。通过 `execution_manager_config` 读上面那份路由表，自己不写 `groups` / `provider_selection`

完整 stack 固定启动 EM + recorder。`with_cameras` 控制工作站 RealSense，
`with_teleop` 控制 Quest 3 teleop；两者分别默认为 `true` 和 `false`。

带相机的录制契约要求全部配置相机就绪；no-camera 契约中的相机为 `required: false`、`start_gate: false`。夹爪命令在两份契约里都不是启动条件。RT 用 `load_pika_hardware:=true|false` 统一加载或关闭双侧 Pika。

## 启动

RT 先起来之后，workstation 三件套一起起：

```bash
ros2 launch marvin_manipulation_workstation_launch workstation_stack.launch.py

# 不启动工作站相机，启用 Quest 3 teleop
ros2 launch marvin_manipulation_workstation_launch workstation_stack.launch.py \
  with_cameras:=false with_teleop:=true
```

只起 Execution Manager：

```bash
ros2 launch marvin_manipulation_workstation_launch execution_manager.launch.py
```

两路 D435 一起起；第三人称默认晚 2 s（`second_camera_delay`）：

```bash
ros2 launch marvin_manipulation_workstation_launch realsense_camera.launch.py
```

只起 recorder：

```bash
ros2 launch marvin_manipulation_workstation_launch recorder.launch.py
```

## 验收

```bash
ros2 node list | grep -E 'execution_manager|episode_recorder|head_d435|third_person_d435'
ros2 topic echo /execution_manager/authority_status --once
ros2 topic hz /third_person_d435/camera/color/image_raw --window 20
ros2 topic hz /joint_states --window 20
```

| Sensor | Topic | Host |
|--------|-------|------|
| Head D435 | `/head_d435/camera/color/image_raw` | workstation |
| Third-person D435 | `/third_person_d435/camera/color/image_raw` | workstation |
| Left D405 | `/left_pika_d405/camera/color/image_raw` | RT |
| Left fisheye | `/left_pika_fisheye/image/compressed` | RT |
| Right D405 | `/right_pika_d405/camera/color/image_raw` | RT |
| Right fisheye | `/right_pika_fisheye/image/compressed` | RT |

`/joint_states` 和 Execution Manager authority status 始终是 `start_gate`；带相机契约还要求全部相机流。夹爪命令是 `std_msgs/Float64MultiArray`。
