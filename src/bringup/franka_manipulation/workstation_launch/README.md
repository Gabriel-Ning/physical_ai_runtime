# Franka Workstation

Workstation 常驻 gamepad teleop、**Execution Manager** 和 episode recorder。腕部 Pika（夹爪 / 鱼眼 / D405）在 RT / beta 上起，不在本包启动。

Package path: `src/bringup/franka_manipulation/workstation_launch`  
ROS package name: `franka_manipulation_workstation_launch`

| Host | Owns |
|---|---|
| **RT**（Franka beta） | 1× FR3 + 1× Pika（夹爪 / 鱼眼 / D405）、`ros2_control` |
| **Workstation** | gamepad teleop、EM、recorder、RMI 应用；**Hikvision 7 路**（待接入） |

## 配置

- `config/execution_manager.yaml` — 唯一 EM routing table，包含 controller 和 command endpoint
- `apps/recording/franka_manipulation.yaml` — 应用选择的 MCAP 流契约（腕部 D405 对齐深度 + 鱼眼；Hikvision 稍后补）
- `apps/recording/franka_manipulation_no_cam.yaml` — 当前 fake-hardware 验证契约；相机流可选且不参与 start gate
- `config/teleop/gamepad.yaml` — Franka base frame 和 Pika gripper joint 映射
- `apps/profiles/fr3_pika_single_arm.yaml` — RMI 应用 API，通过 `execution_manager_config` 引用 routing table，不再定义 `groups`

EM 是唯一命令权威。Policy、Teleop、Planner 和 RMI 应用只写 profile 声明的普通
`/action_sources/...` topic/action；lease、抢占和 controller switching 都封装在 EM 内部。
应用不直接写 `/execution/...`，也不调用 `controller_manager`。

## 启动

RT 先起来之后，完整 workstation stack 一起启动 EM、gamepad teleop 和 recorder：

```bash
ros2 launch franka_manipulation_workstation_launch workstation_stack.launch.py
```

当前 FakeHardware + gamepad + 无相机 Gate 使用同一个 workstation stack；是否等待
相机由 RMI profile 选择的 recorder 契约决定。运行 Example 16 时使用：

Example 16 启动 policy 前会通过 Planner/JTC 用 8 s 回到 profile 的 homing 姿态。
该姿态按 `/joint_states` 的 joint name 从当前 Franka fake-hardware 初始状态记录：
`[0, -π/4, 0, -3π/4, 0, π/2, π/4]`。

```bash
python examples/16_franka_gamepad_teleop.py \
  --profile fr3_pika_single_arm.yaml
```

也可以分别启动：

```bash
ros2 launch franka_manipulation_workstation_launch execution_manager.launch.py
ros2 launch franka_manipulation_workstation_launch gamepad_teleop.launch.py
ros2 launch franka_manipulation_workstation_launch recorder.launch.py
```

## 验收

```bash
ros2 node list | grep -E 'execution_manager|episode_recorder'
ros2 topic echo /execution_manager/authority_status --once
ros2 topic hz /joint_states --window 20
```

| Sensor | Topic | Host |
|--------|-------|------|
| Wrist D405 color | `/pika_d405/camera/color/image_raw` | RT |
| Wrist D405 depth | `/pika_d405/camera/aligned_depth_to_color/image_raw` | RT |
| Wrist fisheye | `/pika_fisheye/image/compressed` | RT |
| Hikvision ×7 | 待接入 | workstation |
