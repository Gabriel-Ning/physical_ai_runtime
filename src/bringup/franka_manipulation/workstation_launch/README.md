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
- `apps/recording/franka_manipulation_real.yaml` — 物理实物真机录制契约（腕部 D405 对齐深度 + 鱼眼）
- `apps/recording/franka_manipulation_mujoco.yaml` — MuJoCo 仿真录制契约（腕部 D405 对齐深度 + agentview 全局视角）
- `config/teleop/gamepad.yaml` — Franka base frame 和 Pika gripper joint 映射
- `apps/profiles/fr3_pika_single_arm.yaml` — RMI 应用 API，通过 `execution_manager_config` 引用 routing table，不再定义 `groups`

EM 是唯一命令权威。Policy、Teleop、Planner 和 RMI 应用只写 profile 声明的普通
`/action_sources/...` topic/action；lease、抢占和 controller switching 都封装在 EM 内部。
应用不直接写 `/execution/...`，也不调用 `controller_manager`。

## 启动

RT 先起来之后再启本包。

真机 / Fake（墙钟，默认）：

```bash
ros2 launch franka_manipulation_workstation_launch workstation_stack.launch.py
```

MuJoCo 仿真（必须跟 RT 的 `/clock` 对齐）：

```bash
ros2 launch franka_manipulation_workstation_launch workstation_stack.launch.py \
  use_sim_time:=true
```

| 场景 | `use_sim_time` | EM 日志应出现 |
|------|----------------|---------------|
| 真机 / Fake | `false`（默认） | `Time source: system wall clock` |
| MuJoCo | `true` | `Time source: ROS simulation clock (/clock)` |

仿真若漏开 `use_sim_time:=true`，EM 用墙钟做 `max_command_age_s`，会把带仿真时间戳的命令判成
`stale_command`，策略 / teleop 看起来「没效果」。

也可以分别启动（同样按上表设 `use_sim_time`）：

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
