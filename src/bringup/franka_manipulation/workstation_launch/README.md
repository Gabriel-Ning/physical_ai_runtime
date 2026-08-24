# Franka Workstation

Workstation 常驻 **Execution Manager** 和 episode recorder。腕部 Pika（夹爪 / 鱼眼 / D405）在 RT / beta 上起，不在本包启动。

Package path: `src/bringup/franka_manipulation/workstation_launch`  
ROS package name: `franka_manipulation_workstation_launch`

| Host | Owns |
|---|---|
| **RT**（Franka beta） | 1× FR3 + 1× Pika（夹爪 / 鱼眼 / D405）、`ros2_control` |
| **Workstation** | EM、recorder、RMI 应用；**Hikvision 7 路**（待接入） |

## 配置

- `config/recording/franka_manipulation.yaml` — MCAP 流契约（腕部 D405 对齐深度 + 鱼眼；Hikvision 稍后补）
- `apps/profiles/fr3_pika_single_arm.yaml` — RMI 应用 profile；workstation launch 默认值从这里读

EM 是唯一命令权威。RMI 只做 claim/lease 客户端，不直接写 `/execution/...`，也不调 `controller_manager`。

## 启动

RT 先起来之后：

```bash
ros2 launch franka_manipulation_workstation_launch franka_workstation.launch.py
```

只起 Execution Manager：

```bash
ros2 launch franka_manipulation_workstation_launch franka_workstation.launch.py \
  with_recorder:=false
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
