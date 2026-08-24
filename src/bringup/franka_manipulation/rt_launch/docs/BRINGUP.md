# FR3 + Pika RT bringup

当前架构（与 Marvin 同一套边界）：

```text
Policy / Teleop / Planner  (workstation, RMI)
        -> Execution Manager claim/lease
        -> /execution/<resource>/...
        -> ros2_control on RT host
        -> FR3 + Pika
```

- workstation 常驻 C++ Execution Manager；RMI 是 Python client SDK
- 本包只起 `xacro` + `ros2_control_node` + 腕部相机
- 不在 RT 上启 EM、recorder、规划器或应用节点
- 路由控制器启动时全部 **inactive**；EM claim 成功后才切换

Launch 命令见 [../README.md](../README.md)。RT 主机用 Pixi **`cpu`** 环境；workstation 用 `default` / `runtime`。

## Controllers that are spawned

Serialized: `joint_state_broadcaster` → inactive `franka_arm_tsjic` /
`franka_arm_jsic` / `franka_arm_jtc`。`load_pika_hardware:=true` 时还有
`pika_gripper_fwd`（同样 inactive）。

**Not spawned:** `franka_robot_state_broadcaster`。EM / RMI 只需要 `/joint_states`。

## Joint-state remap

```text
joint_state_broadcaster  --remap-->  /franka/joint_states
joint_state_publisher    source_list:
                           franka/joint_states
                           franka_gripper/joint_states
                         --> /joint_states
```

## Fake vs real

| Role | Real HW | Fake HW |
|---|---|---|
| Policy | `franka_arm_jsic` effort | 同名，位置 |
| Teleop | `franka_arm_tsjic` effort | 同名，位置 |
| Planner | `franka_arm_jtc` effort | 同名，位置 |

`use_fake_hardware:=true` 加载 `config/controller/controllers_fake.yaml`。名字和 `/execution/...` endpoint 不变。

## Perception cameras

腕部相机在 **RT**。默认 `with_cameras:=true`。

- D405：`/pika_d405/camera/color/image_raw`，`/pika_d405/camera/aligned_depth_to_color/image_raw`（serial `_323622270897`，用 `rs-enumerate-devices`）
- 鱼眼：`/pika_fisheye/image/compressed`

设备节点：`/dev/pika_left_gripper`、`/dev/pika_left_fisheye`（udev `usb-0:6.*`）。不要用 `/dev/ttyUSB*` / `/dev/video*`。

Hikvision 挂 workstation，尚未接入。

## CPU affinity

真机 RT 不要 `cpu_affinity:=none`。空值走 `RT_CM_CPU_AFFINITY`（见 [docs/CPU_HOST_SETUP.md](../../../../docs/CPU_HOST_SETUP.md)）。

仓库里 `scripts/rt_cpu_profile.env` 默认仍是 Marvin 8 核。**beta 本机**改成 Franka preset `12-15`，不要把这份主机文件提交回公共默认。

## Distributed DDS

Workstation 和 RT 必须同：

- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `ROS_DOMAIN_ID=1`（Pixi `[activation.env]`）
- `CYCLONEDDS_URI` 指向本机 Cyclone 文件

绑定 **机器人网**。beta 本机地址是 `192.168.1.100`；不要把 Marvin 的 `cyclonedds_default.xml`（`192.168.1.13` / peer `192.168.1.102`）直接用在这台机器上。从 `.config/cyclonedds_template.xml` 拷一份本机配置。

```bash
ros2 daemon stop
ros2 topic list
```

## Check

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

`franka_arm_*` 和 `pika_gripper_fwd` 必须是 `inactive`。
`/execution_manager/authority_status` 只在 workstation 起 EM 之后出现。
