# Franka RT Host

Franka FR3 + 一只 Pika：`ros2_control`、腕部 D405 / 鱼眼。在 **beta**（`192.168.1.100`）上跑。

命令权威在 workstation 的 Execution Manager。本包只起 RT 闭环和腕部感知，不启 EM、不启 RMI 应用、不启规划器。

## 配置

- `config/controller/controllers.yaml` — 1000 Hz JSIC / TSJIC / JTC + Pika forward
- `config/controller/controllers_fake.yaml` — fake hardware 位置控制器
- `config/camera/pika_cameras.yaml` — 腕部 D405（序列号 `_323622270897`）+ 鱼眼
- `config/model/gripper_tcp.yaml`、`config/model/joint_limits.yaml`

夹爪 / 鱼眼默认走 udev 稳定名（`scripts/udev/rules.d/99-pika.rules`，beta `usb-0:6.*`）：

`/dev/pika_left_gripper`、`/dev/pika_left_fisheye`

不要用 `/dev/ttyUSB*` / `/dev/video*`。D405 按 YAML 里的 `serial_no` 识别。

## 启动

RT 主机用 Pixi **`cpu`** 环境（`pixi install --locked -e cpu`）。

```bash
ros2 launch franka_manipulation_rt_launch rt_stack.launch.py \
  use_fake_hardware:=false \
  robot_ip:=192.168.2.101 \
  use_rviz:=false
```

默认 `with_cameras:=true`（腕部 D405 + 鱼眼）。路由控制器和 `pika_gripper_fwd` 都是 **inactive**，等 workstation EM claim 后再切。夹爪默认 `/dev/pika_left_gripper`，不要再传 `/dev/ttyUSB0`。

Fake / 本机（在 **workstation** 上跑，不要开真相机）：

```bash
ros2 launch franka_manipulation_rt_launch rt_stack.launch.py \
  use_fake_hardware:=true \
  load_pika_hardware:=true \
  with_cameras:=false \
  use_rviz:=true \
  cpu_affinity:=none
```

该组合加载 fake FR3 arm 和 fake Pika gripper，并关闭相机。

当前验证 Gate 是 gamepad teleop + EM 抢占 + 无相机录制。RT 启动后继续按
[`examples/README.md`](../../../../examples/README.md#franka-fake-hardware-gamepad-setup)
启动 workstation stack 和 Example 16。

只起控制器或只起相机：

```bash
ros2 launch franka_manipulation_rt_launch controller_bringup.launch.py \
  use_fake_hardware:=false
ros2 launch franka_manipulation_rt_launch camera_bringup.launch.py
```

```bash
ros2 control list_controllers
ros2 topic hz /joint_states --window 20
```

细节：[docs/BRINGUP.md](docs/BRINGUP.md)。

## Realtime / FCI 通信

Franka FCI 对主机抖动很敏感，常见报错是 `communication_constraints_violation`。
beta 上要把 **Franka 网卡（`192.168.2.x` / `enp2s0`）IRQ** 与 **`ros2_control`**
和 DDS/相机负载拆开。完整说明与脚本：

[`docs/FRANKA_RT_COMMUNICATION.md`](../../../../docs/FRANKA_RT_COMMUNICATION.md)

```bash
# 在 beta（Franka RT 主机）上一次性配置 + 重启
sudo bash scripts/apply_franka_rt_host.sh && sudo reboot
```
