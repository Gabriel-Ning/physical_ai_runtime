# Marvin RT Host

Marvin 双臂 + 双 Pika：`ros2_control`、腕部 D405 / 鱼眼。在 **gamma** 上跑。

## 配置

- `config/controller/controllers.yaml` — 500 Hz JTC / JSPC / TSKPC + Pika forward
- `config/camera/pika_d405.yaml` — 腕部 D405（序列号）
- `config/camera/pika_fisheye.yaml` — 腕部鱼眼（`/dev/pika_*_fisheye`）
- `config/model/gripper_tcp.yaml`、`config/model/joint_limits.yaml`

夹爪串口默认 `/dev/pika_left_gripper`、`/dev/pika_right_gripper`（udev `99-pika.rules`）。

## 启动

```bash
ros2 run marvin_manipulation_rt_launch clear_errors --ip 10.19.0.191

ros2 launch marvin_manipulation_rt_launch rt_stack.launch.py \
  use_fake_hardware:=false \
  use_rviz:=false
```

路由控制器和 `*_pika_gripper_fwd` 都是 **inactive**。腕部相机在这个 launch 里起：两路鱼眼和左 D405 立刻起，右 D405 默认晚 2 s（`right_d405_delay`）。

单侧夹爪测试（默认仍是双爪）：

```bash
ros2 launch marvin_manipulation_rt_launch rt_stack.launch.py \
  use_fake_hardware:=false \
  use_rviz:=false \
  with_right_gripper:=false
```

只起控制器或只起相机：

```bash
ros2 launch marvin_manipulation_rt_launch controller_bringup.launch.py \
  use_fake_hardware:=false
ros2 launch marvin_manipulation_rt_launch pika_camera_bringup.launch.py
```

```bash
ros2 control list_controllers
ros2 topic hz /joint_states --window 20
```

细节：[docs/BRINGUP.md](docs/BRINGUP.md)。
