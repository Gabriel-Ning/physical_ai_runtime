# Franka RT Host

Franka FR3 + 一只 Pika：`ros2_control`、腕部 D405 / 鱼眼。在 **beta**（`192.168.1.100`）上跑。

命令权威在 workstation 的 Execution Manager。本包只起 RT 闭环和腕部感知，不启 EM、不启 RMI 应用、不启规划器。

## 配置

- `config/controller/controllers.yaml` — 1000 Hz TSJIC (TSKPC) / JSIC (JSPC) / JTC + Pika forward（力矩接口，`with_gravity_compensation` 开关：MuJoCo 设为 true，真机 FCI 自动补偿设为 false）
- `config/controller/controllers_fake.yaml` — 仅在 fake hardware (Mock) 模式下使用的位置控制器
- `config/camera/pika_cameras.yaml` — 真机腕部 D405（序列号 `_323622270897`）+ 鱼眼
- `config/mujoco_plugins.yaml` — MuJoCo 仿真底座插件配置，腕部相机话题与真机 D405 物理命名空间（`/pika_d405/...`）保持 100% 一致

MuJoCo 仿真相机默认使用 **POSIX SHM + 独立 C++ `mujoco_image_bridge`**：

```text
ros2_control + MuJoCo CameraPlugin（output: shm）
  只渲帧 → /pai_mj_cam_<camera_name>（seqlock ring）
                     ↓ mmap
独立 mujoco_image_bridge → 公开 Image / Depth / CameraInfo → 多个业务订阅者
```

CM 不创建 Image / CameraInfo publisher。公开 `/agentview/...`、`/pika_d405/...`
名称、像素、内参、frame_id 和仿真采集时间戳保持不变。IMU / FT 仍走 ros2_control；
真机仍使用 RealSense 进程，只有 MuJoCo backend 启动 bridge。

相机列表来自同一份 `config/mujoco_plugins.yaml`：`default_policy: disabled`，
显式启用的 streaming / polled 相机自动进入 bridge。可通过 `mujoco_plugins_yaml:=<路径>`
覆盖配置；并行实例须使用不同 `shm_prefix`，ROS domain 不隔离 SHM。
`output` 支持 `ros|shm|both`；`both` 仅用于调试，会产生额外 ROS publisher。

Bridge 使用墙钟轮询（`use_sim_time: false`），保留图像中的仿真时间戳。
每个新帧最多读取一次所需图像，再由 DDS 扇出；无订阅的流不拷贝、不发布。
默认 QoS 为 best-effort、keep-last 1，与 RMI / recorder 的 sensor-data QoS 一致。
独立启动 bridge 时可用参数 `reliability:=reliable` 兼容可靠订阅者。
无订阅时插件仍渲染；按 reader 需求停止渲染留作后续优化。

本机实测发现 CycloneDDS 数据组播在增加第二个订阅进程后丢帧，SHM 产帧率不变。
因此 launch 仅向 bridge 的 `CYCLONEDDS_URI` 追加 `AllowMulticast=spdp`：
保留发现组播，图像数据走单播；继承原有 NIC / peer / buffer 配置，CM 和客户端环境不变。
需要自定义时设置 `MUJOCO_IMAGE_BRIDGE_CYCLONEDDS_URI`（完整 URI）；其他 RMW 不使用该配置。
这与 [CycloneDDS 的组播配置说明](https://cyclonedds.io/docs/cyclonedds/latest/config/multicasting.html)
一致；单播消除了本机的组播丢帧，但总网络带宽仍随接收端数量增长。


SHM ABI 以 `camera_shm.hpp` 为准：version 2、静态头 720 字节、槽头 16 字节，
槽跨度按 8 字节对齐，序号跨槽递增。Bridge 在 SHM 未就绪时 WARN，持有 fd 与 mmap，
通过 inode 检查识别生产者重启并重新 attach。旧 Python MVP 已由原生可执行文件替代，
修改 C++ 后需重新构建；部署时同时更新插件和 bridge。

启动后检查：

```bash
ros2 topic info /agentview/camera/color/image_raw --verbose
ros2 node info /mujoco_ros2_control_node
ls /dev/shm/pai_mj_cam_*
ros2 topic hz /agentview/camera/color/image_raw
```

公开 Image 的唯一 publisher 应为 `mujoco_image_bridge`，CM 不应出现 Image publisher。
分别测零订户、一个订户和两个独立进程订户，同时观察 `/clock` 的墙钟 RTF；
比较后两者帧率，不能仅比较仿真时间频率。单相机产能充足时墙钟帧率应接近
`camera_publish_rate × RTF`。双路 RGB-D 串行渲染可能受 GPU / GUI 负载限制
（例如约 15 Hz，具体依机器而定）；应结合 SHM 产帧率区分渲染瓶颈和 DDS 丢帧。
此架构隔离 CM 的图像 DDS 成本，CPU / 内存带宽仍由进程共享。

可重复验收（已 source 构建环境，在空闲 ROS domain 中运行；自动启动纯仿真并清理）：

```bash
# 客户端显式保留原始 AllowMulticast=true 配置；仅 bridge 由 launch 追加 spdp。
export CYCLONEDDS_URI="file://$PWD/.config/cyclonedds_default.xml"
unset MUJOCO_IMAGE_BRIDGE_CYCLONEDDS_URI
python src/bringup/franka_manipulation/rt_launch/test/check_camera_fanout.py --domain 98
python src/bringup/franka_manipulation/rt_launch/test/check_camera_fanout.py --domain 98 --dual-camera
python src/bringup/franka_manipulation/rt_launch/test/check_camera_fanout.py --domain 98 --gui --dual-camera
```

脚本按零 / 一 / 两个独立进程依次测量 RGB-D，输出 SHM 产帧率、接收墙钟帧率、RTF，
检查 publisher 身份和时钟回退；双订户每流须达到单订户的 90%。输出记录客户端 DDS URI，
并断言 CM 和 bridge 正常退出，避免帧率通过掩盖清理崩溃。其他依赖进程退出错误单独列出。
该检查不执行手柄运动。

验证记录（2026-09-05，本机，640×480，30 Hz 配置，隔离 ROS domain，未绑定 CPU）：

| 场景 | 单订户墙钟 FPS | 两个独立进程墙钟 FPS | RTF |
| --- | --- | --- | --- |
| 单相机 RGB-D、headless | 29.24–29.25 | 29.28 | ≈1.00 |
| 双相机 RGB-D、headless，每流 | 29.21–29.23 | 29.27–29.29 | ≈1.00 |
| 双相机 RGB-D、原生 GUI，每流 | 29.16–29.18 | 29.21–29.25 | ≈1.00 |

本轮矩阵显式使用原始 `cyclonedds_default.xml`（`AllowMulticast=true`）作为客户端配置，
仅 bridge 追加 `spdp`；没有要求客户端改为 spdp。全部公开 Image 的唯一 publisher
是 bridge，CM 无 Image publisher，无时钟回退。双相机 headless 和 GUI 退出时
CM / bridge 均正常结束，GUI 清理未再出现 `glDeleteTextures` 崩溃。
5 项 SHM 协议测试、1 项 bridge 集成测试、6 项 CameraPlugin 测试通过，
覆盖交替槽不误丢帧、跨进程写入一致性、lazy、消息内容和生产者重启重新 attach。
GUI 退出时的 `glDeleteTextures` 崩溃已修复：`GlfwAdapter` 现在于 UI 渲染线程退出前
销毁，使纹理释放时对应 OpenGL context 仍在当前线程。Simulate 对象和互斥锁保留到
物理线程结束；窗口、物理与相机线程的运行架构不变。

另以原始 `examples/16_franka_gamepad_teleop.py`（振幅 0.1 rad、周期 6 s）连接
workstation + recorder，在 headless 双相机仿真下运行约 15 秒录制。
RMI、recorder 和监测进程同时订阅两路 RGB：约 29.27 FPS，RTF 0.99995，
12 秒测量窗内 1200 条臂参考，最大当前参考与关节位置差约 0.0041 rad。
Recorder 每路收到并写入 437 帧，零 recorder 丢帧。
该次初测因无接管事件而归档失败的问题已修正：Franka 的 real / mujoco / no_cam 合同
将 `authority_events` 设为可空事件流，发生事件时仍录制并校验写入计数；持续发布的
`authority_status` 和原有必需传感器流保持必需检查。未改 recorder 的通用校验逻辑，
也不会为满足合同而伪造接管事件。

修正后复测原始 RMI 示例并完成一次无接管录制：两路 RGB 约 29.29 FPS，RTF 0.99999，
最大当前参考与关节位置差约 0.0037 rad；`authority_events` 为零条时健康检查 PASS，
实际 MCAP episode 成功 finalize，应用正常返回 0。28 项配置/归档校验回归测试通过。





- `config/model/gripper_tcp.yaml`、`config/model/joint_limits.yaml`（单指行程 `[0.0, 0.045]`，契约 `0 = closed pose`，模型已修正原点确保零位闭合接触）

夹爪 / 鱼眼默认走 udev 稳定名（`scripts/udev/rules.d/99-pika.rules`，beta `usb-0:6.*`）：

`/dev/pika_left_gripper`、`/dev/pika_left_fisheye`

不要用 `/dev/ttyUSB*` / `/dev/video*`。D405 按 YAML 里的 `serial_no` 识别。

## 启动

根据目标环境通过 `backend` 或兼容参数选择底层形态（`real` / `mujoco` / `fake`）：

### 1. 真机环境（在 RT 工控机 beta 上运行）

```bash
ros2 launch franka_manipulation_rt_launch rt_stack.launch.py \
  backend:=real \
  use_fake_hardware:=false \
  robot_ip:=192.168.2.101 \
  with_cameras:=true \
  use_rviz:=false
```

默认 `with_cameras:=true`（腕部 D405 + 鱼眼）。路由控制器和 `pika_gripper_fwd` 默认处于 **inactive** 状态，由工作站端的 Execution Manager 动态激活切换。

### 2. MuJoCo 仿真环境（在仿真环境或 Workstation 上运行）

```bash
# 启动包含指定 LIBERO 任务场景的 MuJoCo 物理动力学底座
ros2 launch franka_manipulation_rt_launch rt_stack.launch.py \
  backend:=mujoco \
  task:=pick_up_the_black_bowl_between_the_plate_and_the_ramekin_and_place_it_on_the_plate \
  headless:=false
```

启动后可在另一终端直接启动工作站调度层。

### 3. Fake / Mock 环境（本机纯运动学无动力学调试）

```bash
ros2 launch franka_manipulation_rt_launch rt_stack.launch.py \
  backend:=fake \
  with_cameras:=false \
  use_rviz:=true
```

### 4. 单独拉起控制器或相机

```bash
ros2 launch franka_manipulation_rt_launch controller_bringup.launch.py backend:=real
ros2 launch franka_manipulation_rt_launch camera_bringup.launch.py
```

### 5. 状态检查

```bash
ros2 control list_controllers
ros2 topic hz /joint_states --window 20
```

细节与硬件调试指南请参阅：[docs/BRINGUP.md](docs/BRINGUP.md)。

## Realtime / FCI 通信

Franka FCI 对主机抖动很敏感，常见报错是 `communication_constraints_violation`。
beta 上要把 **Franka 网卡（`192.168.2.x` / `enp2s0`）IRQ** 与 **`ros2_control`**
和 DDS/相机负载拆开。完整说明与脚本：

[`docs/FRANKA_RT_COMMUNICATION.md`](../../../../docs/FRANKA_RT_COMMUNICATION.md)

```bash
# 在 beta（Franka RT 主机）上一次性配置 + 重启
sudo bash scripts/apply_franka_rt_host.sh && sudo reboot
```
