# Physical AI Runtime — Bringup

此目录索引 RT Host 与 Workstation bringup。具体启动和排障请使用各机器人目录的
README。

| Robot | RT Host | Workstation |
|---|---|---|
| Marvin | [rt_launch](marvin_manipulation/rt_launch/README.md) | [workstation_launch](marvin_manipulation/workstation_launch/README.md) |
| Franka | [rt_launch](franka_manipulation/rt_launch/README.md) | [workstation_launch](franka_manipulation/workstation_launch/README.md) |
| Piper | [rt_launch](piper_manipulation/rt_launch/README.md) | [workstation_launch](piper_manipulation/workstation_launch/README.md) |

Piper 的 CAN 设备别名配置见 [UDEV_HOST_SETUP.md](../../docs/UDEV_HOST_SETUP.md)。

当前 Piper 标准三相机拓扑：

| 主机 | 设备 | RGB topic |
|---|---|---|
| 本地 `192.168.1.18` | Orbbec Femto Bolt | `/observation/orbbec/color/image_raw` |
| 本地 `192.168.1.18` | 左腕 D435i | `/observation/left_hand_realsense/color/image_raw` |
| 本地 `192.168.1.18` | 右腕 D435i | `/observation/right_hand_realsense/color/image_raw` |

NUC 双臂 CAN 实际映射为左臂 `piper1`、右臂 `piper0`。完整双主机启动和采集命令见
[Piper workstation README](piper_manipulation/workstation_launch/README.md)。
