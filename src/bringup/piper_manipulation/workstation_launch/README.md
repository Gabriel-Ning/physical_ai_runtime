# piper_manipulation_workstation_launch

Piper 工作站启动入口，负责 Execution Manager（EM）、recorder、相机和 leader。
机械臂控制与 MuJoCo 仿真由同级 [RT 启动包](../rt_launch/README.md) 负责。

## 运行场景

| 组件 | 真机 | MuJoCo 仿真 |
| --- | --- | --- |
| Execution Manager | 常驻 | 常驻 |
| Recorder | 常驻 | 常驻 |
| Orbbec Femto Bolt（`config/camera/femto_bolt.yaml`） | 启动 | 不启动 |
| RealSense D435（`config/camera/realsense_d435.yaml`） | 启动 | 不启动 |
| 左右 Piper leader | 启动 | 不启动 |

Recorder 常驻表示录制服务持续运行；每个 episode 的开始和结束由应用控制。

## 环境准备

在工作站的每个新终端执行，使用包含相机驱动的 `default` 环境：

```bash
cd ~/Documents/physical_ai_runtime
pixi shell -e default
source install/setup.bash
```

启动完整工作站栈前，先启动对应的 RT 真机或 MuJoCo 栈。

## 真机：EM、recorder、三台相机和 leader

```bash
ros2 launch piper_manipulation_workstation_launch workstation_stack.launch.py
```

所有启动参数默认按真机场景配置：EM、recorder、相机和
左右 leader 全部启动，使用系统时间。真机启动无需额外参数。

## MuJoCo 仿真：常驻 EM 和 recorder

```bash
ros2 launch piper_manipulation_workstation_launch workstation_stack.launch.py \
  use_sim_time:=true \
  with_orbbec:=false with_realsense:=false with_leaders:=false
```

EM 和 recorder 使用 MuJoCo 发布的 `/clock`，仿真启动后可检查：

```bash
ros2 topic echo /clock --once
```

## 单独测试相机或 leader

以下命令用于分别测试外设，每条命令在独立终端运行。
如果完整工作站栈已启动对应组件，不要重复启动。

静态 Orbbec Femto Bolt：

```bash
ros2 launch piper_manipulation_workstation_launch piper_orbbec.launch.py
```

实例在 `config/camera/femto_bolt.yaml` 中逐项定义。顶层键就是 `camera_name`，
每项自带 `camera_namespace`、`serial_number` 和 `parameters`。新增同型号相机时，
复制一项并改名称 / 序列号即可，不需要改 launch。

RealSense D435：

```bash
ros2 launch piper_manipulation_workstation_launch piper_realsense.launch.py
```

相机实例在 `config/camera/realsense_d435.yaml` 中按型号逐项定义。顶层键就是相机名称，
每项独立设置 `startup_delay`、`sdk_log_level` 和 `parameters`，没有公共参数或继承。
当前工位是左右腕部两台，分别立即启动和延迟 10 秒启动；文件本身不限制相机数量。

新增相机时，复制一项，修改顶层名称、`parameters.serial_no` 和启动延迟即可；
各相机的曝光、分辨率、帧率等参数可独立修改，不需要更改 launch。
序列号用带下划线的字符串，例如 `"_332522075913"`。

```yaml
front_realsense:
  startup_delay: 20.0
  sdk_log_level: ERROR
  parameters:
    camera_namespace: observation
    serial_no: "_新相机序列号"
    enable_color: true
    enable_depth: false
    rgb_camera.color_profile: "640x480x30"
```

未填写的驱动参数使用 RealSense 驱动默认值。旧的 `left_serial_no`、
`right_serial_no`、`right_camera_delay` 等固定双相机 launch 参数已移除。
其它型号应使用独立的型号 YAML，而不是塞进这份 D435 文件。

左右 leader：

```bash
ros2 launch piper_manipulation_workstation_launch piper_leaders.launch.py
```

相机启动后，在另一个终端逐条检查帧率，按 `Ctrl+C` 结束当前检查：

```bash
ros2 topic hz /observation/static_orbbec/color/image_raw
ros2 topic hz /observation/left_hand_realsense/color/image_raw
ros2 topic hz /observation/right_hand_realsense/color/image_raw
```

三路图像均应持续输出；同时检查驱动日志中的设备序列号是否与左右相机对应。

## 单独启动常驻服务

需要分别查看 EM 和 recorder 日志时，可用两个终端代替完整栈中的服务启动：

```bash
ros2 launch piper_manipulation_workstation_launch execution_manager.launch.py
```

```bash
ros2 launch piper_manipulation_workstation_launch recorder.launch.py
```

MuJoCo 场景下，两条命令都加上 `use_sim_time:=true`。
如果同时使用 `workstation_stack.launch.py` 启动外设，需设置
`with_execution_manager:=false with_recorder:=false`，避免重复启动常驻服务。

## 配置位置

`config/` 是各子 launch 的唯一参数入口。每个子 launch（包括单独启动）从自己的
YAML 读取参数；`workstation_stack.launch.py` 只声明是否启动以及 `use_sim_time`。
命令行参数仅作本次启动的临时覆盖。

Recorder 没有包内 YAML，直接启动 `episode_recorder` 自己的 launch（与 Franka 相同）。

以下 `config/` 路径相对于本包，`apps/` 路径相对于仓库根目录。
`apps/` 的配置只供应用选择录制内容，不参与这些子 launch 的默认参数加载。

| 配置 | 用途 |
| --- | --- |
| `config/execution_manager.yaml` | EM 控制器路由与控制源准入 |
| `config/camera/femto_bolt.yaml` | Femto Bolt 实例（名称、命名空间、序列号、驱动参数） |
| `config/camera/realsense_d435.yaml` | RealSense D435 实例与独立参数 |
| `config/teleop/piper_leaders.yaml` | 左右 leader 配置 |
| `apps/profiles/piper_bimanual.yaml` | Piper 双臂应用配置 |
| `apps/recording/piper_bimanual.yaml` | 含相机流的录制 contract |
| `apps/recording/piper_bimanual_no_cam.yaml` | 无相机录制 contract |

录制 contract 由应用选择；关闭相机 launch 不会自动切换 contract。
真机录制完成后，检查 `episode_health.json` 中 `static_orbbec`、
`left_wrist_cam`、`right_wrist_cam` 的流计数，以及 `recorder_drops` 是否为 `0`。

## 启动日志排查

### Leader 提示 `Error reading from CAN socket: Network is down`

左右 leader 使用 `can0`、`can1`。该错误表示 CAN 接口没有启用。
先在运行 leader 的终端按 `Ctrl+C` 停止节点（完整栈启动时停止该栈），
确认 leader 周围可安全运动，再在仓库根目录恢复接口：

```bash
sudo bash scripts/reset_teleop_piper_can.sh can0 can1
ip -details link show can0
ip -details link show can1
```

接口应包含 `UP` 标志，CAN bitrate 为 `1000000`，随后重新启动工作站栈。
Leader 默认进入 shadow 模式，会跟随 `/joint_states`。

### RealSense 提示缺少 `.realsense-config.json`

Librealsense 2.57.7 在缺少用户配置时会输出
`No valid configuration file found ... loading defaults`。
这与相机图像参数 YAML 是不同的配置。若该文件不存在，可创建空 JSON 对象以使用 SDK 默认值：

```bash
python3 - <<'PYCONFIG'
from pathlib import Path
path = Path.home() / ".realsense-config.json"
if not path.exists():
    with path.open("x") as stream:
        stream.write("{}\n")
PYCONFIG
```

已有配置应保留；若已有文件仍报错，检查其 JSON 格式。
