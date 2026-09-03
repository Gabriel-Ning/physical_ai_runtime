# piper_manipulation_workstation_launch

Piper 当前标准链路使用三路相机：一台 Orbbec 顶部相机和左右腕部两台 RealSense。
相机全部连接工作站；NUC 只运行 follower 双臂 RT stack。两台主机必须使用相同的
`ROS_DOMAIN_ID`（当前为 `1`），录制前时钟偏差应不超过 100 us。

## 设备与话题

| 主机 | 设备 | RGB topic |
|---|---|---|
| 本地 `192.168.1.18` | Orbbec Femto Bolt `CL8384201CG` | `/observation/orbbec/color/image_raw` |
| 本地 | 左腕 D435i `332522075913` | `/observation/left_hand_realsense/color/image_raw` |
| 本地 | 右腕 D435i `332322073584` | `/observation/right_hand_realsense/color/image_raw` |

NUC 双臂的实际 CAN 映射为左臂 `piper1`、右臂 `piper0`。本地 Leader 默认左臂
`can1`、右臂 `can0`。

## 1. NUC：启动 Follower 双臂

NUC 仓库为 `/home/delta/Documents/physical_ai_runtime`：

```bash
cd /home/delta/Documents/physical_ai_runtime
pixi run -e cpu bash -lc \
  'source install/setup.bash && ros2 launch piper_manipulation_rt_launch rt_stack.launch.py use_fake_hardware:=false left_can_interface:=piper1 right_can_interface:=piper0 use_rviz:=false'
```

三相机主链路不要启动 `piper_external_cameras.launch.py`，也不要求 NUC 上的
`d435i1`、`d435i2` topic。

## 2. 本地：一键启动标准工作站

本地仓库固定为 `/home/alpha/physical_ai_runtime`，ACT 独立位于 `/home/alpha/ACT`。

```bash
cd /home/alpha/physical_ai_runtime
ros2 launch piper_manipulation_workstation_launch workstation_stack.launch.py
```

这个标准入口一次启动：

- 左右两台 Leader（shadow/teleop）
- Orbbec 与左右腕部 D435i
- Execution Manager
- episode recorder daemon

只查看相机且不启动机械臂/服务时，可运行：

```bash
ros2 launch piper_manipulation_workstation_launch piper_local_cameras.launch.py
```

在 RViz2 中查看三路画面：

```bash
pixi run -e runtime rviz2
```

## 3. 遥操作或三相机采集

标准 `piper_bimanual.yaml` 只包含上述三个相机。NUC 双臂和本地标准工作站都运行后，
另开本地终端执行：

```bash
cd /home/alpha/physical_ai_runtime
pixi run -e runtime python apps/teleop.py
```

或采集十条：

```bash
cd /home/alpha/physical_ai_runtime
pixi run -e runtime python apps/record.py \
  --profile piper_bimanual.yaml --task bimanual_pickup --episodes 10
```

recorder 的三路相机都设为 `required` / `start_gate`，少任何一路都不会开始写有效
episode。`d435i1`、`d435i2` 不参与新数据录制、转换、训练或部署。

## 4. 最小验收检查

```bash
ros2 node list | grep -E 'execution_manager|episode_recorder|piper_leader'
ros2 topic list | grep -E '/observation/(orbbec|left_hand_realsense|right_hand_realsense)'
ros2 topic hz /observation/orbbec/color/image_raw
ros2 topic hz /observation/left_hand_realsense/color/image_raw
ros2 topic hz /observation/right_hand_realsense/color/image_raw
```

三路 RGB 预期约 30 Hz。正式采集前再运行：

```bash
scripts/sync_clock --status delta@192.168.1.101
```

输出必须为 `GOOD`。三相机的数据录制、转换和 ACT 部署步骤见
`/home/alpha/physical_ai_runtime/apps/README.md`。

## 配置归属

- `config/execution_manager.yaml`：四个执行资源与 `/execution/...` 路由。
- `config/recording/rmi_piper_bimanual.yaml`：三相机 MCAP stream contract。
- `config/camera/femto_bolt.yaml`：Orbbec 参数。
- `config/camera/d435i_dual.yaml`：左右腕部 RealSense 参数。
- `config/teleop/piper_leaders.yaml`：Leader CAN 与发布话题。
- `apps/profiles/piper_bimanual.yaml`：应用、三相机、teleoperator 和 recorder client 配置。

`five_camera_workstation.launch.py`、`piper_external_cameras.launch.py`、
`rmi_piper_bimanual_five_camera.yaml` 和 `piper_bimanual_five_camera.yaml` 仅保留用于复现
历史五相机实验，不属于当前标准入口。
