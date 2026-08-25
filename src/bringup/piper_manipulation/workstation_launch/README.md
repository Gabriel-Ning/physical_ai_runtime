# piper_manipulation_workstation_launch

Piper 工作站侧 bringup：双 Leader 臂、1 路 Orbbec Femto Bolt、2 路腕部
RealSense D435i、Execution Manager 和 episode recorder。机械臂 follower 的
`ros2_control` 仍由 `piper_manipulation_rt_launch` 管理。

## 配置归属

- `config/execution_manager.yaml`：四个执行资源与 `/execution/...` 路由；人工接管和策略推理由 RMI lease/preempt 决定。
- `config/recording/rmi_piper_bimanual.yaml`：MCAP stream contract。
- `config/camera/femto_bolt.yaml`：静态 Orbbec 参数。
- `config/camera/d435i_dual.yaml`：左右腕部 RealSense 参数。
- `config/teleop/piper_leaders.yaml`：Leader CAN 与发布话题默认值。
- `apps/profiles/piper_bimanual.yaml`：应用、传感器、teleoperator 和 recorder client 配置；通过 `execution_manager_config` 引用路由表。

## 一键启动

先启动 RT Host，再在 workstation 执行：

```bash
ros2 launch piper_manipulation_workstation_launch workstation_stack.launch.py
```

该入口固定启动全部工作站子系统，不提供 `with_*` 开关。需要单独调试时使用：

```bash
ros2 launch piper_manipulation_workstation_launch piper_leaders.launch.py
ros2 launch piper_manipulation_workstation_launch piper_orbbec.launch.py
ros2 launch piper_manipulation_workstation_launch piper_realsense.launch.py
ros2 launch piper_manipulation_workstation_launch recorder.launch.py
ros2 launch piper_manipulation_workstation_launch execution_manager.launch.py
```

要在连接真机传感器的情况下验证完整工作站图、但不激活 Leader 电机，请使用：

```bash
ros2 launch piper_manipulation_workstation_launch workstation_stack.launch.py autostart:=false
```

该参数只阻止 Leader 节点打开 CAN / 启动控制环；其余服务（Execution Manager、
recorder、Orbbec、两路 D435i）都会启动。

## 无运动检查

Execution Manager 本身只仲裁和转发命令，不会主动生成运动命令。启动后先检查：

```bash
ros2 node list | grep -E 'execution_manager|episode_recorder|piper_leader'
ros2 topic list | grep -E '^/observation/|^/execution_manager/'
ros2 topic hz /observation/static_orbbec/color/image_raw
ros2 topic hz /observation/left_hand_realsense/color/image_raw
ros2 topic hz /observation/right_hand_realsense/color/image_raw
```

预期三个 RGB topic 均约 30 Hz。开始录制后检查 `episode_health.json`：三个相机
stream count 应增长，`recorder_drops` 应为 0。

Leader 进入 teleop 后，客户端必须逐一打印左右臂的 `Preempt ACTIVE` 成功回执；任
一侧超时或拒绝时会撤销已取得的 lease，不能开始遥操。可用以下命令确认驱动真实
启动了重力补偿控制环：

```bash
ros2 topic echo /teleop/piper_leader_left/status --once
ros2 topic echo /teleop/piper_leader_right/status --once
```

两条状态都应包含 `"mode":"active_preempt"` 和
`"gravity_compensation_running":true`。D435i 在 USB 枚举延迟期间会等待最多 30 秒，
短暂断连后每 2 秒重连；启动后仍应以上述 `ros2 topic hz` 对两个腕部 topic 分别验
证约 30 Hz。

真实 Leader 接管前还应确认 `can1`/`can0` 分别对应左/右 Leader，RT follower
控制器状态与预期一致，并保持急停可用。
