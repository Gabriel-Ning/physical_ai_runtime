# Physical AI Runtime Applications Suite (`apps/`)

Production-ready, profile-driven CLI applications built on top of the **RMI Python SDK**.

当前 runtime 应用任务仍处于逐项验收阶段，不在 `pixi.toml` 中开放 `teleop`、
`record`、`replay`、`eval` 快捷任务。请在 runtime 环境中显式运行对应脚本。

## 环境怎么选

- `default` / `runtime`：ROS 2 bringup、Execution Manager、相机、录制、回放和
  日常调试。进入项目后由 direnv 自动激活的通常就是这个环境。
- `lerobot`：只在转换 LeRobot 数据集、加载 ACT/SmolVLA checkpoint、GPU
  推理或训练时使用。它不是机械臂和相机 bringup 环境。
- 推荐用 `pixi run -e lerobot <task>` 执行单条命令；如果使用
  `pixi shell -e lerobot`，完成后输入 `exit` 回到外层 runtime shell。

当前机器尚未通过机器人网连接 NUC 时，只运行 `--help`、转换器或有 checkpoint
的 `--dry-run`。不要启动真实推理、人工接管、录制动作或 RT 机械臂测试。

---

## 1. `apps/teleop.py` — Interactive Robot Teleoperation

One-command startup for real-time Master-Slave teleoperation (Piper Leader Dual-Arm, Keyboard, SpaceMouse):

```bash
# Piper Bimanual Master-Slave Teleoperation (200 Hz default):
pixi run -e runtime python apps/teleop.py

# Teleoperate single arm via keyboard or custom side:
pixi run -e runtime python apps/teleop.py --side left --left-can can0
```

### Teleoperation Mode Flow

```
[启动 / 启动主手驱动]
       │
       ▼
  Shadow Tracking Mode (主手伺服跟随从手位姿)
       │
       ▼
  Active Preempt Mode (Admit TeleopJoint，主手 500Hz 0-G 浮动，100~200Hz 高频中继至从手)
       │
       ▼
[按 Ctrl+C 退出] -> 自动释放 Preempt 回到 Shadow/下电，安全清理退出
```

---

## 2. `apps/record.py` — Multi-Modal Episode Dataset Recorder

Interactive demonstration data collection workflow with **Quintic Spline Staging**, **Zero-Drop Preemption**, and **Parallel MCAP Sealing**:

```bash
# Record 10 bimanual demonstrations (default 50 Hz):
pixi run -e runtime python apps/record.py \
  --profile piper_bimanual.yaml --task bimanual_pickup --episodes 10

# Another task (rate/homing defaults come from the profile):
pixi run -e runtime python apps/record.py \
  --task cup_stacking --episodes 10
```

`--task` 同时决定 recorder 的任务标签和数据集子目录。例如：

```bash
pixi run -e runtime python apps/record.py \
  --profile piper_bimanual.yaml --task pick_bread --episodes 20
```

该命令写入 `data/episodes/pick_bread/episode_*`，每条 episode manifest 中的
task 也为 `pick_bread`。采集另一个任务时换一个 `--task`，例如
`place_bread`，两类数据不会混到同一目录。任务名允许中文和空格，但不能包含
`/`、`\\`，也不能是 `.` 或 `..`。

### Episode Lifecycle & Teleoperation Recording Flow

```
[启动 / 周期重置]
       │
       ▼
 1. 从手平滑归位 (五次多项式无冲击平滑运动至 Home 姿态，如 [0, 0.5, -0.5, 0, 0, 0] rad)
    + 主手处于 Shadow 模式同步物理镜像伺服到桌前相同位置
       │
       ▼
 2. 准备就绪，等待操作员按 [ENTER] 开始
       │
       ▼
 3. 主手切入 0-G 浮动 (Active Teleop) + 启动多模态数据录制 (MCAP)
    实时显示时长与录制帧数 (预建流零丢帧保障)
       │
       ▼
 4. 操作员完成动作，再次按 [ENTER] 结束本条 Episode
       │
       ▼
 5. 质量确认与落盘：
    - [S]ave (默认/回车) -> 校验并保存 MCAP，Episode +1
    - [D]iscard          -> 丢弃重试本条
    - [R]eplay           -> 从手 1:1 回放校验动作质量 (主手同步 Shadow 镜像)
    - [Q]uit             -> 退出采集
       │
       ▼
 6. 主手切回 Shadow，从手再次平滑回初始 Home 姿态 (与步骤5落盘并发异步进行)，进入下一个 Episode 循环
```

### Key Technical Highlights:

1. **Zero-Drop Guaranteed Start**: The ROS 2 MCAP subscription gate is primed *before* the leader enters 0-G float, ensuring initial motion frames ($t=0$) are 100% captured.
2. **Parallel Return & I/O**: When an episode finishes, the robot immediately starts returning to Home in a background thread while MCAP serialization and user input review proceed in parallel, eliminating operator idle wait time.
3. **Pendant / Gripper Contract**: Follower grippers operate in `[0.0, 0.04] m` per-finger space ($0 \sim 80\text{ mm}$ total opening width), strictly matching real hardware limits.
4. **Interactive Quality Inspection**:
   * `[S]ave` (Default / Enter): Commit `.mcap` file and advance episode counter.
   * `[D]iscard & Retry`: Remove bad demo and repeat the current episode number.
   * `[R]eplay`: Re-execute recorded trajectory on follower while leader mirrors in Shadow mode.
   * `[Q]uit`: Safely conclude the dataset session.

---

## 3. `apps/replay.py` — 1:1 Native Trajectory Replayer

Performs strict 1:1 native timestamp-paced trajectory replay directly from recorded MCAP datasets on real or simulated robot embodiments:

```bash
# Replay latest recorded episode at 1:1 native rate:
pixi run -e runtime python apps/replay.py --profile piper_bimanual.yaml

# Replay specific MCAP episode:
pixi run -e runtime python apps/replay.py --profile piper_bimanual.yaml \
    --mcap-file data/episodes/piper_bimanual_teleop/episode_000001.mcap
```

---

## 4. `apps/eval.py` — Read-only Deployment Evaluation

Checks joint-state availability and age, hardware diagnostics, camera readiness,
and the current provider allocation map without acquiring control or publishing a
command:

```bash
pixi run -e runtime python apps/eval.py \
  --profile piper_bimanual.yaml --duration 10 --check-cameras
```

## 5. Embodiment Profiles (`apps/profiles/`)

All applications are fully decoupled and driven by YAML Embodiment Profiles stored in [`apps/profiles/`](profiles/):

* `piper_bimanual.yaml` — Dual-arm Piper bimanual setup with master-slave leader teleoperation.
* `fr3_pika_single_arm.yaml` — Single-arm Franka Research 3 with Pika Gripper.
* `marvin_bimanual.yaml` — Marvin humanoid dual-arm bimanual setup.

---

## 6. MCAP 转 LeRobot v3

`scripts/convert_episode_to_lerobot.py` 将 Piper recorder 生成的 episode MCAP
同步为 30 Hz（可配置）的 LeRobot v3 数据集。转换要求三路 RGB 图像、
`/joint_states` 和四路 `/execution/.../joint_reference`；默认会拒绝有 recorder
drop 或 writer error 的 episode，并用转换 manifest 防止重复导入。

```bash
# 单条 episode
pixi run -e lerobot lerobot-convert -- \
  --episode data/episodes/pick_bread/episode_000001 \
  --task pick_bread

# 批量转换
pixi run -e lerobot lerobot-convert -- \
  --all --task pick_bread
```

给出 `--task <task>` 时，默认从 `data/episodes/<task>` 读取，写到
`~/lerobot_train/<task>`，并使用 `<task>` 作为 `repo_id`。任一默认值都可用同名
参数显式覆盖；不要把不同 feature schema 或任务写进同一个输出目录。

## 7. ACT / SmolVLA 部署入口

### ACT 真实部署

```bash
pixi run -e lerobot act-piper -- \
  --checkpoint /home/alpha/lerobot_train/outputs/piper_act2 \
  --task pick_bread \
  --real
```

默认以 30 Hz 持续推理，不录制 episode；`Ctrl-C` 会释放 Policy session。终端按
`T` 可切换至 Leader teleop；不需要该功能时添加
`--no-teleop-takeover --no-teleop-hotkey`。
