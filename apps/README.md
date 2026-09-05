# Physical AI Runtime Applications Suite (`apps/`)

Production-ready, profile-driven CLI applications built on top of the **RMI Python SDK**.

All applications can be invoked directly with `pixi run <app>` or `python apps/<app>.py`.

---

## 1. `pixi run teleop` — Interactive Robot Teleoperation

Thin profile-driven client. **Does not** start RT or teleoperators.

Startup order:

1. RT launch
2. Workstation launch (teleoperators already running; `use_sim_time` is decided here)
3. `pixi run teleop --profile <profile.yaml>`

```bash
# Franka gamepad / Quest (device clutch on workstation):
pixi run teleop --profile fr3_pika_single_arm.yaml

# Piper leaders (no hardware clutch — keyboard Enter toggles preempt + relay):
pixi run teleop --profile piper_bimanual.yaml
```

### Behavior by profile

| Profile style | Who streams / preempts | This app |
|---------------|------------------------|----------|
| No `teleoperators` / no `preempt_service` (Franka gamepad, Marvin Quest) | Workstation + device clutch | Ready check + clutch tip; optional `is_active` status |
| `teleoperators` with `preempt_service` (Piper leaders) | App relays leader topics after Enter | Keyboard preempt + JointTrajectory relay |

Clock (`use_sim_time`) is owned by workstation launch; this app has no `--use-sim-time` flag.

---

## 2. `pixi run record` — Multi-Modal Episode Dataset Recorder

Interactive demonstration data collection workflow with **Quintic Spline Staging**, **Zero-Drop Preemption**, and **Parallel MCAP Sealing**:

```bash
# Record 10 bimanual demonstrations (default 50 Hz):
pixi run record --profile piper_bimanual.yaml --task bimanual_pickup --episodes 10

# Custom staging duration and task:
pixi run record --task cup_stacking --homing-duration 2.5 --rate-hz 50.0
```

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

## 3. `pixi run replay` — 1:1 Native Trajectory Replayer

Performs strict 1:1 native timestamp-paced trajectory replay directly from recorded MCAP datasets on real or simulated robot embodiments:

```bash
# Replay latest recorded episode at 1:1 native rate:
pixi run replay --profile piper_bimanual.yaml

# Replay specific MCAP episode:
pixi run replay --profile piper_bimanual.yaml \
    --mcap-file data/episodes/piper_bimanual_teleop/episode_000001.mcap
```

---

## 4. `apps/check_runtime.py` — Read-only Runtime Check

Checks joint-state availability and age, hardware diagnostics, camera readiness,
and the current provider allocation map without acquiring control or publishing a
command:

```bash
python apps/check_runtime.py \
    --profile piper_bimanual.yaml \
    --duration 10 \
    --check-cameras
```

## 5. `apps/eval.py` — LeRobot Policy Evaluation

Loads a checkpoint and evaluates it through the standard RMI application loop:

```text
observation = robot[resource].get_observation()
actions = policy.select_action(observation)
policy_node[resource].submit(actions)
```

```bash
python apps/eval.py \
    --profile piper_bimanual.yaml \
    --checkpoint /path/to/pretrained_model \
    --task "pick up the object" \
    --resource dual_manipulator
```

## 6. Embodiment Profiles (`apps/profiles/`)

All applications are fully decoupled and driven by YAML Embodiment Profiles stored in [apps/profiles/](file:///home/gn/Documents/Git_Space/physical_ai_runtime/apps/profiles/):

* `piper_bimanual.yaml` — Dual-arm Piper bimanual setup with master-slave leader teleoperation.
* `fr3_pika_single_arm.yaml` — Single-arm Franka Research 3 with Pika Gripper.
* `marvin_bimanual.yaml` — Marvin humanoid dual-arm bimanual setup.

Recording stream contracts live in `apps/recording/`. Profiles select one of
these contracts, which RMI injects dynamically when the recorder is prepared;
workstation recorder launch files only start the generic daemon.
