# Motion Planning Backend Review Notes & Physical Test Guide

> Planner algorithms and their numerical validation remain owned by the motion
> planning packages. Runtime ownership, provider arbitration, controller
> selection and dispatch are supplied by RMI through the contracts in
> `MOTION_PLANNER_SOURCE_INTERFACE.md`.

本文档记录了针对 `pyroki_planner_adapter` 和 `ndcurves_planner_adapter` 后端架构走查发现的技术细节、算法假设及优化建议。用于在后续实机测试中评估与选择是否实施改进。

---

## 1. ndcurves Cartesian Streamer Adapter (`ndcurves_planner_adapter`)

### 1.1 弧长离散采样点数 (`arc_length_discretization_count`)
* **配置项**：已在 `NdcurvesCartesianPlanConfig` 中暴露参数 `arc_length_discretization_count: int = 64`（替换原硬编码 256）。
* **测试说明**：实测 64 点即可达到亚毫米级弧长精度。若在极轻量级 CPU 上运行，可尝试调低至 32 点；若存在复杂 S 弯多途经点轨迹，可调高至 128 点。

### 1.2 速度/加速度限幅归一化假设 (`sqrt(3)` 缩放)
* **实现位置**：`motion_planner_core/time_parameterizer.py`
* **机制**：
  ```python
  norm_scale = math.sqrt(3.0)
  velocity_limits = np.asarray([max_linear_velocity / norm_scale] * 3 + [max_angular_velocity / norm_scale] * 3)
  ```
* **影响与改进点**：
  * 为了防止 3D 向量各轴分量合成时模长超过设定的 `max_linear_velocity`，当前代码将单轴速度限制直接除以了 $\sqrt{3} \approx 1.732$。
  * **现象**：当机械臂进行纯单轴直线运动时，实际移动速度只有设定值的 $\approx 57.7\%$（速度保守变慢约 42.3%）。
  * **实机测试建议**：若实机测试中发现 Cartesian Streamer 运动过于缓慢，可将 TOPP-RA 约束重构为对 1D 路径切向速度 $\dot{s}$ 的标量限幅（$|\dot{s}| \le v_{\text{max}}$）。

### 1.3 途经点 (Via-points) 姿态插值限制
* **机制**：在 `_make_exact_cubic_path` 中，平移采用 `nc.exact_cubic` 穿过所有 `via_xyz` 途经点，姿态采用 `nc.SO3Linear` 在起点四元数与终点四元数之间进行 SLERP 线性插值。
* **限制说明**：途经点目前仅支持 3D 位置 `via_xyz`。若实机场景需要在途经点处指定精确的末端姿态，需引入基于 $SO(3)$ 的多段/样条姿态插值器（如 Squad 或分段 SLERP）。

### 1.4 TOPP-RA 高频计算缓存 (Caching)
* **实测性能**：在 50Hz ($dt=20\text{ms}$) 周期下，`step()` 内部包含探针计算与 TOPP-RA 二次规划，热启动平均耗时约 **2.76 ~ 5.52 ms**。
* **实机测试建议**：目前实测耗时在 20ms 帧预算之内。但若实机测试中 CPU 负载较高导致丢帧，可增加缓存逻辑：只有当目标 `CartesianState` 或途经点改变时才重算 TOPP-RA 轨迹，静止或稳态跟踪时仅根据时间偏移量直接采样。

---

## 2. PyRoki Planner Adapter (`pyroki_planner_adapter`)

### 2.1 J-PARSE Setpoint Backend (参考 Exp 14: `14_singularity_aware_ik.py`)
* **核心类**：`PyrokiIkResolverBackend` / `_jparse.py`
* **测试记录点**：
  1. **JAX-Host 频繁数据同步**：`resolve()` 在 Python `for` 循环中最多进行 200 次 J-PARSE 单步迭代，每次迭代产生一次 Python ↔ JAX 设备同步。若实际测试发现 IK 端到端延时偏高，建议将 200 次迭代使用 `jax.lax.while_loop` 整体移入 JAX 内部编译。
  2. **关节极限 Clip**：`jparse_step` 每次迭代使用 `np.clip(cfg + delta)` 进行硬限幅。大步长（如 `max_step_rad=0.05`）下，需在测试中观察关节极限与奇异点附近的收敛平滑度。
  3. **未收敛结果策略**：当前 `require_convergence` 默认为 `False`，有限但未达到位姿阈值的结果会保留为 best-effort 输出。先保持现状，实机测试时记录收敛率、最终位姿误差和耗时，再决定是否将默认值改为 `True`。

### 2.2 TrajOpt Trajectory Backend (参考 Exp 7: `07_trajopt.py`)
* **核心类**：`PyrokiTrajectoryPlannerBackend` / `_trajopt.py`
* **测试记录点**：
  1. **两阶段求解分支跳变 (Branch Jump)**：当前实现先调用 `solve_ik_target` 求解终点关节角 $q_{\text{end}}$，再调用 `solve_trajopt` 进行轨迹优化。若终点 IK 选中了不同的关节解分支（如肘部向上 vs 肘部向下），TrajOpt 优化可能会因无法平滑穿越而失败。实机测试中需重点测试**大角度跨分支目标**。
  2. **步数下限**：`solve_trajopt` 包含 5 点加速度 Cost，要求 `timesteps >= 5`。实机/接口调用时需确保 `timesteps >= 5`。

### 2.3 Horizon MPC Streamer Backend (参考 Exp 6: `06_online_planning.py`)
* **核心类**：`PyrokiHorizonMpcBackend` / `_online_planning.py`
* **测试记录点**：
  1. **Receding Horizon 热启动移位**：当前滑动窗口直接保留上一帧的解 `prev_sols`。标准 MPC 滚动优化在推进 $dt$ 后，上一帧的第 0 步已执行，传入下一个周期前应做 **1-step 移位**（即 `np.concatenate([prev_sols[1:], prev_sols[-1:]], axis=0)`）。测试高频在线追踪时需观察机器人首步加速度是否有跳变。
  2. **目标类型限制**：PyRoki `solve_online_planning` 目前原生仅支持 `CartesianState` 跟踪，若传入 `JointState` 会返回拒绝，实测试验时需注意。

### 2.4 Warmup 异常策略
* PyRoki trajectory/MPC 与 cuRobo MPC 的 warmup 当前会吞掉编译或初始化异常，以允许后续首次真实调用再次尝试。
* 先保持该行为；真机测试时记录 warmup 耗时、首次调用额外延迟及失败类型，再决定 warmup 应 fail-fast 还是只记录 warning。
