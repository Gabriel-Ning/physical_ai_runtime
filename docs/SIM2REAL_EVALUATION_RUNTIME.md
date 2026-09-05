# Sim2Real 仿真评测与执行运行时架构及长期演进路线

状态：长期演进指南与架构共识  
版本：v1.0  
更新日期：2026-09-03  

---

## 1. 核心定位与系统使命 (Core Positioning & Mission)

### 1.1 核心定位
**`physical_ai_runtime` (PAI) 的核心定位是“真机等价（Sim-to-Real Equivalence）”的物理智能仿真评测与策略执行运行时。**

### 1.2 解决的行业核心痛点
在传统的具身智能与机器人学习研究（如基于纯 Python Gym / Robosuite / 原生 LIBERO）中，通常存在严重的**“工程链路断层”**：
- 仿真中，策略直接通过 Python 同步调用 `env.step(action)`，直接穿透写入仿真器的执行器变量，并在同一个线程中阻塞拷贝渲染图像；
- 部署到真机时，工程师不得不推倒重写一套基于 ROS 2 / C++ 的实时控制、通信与驱动链路。

这种断层导致部署失败时，往往无法界定是**算法泛化问题**，还是**通信丢包、控制频率不匹配、插补逻辑差异、驱动器延迟等工程机制引入的伪 Gap**。

### 1.3 核心设计原则
1. **工程机制 100% 同构**：通信拓扑、底层控制器（如 1kHz 力矩回路 TKJIC / JSIC）、安全限幅守卫与生命周期状态机在仿真与真机上完全一致。
2. **Gap 唯一性收敛**：**最终存在的 Sim2Real Gap，应严格且仅仅体现在“数据分布”（物理参数分布与视觉感官分布）上，彻底消除通信、时钟与控制逻辑上的工程 Gap。**

---

## 2. 硬件抽象与两端同构架构 (Hardware Abstraction & Homomorphism)

运行时通过 ROS 2 原生 `ros2_control` 实现硬件抽象层（HAL）解耦：

```text
       +-------------------------------------------------------------+
       |                  Policy / VLA / Evaluator                   |
       |  (LeRobot / OpenVLA / RMI / Python / ros2 action/topic)     |
       +-------------------------------------------------------------+
                                     │  (完全同构的 ROS 2 话题与动作接口)
                                     ▼
       +-------------------------------------------------------------+
       |             ros2_control Manager & Controllers              |
       |     (1kHz TKJIC/JSIC 力矩闭环、JTC 轨迹控制、jtc_guard_node)  |
       +-------------------------------------------------------------+
                                     │
           ┌─────────────────────────┴─────────────────────────┐
           ▼ (Sim 仿真后端)                                     ▼ (Real 真机后端)
   MujocoSystemInterface                             Franka / Piper FCI HardwareInterface
   (MuJoCo Physics, 1kHz implicitfast)               (libfranka 1kHz FCI, CANopen)
   (Sensors, Cameras, TF, FreeJointState)            (真实 RealSense 相机, 电机编码器)
```

- **控制接口对齐**：Franka 在仿真中由原生 `<motor>`（力矩接口）驱动，并开启 `gravcomp="1"` 重力补偿，底层由 TKJIC / JSIC 输出 1kHz 目标力矩，在物理机制上严密对齐 Franka FCI 的实时力矩输入规范；
- **安全与拦截同构**：速度/加速度/力矩限幅守卫节点（如 `jtc_guard_node`）在仿真与真机中同拓扑挂载；
- **Task 与 Control 解耦**：任务层（`BaseTask`、`randomize_scene`）属于纯业务评测逻辑，绝不耦合 `controller_manager`。

---

## 3. 对标 NVIDIA Isaac ROS 的演进启示

对比 NVIDIA 在 Isaac ROS 与 Isaac Sim / Isaac Lab (Omniverse) 上的工业实践，本项目确立以下关键演进借鉴点：

| 维度 | NVIDIA Isaac 体系 | physical_ai_runtime 实践方案 | 长期规划与启示 |
| :--- | :--- | :--- | :--- |
| **软件在环 (SIL)** | 真机与仿真跑同一套容器镜像，节点图 100% 同构 | ROS 2 控制器栈与上层 Policy 节点图同构 | 持续保持节点名称、QoS 策略与话题契约无缝复用 |
| **高通量零拷贝传输** | 提出 **NITROS**（基于 CUDA-IPC / 共享内存）避免序列化开销 | 目前基于 ROS 2 标准 IPC / FastDDS 传输 | 后续在多相机（双腕+头顶）高分辨率场景，引入基于共享内存的零拷贝传输，消除序列化延迟 |
| **物理与渲染解耦** | PhysX 5 (GPU 动力学) 与 Omniverse RTX (光追渲染) 强解耦 | MuJoCo (1kHz 物理求解) 与视觉渲染管线解耦 | 物理步进与图像渲染异步运行，避免渲染等待拉低物理控制回路频率 |

---

## 4. 时钟与端到端延迟模拟注入体系 (Timing & Latency Modeling)

在真实机器人部署中，策略面对的永远是“滞后观测（Delayed Observation）”。为了防止策略在仿真中依赖“零延迟”的虚假完美环境，系统规划在三层进行显式延迟与抖动注入：

```text
 [ MuJoCo 物理时钟源 (1kHz, use_sim_time) ]
         │
         ├──► 1. 传感器观测延迟层 (Sensor Ring Buffer Plugin)
         │       └─ 图像与位姿进入环形缓冲，按 ΔT_sensor ~ N(μ, σ) 滞后发布 (如 30~50ms)
         │
         ├──► 2. 策略异步计算耗时 (Asynchronous Policy Loop)
         │       └─ 策略按真机算力耗时独立步进，基于历史滞后帧推理 Action Chunk
         │
         └──► 3. 动力学执行器延迟层 (MujocoSystemInterface Latency FIFO)
                 └─ 接收力矩指令后延迟 1~2 个 tick (1~2ms) 写进 d->ctrl，模拟驱动器时间常数
```

### 注入技术规范：
1. **传感器环形缓冲区 (Ring Buffer)**：
   在相机发布节点内维护时序队列，确保策略接收到的观测时间戳 $t_{obs} = t_{now} - \Delta t_{latency}$，并支持高斯抖动（Jitter）注入；
2. **异步非阻塞调用 (Asynchronous Execution)**：
   严禁在策略执行环路中使用同步单步阻塞 `env.step()`，推动策略演进为与真机一致的异步订阅-推理-分块执行（Action Chunking）流。

---

## 5. 渲染视觉鸿沟与 3D Gaussian Splatting (3DGS) 长期演进

### 5.1 痛点背景
MuJoCo 自带的 OpenGL 栅格化渲染器材质塑料感重、光影粗糙，是导致视觉策略（OpenVLA / Octo / Diffusion Policy）Sim2Real 泛化受阻的最大瓶颈。

### 5.2 3DGS 神经渲染解耦接入路线
借鉴前沿学术成果（*SplatSim*, *PhysGaussian*, *ManiGaussian*），规划在未来将渲染底座升级为 3D Gaussian Splatting：

```text
              ┌──────────────────────────────────────────────┐
              │           MuJoCo C Engine (1kHz)             │
              │   (负责刚体动力学解算: 刚体位姿 T_world_obj(t)) │
              └──────────────────────┬───────────────────────┘
                                     │ 导出 6-DoF 刚体位姿
                                     ▼
              ┌──────────────────────────────────────────────┐
              │          3DGS Neural Renderer Node           │
              │      (基于 gsplat / CUDA 高性能栅格化)       │
              │  - 加载真实工作台与物体的预扫描 3DGS 点云 PLY  │
              │  - 随刚体位姿刚性变换高斯基元中心与协方差     │
              │  - 实时渲染逼近真实物理相机视角的照片级图像   │
              └──────────────────────┬───────────────────────┘
                                     │ 30 FPS 照片级图像流
                                     ▼
                        /camera/wrist/image_raw
                        /camera/head/image_raw
                                     ▼
                         [ Policy / VLA 决策网络 ]
```

- **平滑兼容性**：
  由于本项目已经建立了标准的 ROS 2 图像话题发布体系与 TF/FreeJointState 状态流，升级到 3DGS 时**无需修改任何任务 Spec、策略接口或控制层**，仅作为渲染插件透明热插拔。

---

## 6. 仿真数据分布的闭环与黑盒原则 (Data Distribution Closure)

所有数据分布的扰动与随机化，**必须完全且只能在仿真内部（Simulation Boundary）闭环完成**：

1. **初始状态分布 $p(s_0)$（Task Layer 负责）**：
   - 物体坐标扰动（$\Delta x, \Delta y$）、自旋角偏航（$\Delta \text{yaw}$）、抓取目标重排；
   - 由各 Task 的 `randomize_scene()` 纯语义化产出。
2. **动力学分布 $p(s_{t+1}|s_t, a_t)$（Physics Engine 负责）**：
   - 物体摩擦系数 $\mu \in [0.4, 1.2]$、质量 $m \pm 15\%$、阻尼与关节间隙扰动；
   - 在仿真加载和重置时动态注入。
3. **感知观测分布 $p(o_t|s_t)$（Render Pipeline 负责）**：
   - 相机安装外参的物理微小误差（如 $\pm 2\text{mm}$、$\pm 1^\circ$ 的装配扰动）；
   - 环境灯光漫反射衰减、光照色温扰动、相机曝光与感光噪点。

### 策略黑盒原则 (Black-Box Rule)
**策略网络（Policy）在任何时候都绝不能接收到来自仿真器的特权真值（Privileged State）或域随机化参数。** 策略只能接触到标准传感器观测，从而迫使其学习到对物理扰动与视觉变化具有内在不变性（Invariance）的稳健表征。

---

## 7. 统一分层运行与评测工作流 (Scheme B Unified Architecture Workflow)

为了解决真机与仿真在启动链路上的割裂，确立标准化三层运行体系：

```text
┌──────────────────────────────────────────────────────────────────────────────────┐
│ [终端 1] RT 底座层 (硬件/仿真动力学)                                                │
│   - 真机运行:  pixi run rt-franka backend:=real                                  │
│   - 仿真运行:  pixi run rt-franka backend:=mujoco task:=<name> headless:=false   │
│   * 职责: 启动物理/仿真计算、ros2_control_node、1kHz 控制器回路、安全守卫与直连相机     │
└────────────────────────────────────────┬─────────────────────────────────────────┘
                                         │
┌────────────────────────────────────────▼─────────────────────────────────────────┐
│ [终端 2] Workstation 中枢层 (多源调度、抢占与统一录制)                              │
│   - 统一运行:  pixi run workstation-franka [use_sim_time:=true|false]            │
│   * 职责: 启动 Execution Manager (EM) 多源仲裁、GamepadTeleop (抢占)、Recorder (录制)│
└────────────────────────────────────────┬─────────────────────────────────────────┘
                                         │
┌────────────────────────────────────────▼─────────────────────────────────────────┐
│ [终端 3] Task & Policy 评测终端 (纯 Python API 驱动，非 Launch 强绑定)              │
│   - 统一运行:  pixi run eval-task --task <task_name> [--episodes 10]              │
│   * 职责: 通过 RMI Context 连接 Robot 与 EM，执行 Task Spec (reset/step/check)   │
│          统计 Episode 成功率与耗时，解耦复杂的 ROS 2 launch 生命周期                │
└──────────────────────────────────────────────────────────────────────────────────┘
```

### 启动时序与就绪依赖契约：
1. **时序顺序**：终端 1 (RT 底座) $\rightarrow$ 终端 2 (Workstation 调度中枢) $\rightarrow$ 终端 3 (Task & Policy 评测)。
2. **容错与重连**：Workstation 栈设计上具备对 RT 底座的话题自适应发现；Task Evaluator API 通过 `context.wait_until_ready(require_execution_manager=True)` 确保只有在 EM 和机器人底盘完全就绪后才进入评测循环，彻底消除了由于进程并发启动带来的竞态条件。

---

## 8. 两阶段具身策略训练与微调范式 (Two-Stage Embodied Policy Paradigm)

针对物理智能系统从算法研发到实体落地的全周期，Runtime 确立以下两阶段范式：

```text
       ┌─────────────────────────────────────────────────────────────┐
       │ 阶段一: 大规模并行预训练 (Massive Cloud / GPU Pre-training)   │
       │   - 底座引擎: mjlab (MuJoCo Warp GPU) / 世界模型 (World Models) │
       │   - 核心任务: 4096+ 环境并行学习，获取基础物理先验、时空因果律与通用技能│
       └──────────────────────────────┬──────────────────────────────┘
                                      │ 输出通用策略底座 (Base Policy Checkpoint)
                                      ▼
       ┌─────────────────────────────────────────────────────────────┐
       │ 阶段二: PAI 运行时高保真微调与部署 (High-Fidelity Finetune on PAI)│
       │   - 底座引擎: physical_ai_runtime (1kHz 控制同构 + EM 调度)  │
       │   - 核心范式: 真机 Online RL / DAgger + Sim 作为安全缓冲与数据增强│
       └─────────────────────────────────────────────────────────────┘
```

### 8.1 Sim Backend 的核心定位：“安全缓冲”与“真机域数据增强”
在阶段二中，尽管直接在真机上运行 DAgger 或 RLPD 进行在线微调是最纯粹的路径，但 **PAI 的 MuJoCo 仿真后端在这里承担不可替代的双重核心使命**：
1. **安全防护缓冲层 (Safety Buffer)**：在策略探索阶段，充当前哨试错缓冲区，杜绝因未成熟探索动作导致的真机碰撞、关节过载与硬件损毁；
2. **真机域数据增强引擎 (Domain Data Augmentation Engine)**：真机示教和采集数据成本极高，仿真后端可在毫秒级批量合成不同光照衰减、相机外参微扰、初始位姿抖动的极端工况样本（Edge Cases），作为真机训练数据的有效补充。

---

## 9. Real2Sim 闭环与 MuGS (MuJoCo Gaussian Splatting) 神经渲染

既然仿真后端作为真机数据的增强与安全缓冲，**就必须做足够精准的 Real-to-Sim 重建，把视觉与接触 Gap 彻底闭合**。

### 9.1 为什么选择 MuGS 路线？
对比笨重的游戏引擎（如 Unreal Engine 5），**[MuGS (Renforce-Dynamics/MuGS)](https://github.com/Renforce-Dynamics/MuGS)** 是轻量化、Python/CUDA 原生契合 PAI Runtime 的终极方案：
- **真机照片一键转资产 (Scan-to-Sim)**：使用单反或手机对真实 Franka 工作台与物品环绕扫描（COLMAP + 3DGS），将包含微观划痕、真实漫反射与环境光吸收的高斯点云直接注入仿真；
- **动力学与神经渲染解耦**：MuJoCo 专心解算接触动力学（刚体变换 $T_{world \to obj}$），MuGS 的 CUDA 光栅化核函数负责在 100+ FPS 下生成逼近物理相机的画面；
- **闭合 Real2Sim 循环**：使仿真生成的增强图像在特征分布上与真机相机拍摄数据无缝对齐。

---

## 10. 终极系统架构全景图 (The Ultimate Physical AI Runtime Vision)

```text
 ┌──────────────────────────────────────────────────────────────────────────────────────────────┐
 │                              阶段一: 大规模预训练数据底座 (Cloud / GPU)                           │
 │     [ mjlab (MuJoCo Warp GPU) 并行加速 ]      [ 世界模型 (World Models) / 跨实体基础模型 ]      │
 └──────────────────────────────────────────────┬───────────────────────────────────────────────┘
                                                │ Base Policy Checkpoint
                                                ▼
 ┌──────────────────────────────────────────────────────────────────────────────────────────────┐
 │                         阶段二: physical_ai_runtime (Sim2Real 统一执行运行时)                  │
 │                                                                                              │
 │  ┌────────────────────────────────────────────────────────────────────────────────────────┐  │
 │  │ 任务与调度层 (Task & Evaluation Layer)                                                  │  │
 │  │  - 纯 Python Task Spec (LIBERO / RoboTwin: reset, randomize_scene, check_success)      │  │
 │  │  - crisp_gym 风格的标准 Gymnasium 接口 (支持 DAgger, RLPD, PPO 在线微调)                 │  │
 │  │  - Execution Manager (EM): 多源动作仲裁、Teleop 离合抢占、统一 Dataset Recorder 录制      │  │
 │  └───────────────────────────────────────────┬────────────────────────────────────────────┘  │
 │                                              │                                               │
 │                     ┌────────────────────────┴────────────────────────┐                      │
 │                     ▼                                                 ▼                      │
 │  ┌────────────────────────────────────────┐     ┌─────────────────────────────────────────┐  │
 │  │ 仿真后端 (Sim Backend - 缓冲与数据增强)  │     │ 真机后端 (Real Robot Backend)           │  │
 │  │  - 物理底座: MuJoCo 1kHz (implicitfast)│     │  - 物理底座: Franka FR3 1kHz FCI        │  │
 │  │  - 渲染底座: MuGS (3DGS 照片级高斯光栅化)│     │  - 传感底座: RealSense D405 + 工业相机  │  │
 │  │  - 角色: 强化学习探索缓冲、数据增强引擎│     │  - 角色: 最终策略执行与真机在线微调     │  │
 │  └────────────────────────────────────────┘     └─────────────────────────────────────────┘  │
 │                     ▲                                                 ▲                      │
 │                     └─────────────────────────┬───────────────────────┘                      │
 │                                               │ 完全同构的 ROS 2 话题、控制回路与安全守卫      │
 └───────────────────────────────────────────────┼──────────────────────────────────────────────┘
                                                 │
                                                 ▼ (WebRTC 画面流 / WebSockets 调度 API)
 ┌──────────────────────────────────────────────────────────────────────────────────────────────┐
 │                           方案 C 终极形态: Frontend Web Studio 云控台                         │
 │     - 交互式三维可视化面板，支持 130+ 任务一键无缝热切换                                     │
 │     - 实时查看策略执行热力图、力矩曲线与延迟监控，一键触发人工接管示教                       │
 └──────────────────────────────────────────────────────────────────────────────────────────────┘
```


