# Physical AI Runtime Applications Suite (`apps/`)

基于 RMI Python SDK 的 profile-driven 应用。当前抓角实验主链路已冻结为：

- 录制：奥比中光顶部相机 + 左腕 + 右腕；
- 模型输入：`top + left_wrist + right_wrist`；
- 部署：只允许奥比中光作为 `top`；
- D435i1、D435i2 不再参与新数据录制、转换、训练或部署。

`default` / `runtime` 环境用于 ROS 2 bringup、录制和真机运行；`lerobot`
环境用于数据转换、训练、checkpoint 加载与 GPU 推理。

## 1. 遥操作

```bash
cd /home/alpha/physical_ai_runtime
pixi run -e runtime python apps/teleop.py
```

## 2. 录制 MCAP

正常的 `piper_bimanual.yaml` profile 和 recorder contract 只要求以下三路图像：

| 数据字段 | ROS topic |
|---|---|
| 顶部（奥比中光） | `/observation/orbbec/color/image_raw` |
| 左腕 | `/observation/left_hand_realsense/color/image_raw` |
| 右腕 | `/observation/right_hand_realsense/color/image_raw` |

```bash
cd /home/alpha/physical_ai_runtime

pixi run -e runtime python apps/record.py \
  --profile piper_bimanual.yaml \
  --task pick_corner \
  --episodes 100
```

`record.py` 会在启动录制前等待三路相机首帧，并在录制中监控断流。任一路缺帧或
停流都会阻止保存该 episode。即使 ROS 图中仍存在 D435 topic，录制程序也不会订阅
或校验它们。

`--task` 同时决定 episode 的任务标签和目录，例如上面的数据写入
`data/episodes/pick_corner/episode_*`。任务名不能包含 `/`、`\\`，也不能是
`.` 或 `..`。

每条 episode 完成后：

- `S` 或回车：校验 checksum、完整读取 MCAP、核对三路相机计数并保存；
- `D`：丢弃并重录当前编号；
- `R`：回放检查；
- `Q`：结束采集。

## 3. 回放与只读检查

```bash
pixi run -e runtime python apps/replay.py \
  --profile piper_bimanual.yaml \
  --mcap-file data/episodes/pick_corner/episode_000001/episode_000001.mcap

pixi run -e runtime python apps/eval.py \
  --profile piper_bimanual.yaml --duration 10 --check-cameras
```

真机回放前必须确认安全区、急停和机械臂初始姿态。

## 4. MCAP 转 LeRobot v3

新数据统一使用 `orbbec` 视角。转换器从 MCAP 读取三路图像、`/joint_states`
和四路 `/execution/.../joint_reference`，同步到 30 Hz，并将奥比中光中心裁剪为
4:3 后缩放到 640×480。输出 feature 固定为：

- `observation.images.top`
- `observation.images.left_wrist`
- `observation.images.right_wrist`
- `observation.state`
- `action`

批量转换：

```bash
cd /home/alpha/physical_ai_runtime

pixi run -e lerobot lerobot-convert -- \
  --all \
  --task pick_corner \
  --camera-view orbbec \
  --output /home/alpha/lerobot_train/pick_corner_orbbec \
  --repo-id pick_corner \
  --fps 30 \
  --require-accepted-demonstration
```

转换单条 episode 时将 `--all` 换成：

```text
--episode data/episodes/pick_corner/episode_000001
```

转换会先校验 SHA-256、MCAP EOF、recorder health sidecar，以及奥比中光和两路腕部
的实际/metadata/written 帧数。不要在转换尚未自然结束时重跑同一命令或启动训练。
转换 CLI 只开放 `--camera-view orbbec`，不会接受旧五相机参数。

## 5. ACT chunk 数量实验

`chunk_size` 不属于录制格式；同一份 LeRobot 数据可以训练不同 chunk 的 ACT。
训练命令只在 `/home/alpha/lerobot_train/README.md` 中维护；本项目不复制训练命令。
该文档采用显式写出具体 chunk 值的方式，例如 50 对应
`policy.chunk_size=50`、`policy.n_action_steps=50`，输出目录后缀为 `_50`。

- `policy.chunk_size`：每次预测的动作序列长度；
- `policy.n_action_steps`：训练配置中每次执行的动作数，不能大于 `chunk_size`；
- 部署参数 `--action-steps M`：只执行 chunk 前 M 步，范围为 `1..chunk_size`。

为了判断训练 chunk 本身的影响，第一轮部署令 `--action-steps` 等于该 checkpoint
的 `chunk_size`。如果后续单独研究重规划频率，再固定 checkpoint，仅扫描更小的
`--action-steps`。

## 6. ACT 三相机部署（单顶部视角）

这里的 `--camera-view single` 表示只使用一个固定顶部视角，不表示模型总共只输入
一台相机。ACT checkpoint 必须包含 `top + left_wrist + right_wrist` 三个图像 feature。

`CHECKPOINT` 不是程序内置变量，需要在每个新终端中显式设置。它应指向包含
`config.json` 的 `pretrained_model` 目录；下面的路径和 `020000` 请替换为实际训练输出。

先做无 ROS、无运动的 checkpoint dry-run：

```bash
cd /home/alpha/physical_ai_runtime

export CHECKPOINT=/home/alpha/lerobot_train/outputs/pick_corner1_50/checkpoints/020000/pretrained_model

test -f "${CHECKPOINT}/config.json"

pixi run -e lerobot act-piper -- \
  --checkpoint "${CHECKPOINT}" \
  --camera-view single \
  --action-steps 50 \
  --device cuda \
  --dry-run
```

dry-run 通过后，再做真机评估：

```bash
cd /home/alpha/physical_ai_runtime

export CHECKPOINT=/home/alpha/lerobot_train/outputs/pick_corner_orbbec_50/checkpoints/020000/pretrained_model

test -f "${CHECKPOINT}/config.json"

LD_LIBRARY_PATH=/home/alpha/physical_ai_runtime/.pixi/envs/runtime/lib \
pixi run -e lerobot act-piper -- \
  --checkpoint "${CHECKPOINT}" \
  --profile apps/profiles/piper_bimanual.yaml \
  --task pick_corner_orbbec_50_eval10 \
  --camera-view single \
  --action-steps 50 \
  --record-episodes 10 \
  --layout-ids L5,L4,L3,L2,L1,L1,L2,L3,L4,L5 \
  --max-input-age-s 1.0 \
  --real
```

部署程序只接受 `/observation/orbbec/color/image_raw` 作为顶部相机，并始终同时使用
左右腕部。`--action-steps` 不能超过 checkpoint 的 `chunk_size`，否则启动时直接失败。

录制评估时：准备阶段回车开始；运行阶段回车提前结束、`T` 切换 ACT/Leader 接管、
`Q` 丢弃并退出；复核阶段回车/`S` 保存、`D` 删除、`Q` 退出。切换 checkpoint 前
先按 `Ctrl-C`，等待 Policy lease 完全释放；不要同时运行两个 `act-piper`。

## 7. SmolVLA 三相机部署

SmolVLA 部署固定使用 `top + left_wrist + right_wrist` 三个图像 feature。与 ACT
不同，`--task` 是模型的语言条件，必须与训练数据中的任务文本完全一致；
pick_corner_smo           → --task pick_corner
pick_corner_smo_finetune  → --task pick_corner_hil

```bash
cd /home/alpha/physical_ai_runtime

export CHECKPOINT=/home/alpha/lerobot_train/outputs/pick_corner_smo/checkpoints/050000/pretrained_model

test -f "${CHECKPOINT}/config.json"

LD_LIBRARY_PATH=/home/alpha/physical_ai_runtime/.pixi/envs/runtime/lib \
pixi run -e lerobot smolvla-piper -- \
  --checkpoint "${CHECKPOINT}" \
  --profile apps/profiles/piper_bimanual.yaml \
  --task pick_corner \
  --device cuda \
  --rate-hz 30 \
  --max-input-age-s 1.0 \
  --real
```

该入口固定订阅以下三路图像：

- 顶部：`/observation/orbbec/color/image_raw`
- 左腕：`/observation/left_hand_realsense/color/image_raw`
- 右腕：`/observation/right_hand_realsense/color/image_raw`

SmolVLA 不使用 ACT 的 `--camera-view` 和 `--action-steps` 参数；执行步数由 checkpoint
中的 `n_action_steps` 决定，当前为 50。运行时按 `T` 可切换 Leader 接管；退出时先按
`Ctrl-C`，等待 Policy lease 完全释放。

## 8. Diffusion 三相机部署

Diffusion 使用与 ACT、SmolVLA 相同的三路相机和 14 维状态/动作约定。当前
`020000` checkpoint 配置为 `n_obs_steps=2`、`horizon=64`、`n_action_steps=32`；
部署程序会在动作执行期间保留连续观测，供下一次 Diffusion 推理使用。

先做无 ROS、无运动的 checkpoint dry-run：

```bash
cd /home/alpha/physical_ai_runtime

export CHECKPOINT=/home/alpha/lerobot_train/outputs/pick_corner_diffusion/checkpoints/020000/pretrained_model

test -f "${CHECKPOINT}/config.json"

pixi run -e lerobot diffusion-piper -- \
  --checkpoint "${CHECKPOINT}" \
  --device cuda \
  --dry-run
```

当前 checkpoint 的预期结果是 `action shape=(32, 14), finite=True`。dry-run 通过，
并且 RT 双臂与三路相机已经启动后，再运行真机部署：

```bash
cd /home/alpha/physical_ai_runtime

export CHECKPOINT=/home/alpha/lerobot_train/outputs/pick_corner_diffusion/checkpoints/020000/pretrained_model

test -f "${CHECKPOINT}/config.json"

LD_LIBRARY_PATH=/home/alpha/physical_ai_runtime/.pixi/envs/runtime/lib \
pixi run -e lerobot diffusion-piper -- \
  --checkpoint "${CHECKPOINT}" \
  --profile apps/profiles/piper_bimanual.yaml \
  --task pick_corner \
  --device cuda \
  --rate-hz 30 \
  --action-steps 32 \
  --max-input-age-s 1.0 \
  --real
```

`--action-steps` 可在 `1..32` 内调小，以提高重新规划频率，但第一次真机验证应保持
checkpoint 的训练值 32。该命令会通过 RMI 向左右机械臂和夹爪发送
`joint_reference`；运行前必须确认安全区、急停、初始姿态和三路相机均正常。
运行时按 `T` 可切换 Leader 接管，退出时按 `Ctrl-C` 并等待 Policy lease 释放。

## 9. Profile

当前双臂主流程使用 `apps/profiles/piper_bimanual.yaml`。五相机 profile 和旧 D435
recorder contract 仅作为历史实验文件保留，不应再用于新一轮录制、训练或部署。
