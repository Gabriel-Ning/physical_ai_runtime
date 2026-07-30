# Cosmos-DROID 策略接入 physical_ai_runtime 运行说明

本目录用于把 `Cosmos3-Nano-Policy-DROID` 接入 `physical_ai_runtime` 的 VLA policy 接口。

推荐结构是：

```text
Cosmos policy server
  <- WebSocket ->
physical_ai_runtime policy node
  -> /action_sources/policy/joint_chunk
  -> Execution Manager
  -> Franka/libfranka 底层控制
```

不要在这个 ROS policy node 里直接加载 Cosmos 大模型，也不要同时绕过 Execution Manager 去直接控制 Franka。Cosmos 仓库负责模型推理，`physical_ai_runtime` 负责 ROS、Execution Manager 和真机控制。

## 目录职责

```text
src/policy_inference/examples/
  cosmos_droid_policy_example.py
    ROS 可运行入口：订阅 joint、gripper、多相机图像，发布 joint chunk。

  cosmos_droid/
    policy.py
      CosmosDroidChunkPolicy：把 ROS observation 转成 Cosmos server payload，并返回动作 chunk。

    ros_observation.py
      ROS 图像、关节、夹爪状态缓存与格式转换。

    gripper.py
      Cosmos gripper 标量和 Franka gripper 表示之间的转换。
```

## 运行前准备

需要两个终端，分别启用两个环境。

### 终端 A：启动 Cosmos policy server

```bash
cd /home/hanyu/WorkSpace/cosmos
source .venv-cosmos5090/bin/activate

export PYTHONPATH=/home/hanyu/WorkSpace/cosmos:$PYTHONPATH

bash scripts/run_droid_policy_server.sh
```

这个终端负责 GPU 推理。确认 server 监听地址类似：

```text
ws://127.0.0.1:8000/
```
```
cd /home/hanyu/WorkSpace/cosmos

PYTHONPATH=packages/cosmos3 python -m cosmos_framework.scripts.action_policy_server_robolab \
  --checkpoint-path models/Cosmos3-Nano-Policy-DROID \
  --host 0.0.0.0 \
  --port 8000
```
### 终端 B：启动 physical_ai_runtime policy node

```bash
cd /home/hanyu/WorkSpace/physical_ai_runtime
source install/setup.bash

export PYTHONPATH=/home/hanyu/WorkSpace/cosmos:/home/hanyu/WorkSpace/physical_ai_runtime/src:$PYTHONPATH
```

如果当前 ROS 环境缺少 WebSocket client 依赖，在终端 B 对应的 Python 环境里安装：

```bash
python -m pip install websockets msgpack pillow numpy
```

## 最小 dry-run 检查顺序

先不要上真机，按下面顺序检查。

### 1. 检查能否 import Cosmos client

```bash
python - <<'PY'
from tools.franka_cosmos_policy.policy_client import CosmosPolicyClient
from tools.franka_cosmos_policy.observation import FrankaObservation
print("Cosmos client import OK")
PY
```

如果报 `No module named tools`，说明 `PYTHONPATH` 没有包含 Cosmos 仓库根目录：

```bash
export PYTHONPATH=/home/hanyu/WorkSpace/cosmos:$PYTHONPATH
```

### 2. 检查 ROS topic 是否存在
你的 CycloneDDS 配置里要求：

<SocketReceiveBufferSize min="10MiB" max="10MiB" />
<SocketSendBufferSize min="10MiB" max="10MiB" />
也就是要求收发缓冲区至少 10MiB。

但系统当前只有：

net.core.rmem_max = 212992
net.core.wmem_max = 212992
所以 ROS 2 报：

failed to increase socket receive buffer size to at least 10485760 bytes
current is 425984 bytes
10485760 bytes 就是 10MiB。
```
sudo sysctl -w net.core.rmem_max=10485760
sudo sysctl -w net.core.rmem_default=10485760
sudo sysctl -w net.core.wmem_max=10485760
sudo sysctl -w net.core.wmem_default=10485760
```
```bash
ros2 topic list
```

至少需要能拿到：

```text
/joint_states
/wrist_camera/color/image_raw
/exterior_camera_1/color/image_raw
/exterior_camera_2/csudo sysctl -w net.core.rmem_max=10485760
sudo sysctl -w net.core.rmem_default=10485760
sudo sysctl -w net.core.wmem_max=10485760
sudo sysctl -w net.core.wmem_default=10485760olor/image_raw
```

实际 topic 名可以不同，但需要在运行参数里对应填进去。

### 3. 启动 Cosmos-DROID policy node

等 `cosmos_droid_policy_example.py` 实现后，运行形式建议保持为：

```bash
python src/policy_inference/examples/cosmos_droid_policy_example.py --ros-args \
  -p server_url:=ws://127.0.0.1:8000/ \
  -p joint_names:="[fr3_joint1,fr3_joint2,fr3_joint3,fr3_joint4,fr3_joint5,fr3_joint6,fr3_joint7]" \
  -p joint_state_topic:=/franka/joint_states \
  -p camera_topics:="[/cameras/cam_0/image_raw,/cameras/cam_1/image_raw,/cameras/cam_2/image_raw,/cameras/cam_3/image_raw,/cameras/cam_4/image_raw,/cameras/cam_5/image_raw,/cameras/cam_6/image_raw,/cameras/cam_7/image_raw]" \
  -p task:="pick up the object and place it in the bowl" \
  -p target_fps:=15.0 \
  -p horizon:=5 \
  -p action_dt_s:=0.1 \
  -p record_data:=true \
  -p record_output_dir:=data \
  -p record_episode_name:=fr3_cosmos_droid_001
```

实时推理只使用 `camera_topics` 前三路：`cam_0` 作为 wrist、`cam_1/cam_2` 作为两个 exterior view。8 路图像都会按 `target_fps` 降采样保存到 `data/<record_episode_name>/`，60fps 相机输入不会全部落盘。第一次真机前仍建议先用 fake policy / dry-run 验证，再把 `record_data` 和真实 Cosmos server 一起打开。

## 数据格式约定

输入给 Cosmos server 的 observation 应该包含：

```python
{
    "prompt": task,
    "observation/joint_position": joint_position,       # [T,7], float32
    "observation/gripper_position": gripper_position,   # [T,1], float32
    "observation/image": concat_view,                   # [H,W,3], uint8
}
```

其中 `concat_view` 使用 Cosmos-DROID 训练时的布局：

```text
上半部分：wrist_image_left
下半部分：exterior_image_1_left 和 exterior_image_2_left 左右拼接
```

可以直接复用 Cosmos 仓库里的：

```python
from tools.franka_cosmos_policy.observation import FrankaObservation
```

## 动作格式约定

Cosmos-DROID policy server 返回：

```python
action.shape == [T, 8]
```

含义是：

```text
action[:, :7]  -> Franka 7 个 arm joint 的绝对关节位置
action[:, 7]   -> gripper 标量
```

发布到 `/action_sources/policy/joint_chunk` 的 `JointTrajectory` 只能放 arm joints：

```python
arm_action = action[:, :7]
```

gripper 不应该塞进 `JointTrajectory.joint_names`，建议后续单独走 gripper command topic 或 Execution Manager 的夹爪接口。

## 安全建议

第一次真机前请保持：

```text
rate_hz: 1.0 到 2.0
horizon: 3 到 5
每次只执行 action chunk 的前 1 步
关节单步限幅：0.02 到 0.03 rad
先不上夹爪闭合，只观察 arm 是否朝正确方向动
```

如果 Cosmos server 返回 NaN、Inf、维度不对，policy node 必须拒绝发布。

如果图像任一路缺失，policy node 应该保持当前位置，不发布新动作。

## 常见错误

### `No module named tools`

没有把 Cosmos 仓库加入 `PYTHONPATH`：

```bash
export PYTHONPATH=/home/hanyu/WorkSpace/cosmos:$PYTHONPATH
```

### `No module named websockets` 或 `No module named msgpack`

在运行 ROS policy node 的 Python 环境里安装：

```bash
python -m pip install websockets msgpack
```

### 连接不上 `ws://127.0.0.1:8000/`

确认终端 A 的 Cosmos policy server 已启动。若 server 在另一台机器，把 `server_url` 改成对应 IP：

```text
ws://服务器IP:8000/
```

### action 维度不匹配

Cosmos 输出 `[T,8]`，`JointTrajectory` 通常只接受 7 个 arm joint。发布前必须切掉 gripper 维度：

```python
arm_action = action[:, :7]
```

