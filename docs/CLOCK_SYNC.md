# 跨机对时（工作站 ↔ RT）

只有一个脚本：[`scripts/sync_clock`](../scripts/sync_clock)。

两台机器：

| 你站在哪 | 机器 | 例子 |
|---|---|---|
| **工作站** | 跑 marker / `new_apps` 的桌面机 | `192.168.1.13` |
| **RT 主机** | `rt_user@rt_ip`，跑 `ros2_control` / EM | `delta@192.168.1.101` |

看 **chrony offset**。不要看 SSH `date` 的 skew（半个 RTT，抖的是毫秒）。

目标：**|offset| ≤ 100 us**（GOOD）。≤ 500 us 还能用；> 2 ms 先别开跨机 demo。

---

## 人在工作站前

仓库根目录。RT 主机不用登录。

```bash
# 1. SSH 先通
ssh rt_user@rt_ip

# 2. 同步：本机开 LAN NTP，RT 跟着本机
scripts/sync_clock --setup rt_user@rt_ip
```

等待脚本自己结束。它会一直等到 chrony **|offset| ≤ 100 us**，或超时 120 s。

```bash
# 3. 再看一次状态
scripts/sync_clock --status rt_user@rt_ip
```

`verdict: GOOD`（≤ 100 us）之后再跑跨机节点，例如 marker / `new_apps`。

固定主机例子：

```bash
scripts/sync_clock --setup  delta@192.168.1.101
scripts/sync_clock --status delta@192.168.1.101
```

`--setup` 会问两次密码：工作站 `sudo`（开 NTP），以及 RT 的 sudo（SSH 里输一次）。

下次开 demo：先 `--status`。不是 GOOD / OK 再 `--setup`。

---

## 人在 RT 主机上

不要在 RT 上跑 `--setup` / `--status`（那是工作站命令，会再 SSH 出去）。

工作站必须已经在给局域网供时。若还没有，先让人在工作站执行一次：

```bash
# 在工作站上
sudo scripts/sync_clock --serve --local-ip 192.168.1.13
```

然后**在 RT 主机**、仓库根目录，让本机去跟远程工作站：

```bash
# 192.168.1.13 换成工作站在机器人网上的 IP
sudo scripts/sync_clock --ip 192.168.1.13 --follow
```

等待打印 `Converged (|offset| <= 100 us)`。

---

## 精度

| 方法 | 稳态钟差 |
|---|---|
| chrony / 局域网 NTP（本脚本） | 几十～几百微秒，门限 100 us |
| PTP + 网卡硬件打戳 | 亚微秒～几微秒（还没上） |

钟齐了只表示墙钟一致。`header.stamp` 仍要打在规划起点；控制器用 `now - stamp`，不要用 `points[0]`。

## Related

- Realtime / isolcpus → [`CPU_HOST_SETUP.md`](CPU_HOST_SETUP.md)
- Host udev / CAN → [`UDEV_HOST_SETUP.md`](UDEV_HOST_SETUP.md)
