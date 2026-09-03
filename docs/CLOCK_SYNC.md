# 跨机对时（工作站 ↔ RT）

脚本：[`scripts/sync_clock`](../scripts/sync_clock)。

| 机器 | 作用 | 例子 |
|---|---|---|
| 工作站 / host | 跑 marker / `new_apps`，给局域网供 NTP | `192.168.1.18` |
| RT | 跑 `ros2_control` / EM，跟着工作站的钟 | `delta@192.168.1.101` |

看 **chrony offset**。不要看 SSH `date` 的 skew。

目标：**|offset| ≤ 100 us**（GOOD）。≤ 500 us 还能用；> 2 ms 先别开跨机 demo。

命令按终端标。把用户 / IP 换成你的。

---

## 对时

`--setup` / `--status` 只打在 **host terminal**。脚本自己 SSH 进 RT，不要先 `ssh` 进去再跑这两条。

```bash
# host terminal（仓库根目录）
ssh delta@192.168.1.101
exit

scripts/sync_clock --setup  delta@192.168.1.101
scripts/sync_clock --status delta@192.168.1.101
```

`--setup` 可能问密码：

- host `sudo`：本机 NTP 还没开时
- RT `sudo`：脚本 SSH 上去之后会停在 host terminal 等你输入

等到脚本自己结束。`verdict: GOOD` 再跑跨机节点。下次先 `--status`，不是 GOOD / OK 再 `--setup`。

---

## 拆开两步（等价）

host 先供时，再 SSH 进 RT 让它 follow。

```bash
# host terminal（仓库根目录）
sudo scripts/sync_clock --serve --local-ip 192.168.1.18
ssh delta@192.168.1.101
```

```bash
# ssh rt host terminal（已经 ssh 进去之后）
sudo scripts/sync_clock --ip 192.168.1.18 --follow
```

等到打印 `Converged (|offset| <= 100 us)`。

不要在 `ssh rt host terminal` 里跑 `--setup` / `--status`。

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
