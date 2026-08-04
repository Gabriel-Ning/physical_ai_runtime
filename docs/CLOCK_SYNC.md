# Cross-host clock sync (workstation ↔ RT PC)

Cross-machine ROS (marker / leader / demos on a workstation, bringup + EM on
an RT host) rejects stamps when wall-clock skew exceeds EM `max_future_s`
(often **0.1 s**). Typical symptom:

```text
future pose stamp rejected: NNN ms ahead
```

Public NTP on some RT hosts resolves to unreachable **fake-ip** (`198.18.0.x`),
so chrony never reaches `System clock synchronized: yes`. Prefer a **LAN NTP**
peer: the workstation serves time; the RT host follows it until offset
converges (you will see `System time` shrink, e.g. `0.227s` → `0.027s` → ms).

Scripts:

| Script | Where to run | Role |
|---|---|---|
| [`scripts/sync_clock`](../scripts/sync_clock) | **RT host** (delta) | One-shot or `--follow` sync from a peer IP |
| [`scripts/sync_remote_clock.sh`](../scripts/sync_remote_clock.sh) | **Workstation** | Measure skew / optional push via SSH |

## 1. Workstation: allow LAN NTP (once)

On the desk PC that already has a good clock (`timedatectl` →
`System clock synchronized: yes`), e.g. `192.168.1.13`:

```bash
echo 'allow 192.168.1.0/24' | sudo tee /etc/chrony/conf.d/allow-lan.conf
sudo systemctl restart chrony
ss -ulnp | grep ':123'   # expect chronyd listening on UDP/123
```

Adjust the subnet if your robot LAN is not `192.168.1.0/24`.

## 2. RT host: follow until converged

On the RT PC (e.g. delta), with the workstation IP:

```bash
cd ~/Documents/Git_space/physical_ai_runtime   # or your checkout path

# Self-converging chrony client (recommended)
sudo scripts/sync_clock --ip 192.168.1.13 --follow
```

`--follow` writes `/etc/chrony/sources.d/pai-peer.sources` with
`server <ip> iburst prefer`, restarts chrony, runs `makestep`, and prints
per-second `chronyc tracking` until `Leap status: Normal` /
`NTPSynchronized=yes`. Chrony **keeps running** against that peer.

One-shot only (then disables NTP on the RT host):

```bash
sudo scripts/sync_clock --ip 192.168.1.13
```

Paste fallback if NTP is unavailable:

```bash
# On workstation:
date -u +'%Y-%m-%d %H:%M:%S'
# On RT host immediately:
sudo scripts/sync_clock --utc '2026-08-04 03:25:10'
```

## 3. Verify from the workstation

```bash
scripts/sync_remote_clock.sh --status
```

Target: **|skew| ≤ 50 ms** (comfortably under EM `max_future_s`).  
`HIGH` means cross-host teleop / marker stamps may still be rejected.

## Related

- Realtime / isolcpus → [`CPU_HOST_SETUP.md`](CPU_HOST_SETUP.md)
- Host udev / CAN → [`UDEV_HOST_SETUP.md`](UDEV_HOST_SETUP.md)
