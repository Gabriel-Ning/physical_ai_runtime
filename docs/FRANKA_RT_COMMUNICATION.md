# Franka RT host: FCI communication constraints

Franka’s Factory Communication Interface (FCI) runs a **1 kHz** UDP loop on a
dedicated NIC. If the control PC misses the timing window, libfranka aborts with:

```text
communication_constraints_violation
```

This is a **host realtime / networking** failure, not an RMI or application bug.
It showed up more often on beta after stacking cameras + DDS on the same RT PC
while GRUB isolation lagged the profile (`isolcpus=14,15` vs profile `12-15`).

Pika gripper USB traffic is **not** as strict; we do **not** isolate gripper USB
IRQs. Protect the Franka NIC and the control threads first.

## What must stay true on beta

| Role | Interface / process | CPUs |
| --- | --- | --- |
| General OS, CycloneDDS, D405, fisheye | `enp90s0` (`192.168.1.100`), USB cameras | `0-11` |
| Franka FCI NIC IRQs only | `enp2s0` (`192.168.2.100` → robot `192.168.2.101`) | `12-13` |
| `ros2_control` (Franka arm + Pika gripper HW) | `taskset` via `RT_CM_CPU_AFFINITY` | `14-15` |

Kernel cmdline must match the profile:

```text
isolcpus=12-15 nohz_full=12-15 rcu_nocbs=12-15
```

NIC separation is mandatory: **never** put CycloneDDS or workstation peers on
`enp2s0`. Keep ROS on `enp90s0` (or another non-FCI NIC).

## Scripts (canonical copies in this repo)

| File | Purpose |
| --- | --- |
| [`scripts/rt_cpu_profile.franka_beta.env`](../scripts/rt_cpu_profile.franka_beta.env) | Franka beta CPU / NIC profile |
| [`scripts/apply_rt_isolcpus.sh`](../scripts/apply_rt_isolcpus.sh) | Write/replace GRUB `isolcpus` (`--apply --replace`) |
| [`scripts/apply_franka_rt_networking.sh`](../scripts/apply_franka_rt_networking.sh) | Pin FCI / ROS NIC IRQs + `ethtool` coalesce; install boot service |
| [`scripts/apply_franka_rt_host.sh`](../scripts/apply_franka_rt_host.sh) | One-shot: activate Franka profile + GRUB + networking service |
| [`scripts/pai-franka-rt-networking.service`](../scripts/pai-franka-rt-networking.service) | systemd unit template (installed by networking script) |

On the Franka RT host:

```bash
cd ~/Documents/Git_space/physical_ai_runtime   # or your clone path
sudo bash scripts/apply_franka_rt_host.sh
sudo reboot
```

After reboot:

```bash
cat /sys/devices/system/cpu/isolated          # 12-15
bash scripts/apply_franka_rt_networking.sh --status
# enp2s0 IRQs -> 12-13 ; enp90s0 IRQs -> 0-11
cat /sys/devices/system/cpu/cpu14/cpufreq/scaling_governor   # performance
```

Then start the stack (`cpu` Pixi env). `ros2_control_node` should be pinned to
`14,15` via `RT_CM_CPU_AFFINITY` from the profile / `.envrc`.

## Why this layout

1. **FCI packets and softirqs** must not share cores with DDS image floods or
   camera USB load.
2. **Control threads** (`SCHED_FIFO`) stay on `14-15`; Franka NIC IRQs use
   neighboring isolated cores `12-13` so hard IRQ / softirq do not preempt the
   1 kHz loop on the same core.
3. Cameras (D405 + fisheye) stay on non-isolated CPUs. Gripper shares
   `ros2_control` but is low-rate serial — no dedicated USB IRQ island.

## Related symptoms (different causes)

| Symptom | Typical cause |
| --- | --- |
| `communication_constraints_violation` | Host jitter / IRQ / isolcpus mismatch / FCI NIC shared with DDS |
| `joint_motion_generator_*_discontinuity` | Command jumps / missing filtering — not NIC isolation |
| EM stays `FAULT` after RT restart | Workstation EM needs explicit clear/preempt — not FCI |

## Related docs

- General CPU RT host setup: [`CPU_HOST_SETUP.md`](CPU_HOST_SETUP.md)
- Franka bringup package: [`../src/bringup/franka_manipulation/rt_launch/README.md`](../src/bringup/franka_manipulation/rt_launch/README.md)
- Pika udev names: [`UDEV_HOST_SETUP.md`](UDEV_HOST_SETUP.md)

> Workstation clones keep Marvin defaults in `scripts/rt_cpu_profile.env`.
> Franka beta settings live in `scripts/rt_cpu_profile.franka_beta.env` and are
> installed onto the RT host by `apply_franka_rt_host.sh`.
