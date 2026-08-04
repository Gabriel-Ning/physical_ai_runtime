# CPU / realtime-kernel host setup

Use this guide on a **PREEMPT_RT** control PC that runs Franka / Marvin / Piper
`ros2_control` (no NVIDIA / Isaac Teleop / cuRobo / CloudXR).

Goal after setup:

```text
performance governor (boot + now)
  + realtime group + PAM rtprio/memlock (SCHED_FIFO for ros2_control)
  + isolcpus / nohz_full / rcu_nocbs from scripts/rt_cpu_profile.env
  + ros2_control_node pinned to RT_CM_CPU_AFFINITY (bringup taskset)
```

You may reboot once when isolation is first written to GRUB. That reboot also
refreshes PAM limits and `realtime` group membership.

## Install and setup

```bash
pixi install --locked -e cpu
pixi run -e cpu setup
```

`setup` (cpu env only) runs [`scripts/setup_cpu_rt_host.sh`](../scripts/setup_cpu_rt_host.sh):

1. **Performance** — [`scripts/enable_cpu_performance_governor.sh`](../scripts/enable_cpu_performance_governor.sh)
   (`--ensure-boot`: apply now + systemd unit).
2. **Realtime limits** — [`scripts/ensure_realtime_limits.sh --ensure`](../scripts/ensure_realtime_limits.sh)
   - creates group `realtime`
   - adds the operator user (`$USER` / `$SUDO_USER`)
   - writes `/etc/security/limits.d/99-pai-realtime.conf`
     (`rtprio 99`, `memlock unlimited`)
3. **Isolation** — reads [`scripts/rt_cpu_profile.env`](../scripts/rt_cpu_profile.env)
   (`RT_ISOL_CPUS`, default `14,15`) and ensures the matching GRUB fragment via
   [`scripts/apply_rt_isolcpus.sh --ensure`](../scripts/apply_rt_isolcpus.sh).
4. **Isolated-core clocks** — when isolation is already active, raises
   `scaling_min_freq` on those CPUs to `scaling_max_freq` so idle isolcpus
   do not sit at 800 MHz.
5. Prints governor / isolated / `RT_CM_CPU_AFFINITY` / `ulimit -r` status.

If GRUB or realtime limits were updated (or isolation is configured but not
yet active in this boot), `setup` exits **3**:

```bash
sudo reboot
# after reboot:
pixi run -e cpu setup
ulimit -r   # expect 99
```

Sudo is required the first time for governor install, PAM limits, and GRUB
writes. Limits alone would only need a re-login; when `isolcpus` also changes,
one reboot covers both.

## Activate the cpu env

Prefer Direnv (see root [README](../README.md)). After `pixi run -e cpu setup`,
`.pixi/environment` is `cpu`, and [`.envrc`](../.envrc) sources
`scripts/rt_cpu_profile.env` so `RT_CM_CPU_AFFINITY` is exported in the shell.

```bash
direnv allow
# or: eval "$(pixi shell-hook --frozen -e cpu)"
#     set -a && source scripts/rt_cpu_profile.env && set +a
```

## Verify

```bash
uname -a | grep -i realtime          # PREEMPT_RT
cat /sys/kernel/realtime             # 1
cat /sys/devices/system/cpu/isolated # e.g. 14-15
grep -E 'isolcpus|nohz_full' /proc/cmdline
scripts/enable_cpu_performance_governor.sh --status
scripts/ensure_realtime_limits.sh --status
ulimit -r                            # 99
echo "affinity=$RT_CM_CPU_AFFINITY"   # e.g. 14,15
```

## Launch controllers (no manual taskset)

On a configured cpu host these bringups pin **only** `ros2_control_node`:

| App | Package |
|---|---|
| Franka | `franka_manipulation_controller_bringup` |
| Marvin | `marvin_manipulation_controller_bringup` |
| Piper | `piper_manipulation_controller_bringup` |

```bash
# Franka example
ros2 launch franka_manipulation_controller_bringup \
  controller_bringup.launch.py \
  use_fake_hardware:=false robot_ip:=192.168.2.101

# Piper example (SocketCAN names from udev — see docs/UDEV_HOST_SETUP.md)
ros2 launch piper_manipulation_controller_bringup \
  controller_bringup.launch.py \
  arms:=left use_fake_hardware:=false \
  left_can_interface:=piper0 left_end_effector:=piper_gripper
```

Affinity resolution order (same in each bringup):

1. Launch arg `cpu_affinity:=14,15` (explicit override)
2. Else env `RT_CM_CPU_AFFINITY` (from the cpu RT profile / `.envrc`)
3. `cpu_affinity:=none` disables pinning

Implementation note: launch performs prefix substitutions by concatenation
**without spaces**, then `shlex.split`s the result. Bringups therefore pass a
single string:

```python
prefix=f"taskset -c {cpu_affinity}" if cpu_affinity else None
```

Do **not** use `prefix=["taskset", "-c", cpu_affinity]` — that becomes the
nonexistent executable `taskset-c14,15`.

Confirm after start:

```bash
pgrep -af ros2_control_node
ps -eLo pid,psr,comm,rtprio,policy | awk '/ros2_control/'
```

- `PSR` should be only the affinity CPUs (e.g. 14 and/or 15).
- After realtime limits + re-login/reboot, CM threads should show non-`-`
  `rtprio` and `FF` (SCHED_FIFO), not
  `Could not enable FIFO RT scheduling policy`.

Do **not** wrap the whole `ros2 launch` in `taskset` for camera / Foxglove /
UI processes — keep those off the isolated cores.

## Change the isolated CPU set

Edit [`scripts/rt_cpu_profile.env`](../scripts/rt_cpu_profile.env):

```bash
RT_ISOL_CPUS=12,13,14,15
RT_CM_CPU_AFFINITY=12,13,14,15
```

Then:

```bash
pixi run -e cpu setup   # may rewrite GRUB + re-check limits
sudo reboot
pixi run -e cpu setup
ulimit -r               # expect 99
```

If GRUB already has a different `isolcpus=`, the helper refuses to clobber it;
edit `/etc/default/grub` by hand, `sudo update-grub`, reboot.

## Related

- Profile defaults: [`scripts/rt_cpu_profile.env`](../scripts/rt_cpu_profile.env)
- Realtime PAM limits: [`scripts/ensure_realtime_limits.sh`](../scripts/ensure_realtime_limits.sh)
- CPU RT host orchestrator: [`scripts/setup_cpu_rt_host.sh`](../scripts/setup_cpu_rt_host.sh)
- Host udev / USB-CAN (hardware, not RT): [`UDEV_HOST_SETUP.md`](UDEV_HOST_SETUP.md)
- Cross-host chrony / EM stamp skew: [`CLOCK_SYNC.md`](CLOCK_SYNC.md)
- Colocation evidence (Marvin + Hik on this class of host):
  [COLOCATION_VALIDATION.md](COLOCATION_VALIDATION.md)
- Bringups:
  - [`franka_manipulation_controller_bringup`](../src/apps/franka_manipulation_controller_bringup/README.md)
  - [`marvin_manipulation_controller_bringup`](../src/apps/marvin_manipulation_controller_bringup/README.md)
  - [`piper_manipulation_controller_bringup`](../src/apps/piper_manipulation_controller_bringup/README.md)
