# CPU / realtime-kernel host setup

Use this guide on a **PREEMPT_RT** control PC that runs Franka / Marvin
`ros2_control` (no NVIDIA / Isaac Teleop / cuRobo / CloudXR).

Goal after setup:

```text
performance governor (boot + now)
  + isolcpus / nohz_full / rcu_nocbs from scripts/rt_cpu_profile.env
  + ros2_control_node pinned to RT_CM_CPU_AFFINITY (bringup taskset)
```

You may reboot once when isolation is first written to GRUB.

## Install and setup

```bash
pixi install --locked -e cpu
pixi run -e cpu setup
```

`setup` (cpu env only) runs [`scripts/setup_cpu_rt_host.sh`](../scripts/setup_cpu_rt_host.sh):

1. **Performance** — [`scripts/enable_cpu_performance_governor.sh`](../scripts/enable_cpu_performance_governor.sh)
   (`--ensure-boot`: apply now + systemd unit).
2. **Isolation** — reads [`scripts/rt_cpu_profile.env`](../scripts/rt_cpu_profile.env)
   (`RT_ISOL_CPUS`, default `14,15`) and ensures the matching GRUB fragment via
   [`scripts/apply_rt_isolcpus.sh --ensure`](../scripts/apply_rt_isolcpus.sh).
3. **Isolated-core clocks** — when isolation is already active, raises
   `scaling_min_freq` on those CPUs to `scaling_max_freq` so idle isolcpus
   do not sit at 800 MHz.
4. Prints governor / isolated / `RT_CM_CPU_AFFINITY` status.

If GRUB was updated or isolation is configured but not yet in this boot,
`setup` exits **3** and asks for:

```bash
sudo reboot
# after reboot:
pixi run -e cpu setup
```

Sudo is required the first time for governor install and GRUB writes.

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
echo "affinity=$RT_CM_CPU_AFFINITY"
```

## Launch controllers (no manual taskset)

On a configured cpu host, Franka bringup pins **only** `ros2_control_node`:

```bash
ros2 launch franka_manipulation_controller_bringup \
  controller_bringup.launch.py \
  use_fake_hardware:=false robot_ip:=192.168.2.101
```

Affinity resolution order:

1. Launch arg `cpu_affinity:=14,15` (explicit override)
2. Else env `RT_CM_CPU_AFFINITY` (from the RT profile / `.envrc`)
3. `cpu_affinity:=none` disables pinning

Confirm after start:

```bash
pgrep -a ros2_control_node
ps -eLo pid,psr,comm,rtprio | awk '/ros2_control/'
```

`PSR` should be 14 and/or 15 (or whatever you set in the profile).

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
pixi run -e cpu setup   # may rewrite GRUB
sudo reboot
pixi run -e cpu setup
```

If GRUB already has a different `isolcpus=`, the helper refuses to clobber it;
edit `/etc/default/grub` by hand, `sudo update-grub`, reboot.

## Related

- Profile defaults: [`scripts/rt_cpu_profile.env`](../scripts/rt_cpu_profile.env)
- Colocation evidence (Marvin + Hik on this class of host):
  [COLOCATION_VALIDATION.md](COLOCATION_VALIDATION.md)
- Franka app bringup: [`src/apps/franka_manipulation_controller_bringup/README.md`](../src/apps/franka_manipulation_controller_bringup/README.md)
