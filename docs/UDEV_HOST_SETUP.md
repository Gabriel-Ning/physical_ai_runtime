# Host udev setup (hardware devices)

Host device naming and access for Physical AI Runtime. This is **hardware
provisioning**, not realtime-kernel tuning.

- Realtime / isolcpus / SCHED_FIFO → [`CPU_HOST_SETUP.md`](CPU_HOST_SETUP.md)
- Cross-host chrony / marker stamp skew → [`CLOCK_SYNC.md`](CLOCK_SYNC.md)
- USB-CAN names, future camera permissions, similar rules → **this doc**

udev rules are host configuration, not ROS parameters and not application
runtime data. Ordinary `pixi run setup` / build / launch never writes
`/etc/udev/rules.d/`.

## Layout

```text
scripts/udev/
  install.sh          # privileged install of all rules.d/*.rules
  rules.d/
    *.rules           # version-controlled source of truth
  README.md           # short pointer; details live here
```

## Install

On each robot / RT desk PC that needs these devices:

```bash
sudo bash scripts/udev/install.sh
```

That copies every `scripts/udev/rules.d/*.rules` into `/etc/udev/rules.d/`
(mode `0644`) and runs `udevadm control --reload-rules`. If a device name did
not change yet, unplug/replug it or reboot.

## Current rules

| File | Purpose |
|---|---|
| `99-can-piper0.rules` | gs_usb serial → **`piper0`** (left Piper follower), 1 Mbps, `txqueuelen=1000` |
| `99-can-piper1.rules` | gs_usb serial → **`piper1`** (right Piper follower), 1 Mbps, `txqueuelen=1000` |
| `99-obsensor-libusb.rules` | Orbbec (obsensor) USB permissions / vendor symlinks |
| `99-realsense-libusb.rules` | Intel RealSense USB permissions |
| `99-pika-fisheye.rules` | Pika UVC fisheye → **`fisheye0`/`fisheye1`**. **Site-specific (Marvin gamma RT host USB `3-9.1` left / `3-7.1` right); expect to change after rewiring.** Single-cam rule is commented. |

Serials in the CAN rules are site-specific identities of the USB-CAN dongles
used with this workspace. Applications select `piper0` / `piper1` by name; they
do not own the host mechanism that creates those names. Camera rules grant
device access for the Pixi/prefix drivers; physical serials and namespaces stay
in application config.

`99-pika-fisheye.rules` currently pins Marvin **gamma** RT-host USB ports. Treat
it as provisional: after rewiring, update the rule **and**
`marvin_manipulation_controller_bringup/config/hardware/marvin_host.yaml` /
`config/camera/marvin_cameras.yaml` together.

### Piper bringup example (after udev)

```bash
ros2 launch piper_manipulation_controller_bringup \
  controller_bringup.launch.py \
  arms:=left use_fake_hardware:=false \
  left_can_interface:=piper0 left_end_effector:=piper_gripper
```

## Adding rules later

Drop another `*.rules` under `scripts/udev/rules.d/` (cameras, permissions,
symlinks, …). The same `install.sh` installs all of them. Prefer stable USB
attributes (`idVendor` / `idProduct` / `serial`) over USB port topology.

## Related

- Installer: [`scripts/udev/install.sh`](../scripts/udev/install.sh)
- Rule sources: [`scripts/udev/rules.d/`](../scripts/udev/rules.d/)
- Camera packaging vs host udev boundary: [`CAMERA_DEPLOYMENT.md`](CAMERA_DEPLOYMENT.md)
- RT kernel host (separate concern): [`CPU_HOST_SETUP.md`](CPU_HOST_SETUP.md)
