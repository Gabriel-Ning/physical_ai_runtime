# Camera Driver, Configuration, and Host Provisioning

Status: **Accepted; implementation pending**  
Decision date: 2026-08-01

This document records the ownership and deployment decisions for Hikvision,
RealSense, Orbbec, and generic UVC cameras in Physical AI Runtime.

## Decision summary

Camera integration is split into three independently owned layers:

1. **Driver binaries** live in the Pixi/prefix runtime.
2. **Physical camera and rig configuration** lives in the application that
   composes the camera into a robot or data-collection system.
3. **Host device provisioning** such as udev rules lives under `scripts/` and
   is installed explicitly during workstation setup.

The source tree must not shadow a released driver already present in the
runtime prefix.

## Hikvision release boundary

`hikvision_ros2` has completed substantial real-hardware validation and is a
release candidate. Its reusable ROS packages should be published as binary
packages to the `gabriel-robotics` prefix.dev channel:

- `hik_camera_msg`: ROS messages and services;
- `hik_camera`: driver, generic launch support, diagnostics, and validation
  utilities.

The dependency direction is:

```text
libmvs -----------+
                  +-> hik_camera -> hik_camera_bringup
hik_camera_msg ---+
```

`libmvs` remains an independently versioned Pixi dependency. Site composition
does not become part of the driver binary: `hik_camera_bringup` continues to
own camera serial identities, the eight-camera PTP profile, launch composition,
and optional Foxglove integration.

Publishing requires more than successful hardware testing. Before declaring a
release complete, verify:

- clean, reproducible package builds with explicit versions and release tags;
- runtime linkage to `libmvs` without development-machine absolute paths;
- installation and launch from the prefix with the source checkout absent;
- `validate_only`, single-camera, and validated multi-camera acceptance paths;
- a pinned `pixi.toml` dependency for the published packages.

After the binary is consumed through Pixi, `hikvision_ros2` must not remain in
the runtime workspace overlay. A source checkout would shadow the released
prefix package and make deployment non-reproducible.

## Configuration ownership

RealSense, Orbbec, and generic UVC cameras use upstream ROS 2 drivers supplied
by Pixi/Robostack:

- `realsense2_camera`;
- `orbbec_camera`;
- `usb_cam`.

The current `realsense_camera_config`, `orbbec_camera_config`, and
`usb_cam_camera_config` packages contain no driver implementation. They are
configuration references, and some values are already tied to a particular
device or application. The long-term ownership rule is:

| Configuration | Owner |
|---|---|
| Driver executable and vendor defaults | Pixi/prefix driver package |
| Generic, hardware-independent example | Optional documentation/template |
| Serial number, USB port, or stable `/dev` symlink | Application |
| Namespace, camera name, and frame prefix | Application |
| Resolution, FPS, depth alignment, point-cloud policy | Application |
| Robot mounting transform and calibrated extrinsics | Application/robot bringup |
| Device-managed intrinsics | Camera driver at runtime |

Concrete camera YAML therefore belongs under the consumer, for example:

```text
src/apps/<application>/config/cameras/
```

A reusable configuration package is justified only when a genuinely generic
profile is shared by multiple applications. Such a profile must not contain a
physical serial number, workstation USB topology, application namespace, or
robot-specific frame name. If there is only one consumer, keeping a separate
`*_camera_config` ROS package adds indirection without creating reuse and the
configuration should move into the application.

## udev and host provisioning

udev rules are host configuration, not ROS parameters and not application
runtime data. Orbbec permissions, stable camera symlinks, USB-CAN naming, and
similar rules should be consolidated under:

```text
scripts/
  udev/
    rules.d/
      <workspace-owned rules>
    check.sh
    install.sh
    README.md
```

The provisioning contract is:

- `rules.d/` is the version-controlled source of truth;
- `check.sh` is non-privileged and reports missing or differing installed
  rules;
- `install.sh` performs an explicit, idempotent privileged installation with
  mode `0644`, reloads the udev rule database, and tells the operator whether
  devices must be replugged;
- ordinary build and launch tasks never write `/etc/udev/rules.d`;
- workspace setup checks the rules by default but installs them only through
  an explicit operator action such as an `--install-udev` option.

Rules must be reviewed before centralization. Prefer a controlled device group
or `TAG+="uaccess"` over globally writable `MODE:="0666"` unless the vendor
requires and justifies broader access.

## CAN boundary

Only stable USB-CAN device identification and access permissions belong in
udev. SocketCAN interface configuration is a separate host concern:

```text
scripts/
  can/
    check_socketcan.sh
    setup_socketcan.sh
```

Bitrate, interface activation, restart behavior, and persistent network setup
must be handled by the CAN setup scripts or the selected host network service,
not by udev rules. Applications may select an interface such as `piper0`, but
must not own the host mechanism that creates and configures it.

## Migration sequence

1. ~~Add the centralized udev check/install mechanism and migrate existing
   Orbbec, RealSense, and USB-CAN rules into it.~~ Done:
   `scripts/udev/install.sh` plus `rules.d/` (`99-can-piper*`,
   `99-obsensor-libusb`, `99-realsense-libusb`). Operator doc:
   [`UDEV_HOST_SETUP.md`](UDEV_HOST_SETUP.md). (`check.sh` still optional.)
2. Move physical RealSense, Orbbec, and UVC configurations into their consumer
   applications; retain only proven cross-application templates.
3. Package and publish `hik_camera_msg` and `hik_camera`, validate a
   prefix-only deployment, then remove the source overlay from the runtime
   workspace.
