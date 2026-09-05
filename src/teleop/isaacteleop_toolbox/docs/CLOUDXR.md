# CloudXR setup and networking

## Setup

Prepare workspace-owned assets once (needs network):

```bash
# Physical AI Runtime: CLOUDXR_DIR comes from pixi [activation.env]
ros2 run isaacteleop_toolbox isaacteleop-cloudxr-setup

# Elsewhere:
export CLOUDXR_DIR=/path/to/ws/.cloudxr
ros2 run isaacteleop_toolbox isaacteleop-cloudxr-setup
# or:
ros2 run isaacteleop_toolbox isaacteleop-cloudxr-setup \
  --cloudxr-dir /path/to/ws/.cloudxr
```

This downloads the IsaacTeleop web client and pinned WebXR controller profiles
(1.0.20), patches client URLs to `/client/`, and validates the offline set.
Runtime launch never downloads assets and never kills processes holding ports;
missing assets or port conflicts fail explicitly.

`$CLOUDXR_DIR` must be workspace-owned. It never defaults to `$HOME`.

## Connecting the Quest

On a normal single-LAN PC:

1. Start the source launch on the PC.
2. In the Quest browser open:

   ```text
   https://<pc-lan-ip>:48322/client/
   ```

3. Accept the certificate / continue into the client.

You usually **do not** put the PC IP into ROS YAML or launch arguments.
IsaacTeleop auto-detects the LAN address used in headset URLs.

## When to edit `cloudxr-env-config.env`

Default file: `$CLOUDXR_DIR/cloudxr-env-config.env` (created empty by setup).

Override only when auto-detection is wrong, for example multi-NIC, port
collision, container, NAT, or USB-local mode.

| Variable | Meaning | Default |
|---|---|---|
| `PROXY_PORT` | WSS proxy + host-client TLS port | `48322` |
| `TELEOP_STREAM_SERVER_IP` | Streaming IP reported to the client | auto LAN IP |
| `TELEOP_PROXY_HOST` | Host used in headset URLs | auto LAN IP |
| `TELEOP_STREAM_PORT` | Signaling port advertised to the client | proxy port |
| `USB_UI_PORT` | Loopback UI in USB-local mode | `8080` |
| `USB_BACKEND_PORT` | CloudXR backend for USB reverse mapping | `49100` |
| `USB_TURN_PORT` | TURN in USB-local mode | `3478` |

Example multi-NIC pin:

```bash
PROXY_PORT=49322
TELEOP_STREAM_SERVER_IP=192.168.50.10
TELEOP_PROXY_HOST=192.168.50.10
TELEOP_STREAM_PORT=49322
```

```bash
ros2 launch isaacteleop_toolbox bimanual_target_live.launch.py \
  cloudxr_env_config:=/absolute/path/to/cloudxr-env-config.env
```

Firewall: baseline upstream paths include UDP `47998` and TCP `49100` /
`48322` (or your replacements).

## Configuration split

- Retargeting / ROS topics → parameter YAML  
- Process / network → CloudXR env file  

Do not duplicate upstream env keys as ROS parameters unless a stable
`CloudXRLauncher` API can forward them.
