# mjpeg_cam

Reusable V4L2 publisher for Pika / Sunplus DECXIN fisheye cameras (and any
UVC device that streams MJPEG). Used by Marvin and Franka RT camera bringup.

Publishes **original JPEG packets** as `sensor_msgs/CompressedImage`. It does
**not** depend on [`image_transport_plugins`](https://github.com/ros-perception/image_transport_plugins):
those plugins take `sensor_msgs/Image` and re-encode. Decode happens on the
workstation consumer.

## Topics

Default (node name `camera`, namespace set by launch):

- `image/compressed` (`sensor_msgs/CompressedImage`, `format=jpeg`, SensorData QoS)

Stamp is `now()` at USB dequeue. Header `frame_id` comes from parameters.

## Parameters

Template (also the launch default): `config/mjpeg_cam.yaml`.

| Name | Default | Meaning |
|---|---|---|
| `video_device` | `/dev/video0` | V4L2 device |
| `frame_id` | `camera_optical_frame` | `header.frame_id` |
| `image_width` / `image_height` | `1280` / `720` | Requested MJPEG size |
| `framerate` | `30.0` | Requested FPS |
| `compressed_topic` | `image/compressed` | Relative topic |
| `format` | `jpeg` | `CompressedImage.format` |
| `camera_name` | `""` | Optional label; unused by the node |

## Launch

```bash
ros2 launch mjpeg_cam mjpeg_cam.launch.py \
  namespace:=left_pika_fisheye \
  video_device:=/dev/fisheye0 \
  frame_id:=left_pika_fisheye_link
```

| Argument | Default | Meaning |
|---|---|---|
| `params_file` | `share/mjpeg_cam/config/mjpeg_cam.yaml` | Parameter YAML |
| `namespace` | empty | Node namespace |
| `name` | `camera` | Node name |
| `video_device` / `frame_id` / `camera_name` / `compressed_topic` | empty | Override YAML when set |

Robot bringup should copy the template into a model YAML (see Marvin
`config/camera/pika_fisheye.yaml`) and pass that as `params_file`.

## Run

```bash
ros2 run mjpeg_cam mjpeg_cam_node --ros-args \
  -r __ns:=/left_pika_fisheye \
  --params-file $(ros2 pkg prefix mjpeg_cam)/share/mjpeg_cam/config/mjpeg_cam.yaml \
  -p video_device:=/dev/fisheye0 \
  -p frame_id:=left_pika_fisheye_link
```
