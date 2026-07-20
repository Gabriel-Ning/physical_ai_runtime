# rviz_interactive_marker_teleop

RViz interactive-marker teleop **source**: drag a 6-DoF marker and publish
`geometry_msgs/PoseStamped` targets. ROS 2 Jazzy.

This package does **not** own robots, controllers, RViz configs, or safety
limits. Apps compose those around it.

## Role

```text
RViz InteractiveMarker
  -> target_marker_node
  -> PoseStamped (configurable topic)
  -> execution manager / planner / controller
```

Default output topic matches the EM pose contract:

`/action_sources/marker/pose_target`

## Launch

```bash
ros2 launch rviz_interactive_marker_teleop teleop.launch.py
```

Common overrides:

```bash
ros2 launch rviz_interactive_marker_teleop teleop.launch.py \
  pose_topic:=/action_sources/marker/pose_target \
  base_frame:=base_link \
  target_frame:=flange_link \
  publish_frequency:=50.0
```

RViz must already be running with InteractiveMarkers enabled, and TF must
provide `base_frame` → `target_frame` so the marker can initialise at the
current EE pose.

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `pose_topic` | `/action_sources/marker/pose_target` | Output `PoseStamped` |
| `publish_frequency` | `50.0` | Publish rate (Hz) |
| `base_frame` | `base_link` | Marker / pose frame |
| `target_frame` | `flange_link` | TF frame used to seed the marker |
| `server_namespace` | `target_marker` | `InteractiveMarkerServer` namespace |
| `marker_name` | `target_marker` | Marker name |

## Dependencies

`rclpy`, `geometry_msgs`, `interactive_markers`, `visualization_msgs`, `tf2_ros`

## License

Apache-2.0
