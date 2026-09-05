# execution_manager_interfaces

ROS 2 Jazzy message, service, and action definitions used by
[`execution_manager`](https://github.com/Gabriel-Ning/execution_manager).

The package defines authority leases, fenced streaming commands, leased arm and
parallel-gripper actions, and replayable arm/gripper execution traces. It has no
runtime node and is intended to be released before packages that consume a new
interface version.

## Install

```toml
[dependencies]
ros-jazzy-execution-manager-interfaces = "==0.2.0"
```

## Build from source

```bash
colcon build --packages-select execution_manager_interfaces \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## License

Apache-2.0
