# Dependencies

## What this package owns

`isaacteleop_toolbox` is a normal `ament_python` package. Its `package.xml`
declares only ROS / system Python deps that rosdep can resolve, for example:

- `rclpy`, `geometry_msgs`, `std_msgs`, `tf2_ros`, `launch_ros`, `rviz2`
- `python3-numpy`, `python3-scipy`

It does **not** install IsaacTeleop, CloudXR, CUDA, or PyTorch through
setuptools or rosdep.

## What the consuming environment must own

| Dependency | Why it is external |
|---|---|
| IsaacTeleop wheel (validated: 1.3.131) | ABI-sensitive; NVIDIA index / manylinux tags |
| CloudXR runtime + host-client assets | Proprietary; downloaded into `$CLOUDXR_DIR` |
| Matching Python 3.12 + numpy/scipy/OpenCV | Must stay ABI-compatible with ROS / IsaacTeleop |
| Optional: torch (if your retargeter needs it) | Same CUDA stack as the rest of the workspace |

This package imports IsaacTeleop at runtime (`isaacteleop.*`). If that import
fails, the node fails at startup — by design.

## Recommended: Physical AI Runtime

[Physical AI Runtime](https://github.com/Gabriel-Ning/physical_ai_runtime) pins
IsaacTeleop in its single Pixi environment (`pixi.toml` / `pixi.lock`) and sets:

```toml
[activation.env]
CLOUDXR_DIR = "$PIXI_PROJECT_ROOT/.cloudxr"
```

That is the only fully validated integration path today.

## Standalone / other workspaces

You *can* clone this package into any ROS 2 Jazzy workspace, but **you** must
reproduce a compatible Python environment before `colcon build` / launch:

1. Python 3.12 with ROS Jazzy (`rclpy`, etc.).
2. Install the **same** IsaacTeleop wheel the runtime pins, for example:

   ```bash
   pip install \
     https://pypi.nvidia.com/isaacteleop/isaacteleop-1.3.131-cp312-cp312-manylinux_2_35_x86_64.whl
   ```

   Prefer conda/Pixi over ad-hoc `pip` when possible. Do not pip-install
   `numpy` / `opencv-python` on top of a RoboStack ROS prefix.
3. Provide IsaacTeleop's CloudXR extras (host-client deps such as `mcap`,
   retargeting stack if you use those code paths). Mirror the versions used by
   Physical AI Runtime when something breaks.
4. Export a workspace-owned CloudXR dir, then run setup:

   ```bash
   export CLOUDXR_DIR="${PWD}/.cloudxr"
   source install/setup.bash
   ros2 run isaacteleop_toolbox isaacteleop-cloudxr-setup
   ```

### What standalone does *not* get for free

- A solved Pixi lockfile for IsaacTeleop + CUDA + ROS.
- Guaranteed manylinux / glibc compatibility on older hosts
  (IsaacTeleop 1.3.131 needs `manylinux_2_35` / glibc ≥ 2.35).
- Support for mixing random torch / IsaacTeleop / OpenCV builds.

If you maintain your own workspace, treat Physical AI Runtime's `pixi.toml`
IsaacTeleop / teleop PyPI block as the reference pin list, and bump this
package only after you re-validate that environment.

## Why not declare IsaacTeleop in `package.xml` / `install_requires`?

1. It is not on default PyPI/rosdep indexes as a portable ROS dependency.
2. The wheel URL / CUDA / manylinux constraints belong to the **environment
   lock**, not to every ROS package that imports it.
3. Declaring it in setuptools would encourage `pip install` into a ROS prefix
   and break ABI (numpy/OpenCV/torch) for many users.

Optional future: a documented `extras_require` that only points at the pinned
wheel URL for advanced users — still not a substitute for a locked runtime.
