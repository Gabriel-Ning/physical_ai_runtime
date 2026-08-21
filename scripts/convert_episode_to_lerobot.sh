#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
lerobot_python="${LEROBOT_PYTHON:-/home/alpha/lerobot_train/.pixi/envs/default/bin/python}"
ros_site_packages="$workspace_root/.pixi/envs/default/lib/python3.12/site-packages"
ros_lib="$workspace_root/.pixi/envs/default/lib"

if [[ ! -x "$lerobot_python" ]]; then
    echo "找不到 LeRobot Python: $lerobot_python" >&2
    echo "请设置 LEROBOT_PYTHON=/path/to/lerobot/.pixi/envs/default/bin/python" >&2
    exit 1
fi

# pixi activation prepends the workspace's NumPy/Torch site-packages.  Keep
# only the project interface path here; the converter itself appends ROS
# modules after LeRobot's own dependency set so Torch and torchvision stay
# version-matched.
export PYTHONPATH="$workspace_root/src/interfaces/rmi"
export ROS_SITE_PACKAGES="$ros_site_packages"
export LD_LIBRARY_PATH="$ros_lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$lerobot_python" "$workspace_root/scripts/convert_episode_to_lerobot.py" "$@"
