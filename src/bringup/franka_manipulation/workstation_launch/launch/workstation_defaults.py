"""Resolve unique config defaults for workstation launch files.

Editable embodiment profiles live only under ``apps/profiles/``. Launch files
must load defaults from that tree (or the install copy built from it), never
embed site/business values as Python literals.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory


def apps_profile_path(profile_name: str) -> Path:
    """Return the unique embodiment profile path for ``profile_name``.

    Prefers the repository ``apps/profiles`` tree so edits stay single-sourced.
    Falls back to this package's installed ``share/.../profiles`` copy for
    deployed workspaces that no longer contain the source tree.
    """
    name = profile_name
    if not name.endswith(".yaml"):
        name = f"{name}.yaml"

    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "apps" / "profiles" / name
        if candidate.is_file():
            return candidate

    installed_candidates = (
        "piper_manipulation_workstation_launch",
        "franka_manipulation_workstation_launch",
        "marvin_manipulation_workstation_launch",
    )
    for package_name in installed_candidates:
        try:
            installed = (
                Path(get_package_share_directory(package_name)) / "profiles" / name
            )
        except Exception:
            continue
        if installed.is_file():
            return installed
    raise FileNotFoundError(
        f"embodiment profile {name!r} not found under apps/profiles or "
        "installed workstation package share"
    )


def load_yaml(path: Path | str) -> dict[str, Any]:
    with Path(path).open(encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise TypeError(f"expected a mapping in {path}")
    return data


def _as_launch_bool(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    text = str(value).strip().lower()
    if text in {"true", "1", "yes", "false", "0", "no"}:
        return "true" if text in {"true", "1", "yes"} else "false"
    raise ValueError(f"expected boolean launch value, got {value!r}")


def workstation_launch_defaults(profile_name: str) -> dict[str, str]:
    """Load workstation launch defaults from the unique embodiment profile."""
    profile_path = apps_profile_path(profile_name)
    profile = load_yaml(profile_path)

    host_roles = profile.get("host_roles") or {}
    rt_bringup = ((host_roles.get("rt_host") or {}).get("bringup") or {})
    ws_bringup = ((host_roles.get("workstation_host") or {}).get("bringup") or {})
    args = dict(ws_bringup.get("arguments") or {})
    recorder = dict(profile.get("recorder") or {})
    metadata = dict(profile.get("metadata") or {})

    rt_package = rt_bringup.get("package")
    if not rt_package:
        raise KeyError(
            f"{profile_path}: host_roles.rt_host.bringup.package is required"
        )
    ws_package = ws_bringup.get("package")
    config_packages: list[str] = []
    for package_name in (ws_package, rt_package):
        if package_name and str(package_name) not in config_packages:
            config_packages.append(str(package_name))
    if not config_packages:
        raise KeyError(
            f"{profile_path}: host_roles.*.bringup.package is required "
            "to resolve recording/camera configs"
        )
    config_shares = [
        Path(get_package_share_directory(name)) for name in config_packages
    ]

    stream_name = recorder.get("profile")
    if not stream_name:
        raise KeyError(f"{profile_path}: recorder.profile is required")
    recording_stream_config = None
    for share in config_shares:
        candidate = share / "config" / "recording" / f"{stream_name}.yaml"
        if candidate.is_file():
            recording_stream_config = candidate
            break
    if recording_stream_config is None:
        searched = ", ".join(str(s / "config" / "recording") for s in config_shares)
        raise FileNotFoundError(
            f"recording stream config {stream_name!r} not found under: {searched}"
        )

    root_dir = recorder.get("root_dir")
    task = recorder.get("task")
    experiment_name = recorder.get("experiment_name") or metadata.get("name")
    if not root_dir or not task or not experiment_name:
        raise KeyError(
            f"{profile_path}: recorder.root_dir, recorder.task, and "
            "recorder.experiment_name (or metadata.name) are required"
        )

    max_command_age_s = args.get("max_command_age_s")
    if max_command_age_s is None:
        raise KeyError(
            f"{profile_path}: host_roles.workstation_host.bringup.arguments."
            "max_command_age_s is required"
        )

    for required_flag in ("with_execution_manager", "with_recorder"):
        if required_flag not in args:
            raise KeyError(
                f"{profile_path}: host_roles.workstation_host.bringup.arguments."
                f"{required_flag} is required"
            )

    defaults: dict[str, str] = {
        "embodiment_profile": str(profile_path),
        "recording_stream_config": str(recording_stream_config),
        "root_dir": str(root_dir),
        "experiment_name": str(experiment_name),
        "task": str(task),
        "max_command_age_s": str(max_command_age_s),
        "with_execution_manager": _as_launch_bool(args["with_execution_manager"]),
        "with_recorder": _as_launch_bool(args["with_recorder"]),
    }

    camera_config = args.get("camera_config")
    if camera_config:
        camera_path = Path(str(camera_config))
        if not camera_path.is_absolute():
            resolved = None
            for share in config_shares:
                candidate = share / camera_path
                if candidate.is_file():
                    resolved = candidate
                    break
            if resolved is None:
                searched = ", ".join(str(s / camera_path) for s in config_shares)
                raise FileNotFoundError(
                    f"camera_config {camera_config!r} not found under: {searched}"
                )
            camera_path = resolved
        defaults["camera_config"] = str(camera_path)

    for key in (
        "with_cameras",
        "with_orbbec",
        "with_realsense",
        "with_leaders",
    ):
        if key in args:
            defaults[key] = _as_launch_bool(args[key])

    return defaults
