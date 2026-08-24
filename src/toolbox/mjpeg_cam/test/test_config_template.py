from pathlib import Path

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]

DECLARED_PARAMS = (
    "video_device",
    "frame_id",
    "format",
    "image_width",
    "image_height",
    "framerate",
    "compressed_topic",
    "camera_name",
)

LAUNCH_ARGS = (
    "params_file",
    "namespace",
    "name",
    "video_device",
    "frame_id",
    "camera_name",
    "compressed_topic",
)


def test_template_covers_declared_node_parameters() -> None:
    loaded = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "mjpeg_cam.yaml").read_text(encoding="utf-8")
    )
    params = loaded["/**"]["ros__parameters"]
    assert set(params) == set(DECLARED_PARAMS)
    node_src = (PACKAGE_ROOT / "src" / "node.cpp").read_text(encoding="utf-8")
    for name in DECLARED_PARAMS:
        assert f'declare_parameter("{name}"' in node_src


def test_launch_declares_startup_config_arguments() -> None:
    source = (PACKAGE_ROOT / "launch" / "mjpeg_cam.launch.py").read_text(
        encoding="utf-8"
    )
    for argument in LAUNCH_ARGS:
        assert f'"{argument}"' in source
    assert "mjpeg_cam.yaml" in source
