"""Workspace-local CloudXR host-client setup and compatibility layer."""

from __future__ import annotations

import io
import logging
import mimetypes
import re
import tarfile
import urllib.request
from pathlib import Path

WEBXR_ASSETS_VERSION = "1.0.20"
WEBXR_ASSETS_ROOT = (
    Path("npm") / "@webxr-input-profiles" / f"assets@{WEBXR_ASSETS_VERSION}"
)
WEBXR_PROFILES_ROOT = WEBXR_ASSETS_ROOT / "dist" / "profiles"


def required_asset_paths(static_dir: Path) -> tuple[Path, ...]:
    """Return the minimum files required by the local Quest host client."""
    profiles = static_dir / WEBXR_PROFILES_ROOT
    return (
        static_dir / "index.html",
        static_dir / "bundle.js",
        profiles / "profilesList.json",
        profiles / "meta-quest-touch-plus" / "left.glb",
        profiles / "meta-quest-touch-plus" / "right.glb",
        profiles / "meta-quest-touch-plus-v2" / "left.glb",
        profiles / "meta-quest-touch-plus-v2" / "right.glb",
    )


def prepare_workspace_cloudxr(cloudxr_dir: Path, logger=None) -> Path:
    """Download and patch all assets needed by offline ``host_client`` mode."""
    logger = logger or logging.getLogger(__name__)
    cloudxr_dir = cloudxr_dir.expanduser().resolve()
    static_dir = cloudxr_dir / "static-client"
    static_dir.mkdir(parents=True, exist_ok=True)
    (cloudxr_dir / "cloudxr-env-config.env").touch(exist_ok=True)

    import os

    os.environ["TELEOP_WEB_CLIENT_STATIC_DIR"] = str(static_dir)
    from isaacteleop.cloudxr.oob_teleop_env import require_web_client_static_dir

    require_web_client_static_dir()
    _download_webxr_profiles(static_dir, logger)
    _patch_client_assets(static_dir, logger)
    validate_workspace_cloudxr(cloudxr_dir)
    return static_dir


def validate_workspace_cloudxr(cloudxr_dir: Path) -> Path:
    """Validate setup without performing network or modifying installed assets."""
    cloudxr_dir = cloudxr_dir.expanduser().resolve()
    static_dir = cloudxr_dir / "static-client"
    required = (
        cloudxr_dir / "cloudxr-env-config.env",
        *required_asset_paths(static_dir),
    )
    missing = [path for path in required if not path.is_file()]
    if missing:
        details = "\n".join(f"  - {path}" for path in missing)
        raise RuntimeError(
            "CloudXR host-client assets are not prepared. Run "
            "`ros2 run isaacteleop_toolbox isaacteleop-cloudxr-setup` "
            "with network access.\n"
            f"Missing files:\n{details}"
        )

    index = (static_dir / "index.html").read_text(encoding="utf-8")
    bundle = (static_dir / "bundle.js").read_bytes()
    if 'src="/client/bundle.js"' not in index:
        raise RuntimeError("CloudXR index.html is not patched for the /client/ route")
    if _local_profiles_url().encode() not in bundle:
        raise RuntimeError(
            "CloudXR bundle.js is not patched for local controller assets"
        )
    return static_dir


def apply_static_asset_compatibility_patch() -> None:
    """Extend IsaacTeleop's host-client handler to serve nested local assets."""
    from isaacteleop.cloudxr import wss
    from websockets.datastructures import Headers
    from websockets.http11 import Response

    if getattr(wss, "_isaacteleop_toolbox_static_patch", False):
        return
    if not hasattr(wss, "_make_http_handler") or not hasattr(
        wss, "_normalize_request_path"
    ):
        raise RuntimeError("Unsupported IsaacTeleop CloudXR WSS API")

    original_make_http_handler = wss._make_http_handler

    def patched_make_http_handler(
        backend_host, backend_port, hub=None, static_dir=None
    ):
        handler = original_make_http_handler(
            backend_host, backend_port, hub=hub, static_dir=static_dir
        )

        async def patched_handler(connection, request):
            path = wss._normalize_request_path(request.path or "/")
            raw_path = request.path or "/"
            if (
                static_dir is not None
                and path == "/client"
                and not raw_path.split("?")[0].endswith("/")
            ):
                query = raw_path[len(path) :]
                return Response(
                    301,
                    "Moved Permanently",
                    Headers({"Location": f"/client/{query}", **wss.CORS_HEADERS}),
                    b"",
                )

            is_client_path = path == "/client" or path.startswith("/client/")
            is_profile_path = path.startswith("/npm/@webxr-input-profiles/")
            if static_dir is not None and (is_client_path or is_profile_path):
                tail = (
                    path[len("/client") :].lstrip("/")
                    if is_client_path
                    else path.lstrip("/")
                ) or "index.html"
                root = Path(static_dir).resolve()
                candidate = (root / tail).resolve()
                try:
                    candidate.relative_to(root)
                except ValueError:
                    return Response(403, "Forbidden", Headers(wss.CORS_HEADERS), b"")
                if candidate.is_file():
                    content_type = (
                        mimetypes.guess_type(candidate.name)[0]
                        or "application/octet-stream"
                    )
                    return Response(
                        200,
                        "OK",
                        Headers({"Content-Type": content_type, **wss.CORS_HEADERS}),
                        candidate.read_bytes(),
                    )
            return await handler(connection, request)

        return patched_handler

    wss._make_http_handler = patched_make_http_handler
    wss._isaacteleop_toolbox_static_patch = True


def _download_webxr_profiles(static_dir: Path, logger) -> None:
    profiles_list = static_dir / WEBXR_PROFILES_ROOT / "profilesList.json"
    if profiles_list.is_file():
        return
    url = (
        "https://registry.npmjs.org/@webxr-input-profiles/assets/-/"
        f"assets-{WEBXR_ASSETS_VERSION}.tgz"
    )
    logger.info("Downloading WebXR controller assets from %s", url)
    request = urllib.request.Request(url, headers={"User-Agent": "isaacteleop-toolbox"})
    with urllib.request.urlopen(request, timeout=120) as response:
        archive = response.read()
    with tarfile.open(fileobj=io.BytesIO(archive), mode="r:gz") as tar:
        prefix = "package/dist/profiles/"
        for member in tar.getmembers():
            if not member.isfile() or not member.name.startswith(prefix):
                continue
            relative = Path(member.name.removeprefix("package/"))
            target = _safe_asset_target(static_dir, relative, member.name)
            target.parent.mkdir(parents=True, exist_ok=True)
            source = tar.extractfile(member)
            if source is not None:
                target.write_bytes(source.read())


def _safe_asset_target(static_dir: Path, relative: Path, member_name: str) -> Path:
    if relative.is_absolute() or ".." in relative.parts:
        raise RuntimeError(f"Unsafe path in WebXR asset archive: {member_name}")
    asset_root = (static_dir / WEBXR_ASSETS_ROOT).resolve()
    target = (asset_root / relative).resolve()
    try:
        target.relative_to(asset_root)
    except ValueError as exc:
        raise RuntimeError(
            f"Unsafe path in WebXR asset archive: {member_name}"
        ) from exc
    return target


def _local_profiles_url() -> str:
    return f'new URL("{WEBXR_PROFILES_ROOT.as_posix()}/",window.location.href).href'


def _patch_client_assets(static_dir: Path, logger) -> None:
    index_path = static_dir / "index.html"
    index = index_path.read_text(encoding="utf-8")
    patched_index = index.replace('src="bundle.js"', 'src="/client/bundle.js"')
    patched_index = patched_index.replace(
        'href="favicon.ico"', 'href="/client/favicon.ico"'
    )
    if patched_index != index:
        index_path.write_text(patched_index, encoding="utf-8")
        logger.info("Patched WebXR index paths for /client/")

    bundle_path = static_dir / "bundle.js"
    bundle_str = bundle_path.read_text(encoding="utf-8")
    patched_bundle, count = re.subn(
        r'([a-zA-Z0-9_$]+)=["\']https://cdn\.jsdelivr\.net/npm/@webxr-input-profiles/assets@1\.0/dist/profiles/["\']',
        rf'\1={_local_profiles_url()}',
        bundle_str,
    )
    if count > 0:
        bundle_path.write_text(patched_bundle, encoding="utf-8")
        logger.info("Patched WebXR controller profiles to workspace-local assets")
