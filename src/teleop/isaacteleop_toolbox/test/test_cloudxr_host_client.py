"""Offline tests for the workspace-local CloudXR asset contract."""

import tempfile
import unittest
from pathlib import Path

from isaacteleop_toolbox.cloudxr_host_client import (
    WEBXR_PROFILES_ROOT,
    _local_profiles_url,
    _patch_client_assets,
    _safe_asset_target,
    required_asset_paths,
    validate_workspace_cloudxr,
)


class _Logger:
    def info(self, *args, **kwargs):
        pass


class CloudXRHostClientTest(unittest.TestCase):
    def test_patch_and_validate_workspace_assets(self):
        with tempfile.TemporaryDirectory() as tmp:
            cloudxr_dir = Path(tmp)
            static_dir = cloudxr_dir / "static-client"
            static_dir.mkdir()
            (cloudxr_dir / "cloudxr-env-config.env").touch()
            (static_dir / "index.html").write_text(
                '<script src="bundle.js"></script>', encoding="utf-8"
            )
            remote = 'Le="https://cdn.jsdelivr.net/npm/@webxr-input-profiles/assets@1.0/dist/profiles/"'
            (static_dir / "bundle.js").write_text(remote, encoding="utf-8")
            for path in required_asset_paths(static_dir)[2:]:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(b"fixture")

            _patch_client_assets(static_dir, _Logger())

            self.assertEqual(validate_workspace_cloudxr(cloudxr_dir), static_dir)
            self.assertIn(
                'src="/client/bundle.js"',
                (static_dir / "index.html").read_text(encoding="utf-8"),
            )
            self.assertIn(
                _local_profiles_url(),
                (static_dir / "bundle.js").read_text(encoding="utf-8"),
            )

    def test_validation_reports_missing_assets(self):
        with tempfile.TemporaryDirectory() as tmp, self.assertRaisesRegex(
            RuntimeError, "cloudxr-setup"
        ):
            validate_workspace_cloudxr(Path(tmp))

    def test_profile_path_is_versioned_and_nested(self):
        self.assertEqual(
            WEBXR_PROFILES_ROOT.as_posix(),
            "npm/@webxr-input-profiles/assets@1.0.20/dist/profiles",
        )

    def test_asset_archive_paths_cannot_escape_static_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            static_dir = Path(tmp)
            with self.assertRaisesRegex(RuntimeError, "Unsafe path"):
                _safe_asset_target(
                    static_dir,
                    Path("dist/profiles/../../../../escaped"),
                    "package/dist/profiles/../../../../escaped",
                )


if __name__ == "__main__":
    unittest.main()
