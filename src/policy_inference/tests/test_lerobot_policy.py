from __future__ import annotations

import subprocess
import sys


def test_lerobot_policy_loader_import_is_lazy() -> None:
    code = """
import sys
import policy_inference.lerobot.policy
assert 'torch' not in sys.modules
assert 'lerobot.policies' not in sys.modules
"""
    subprocess.run([sys.executable, "-c", code], check=True)
