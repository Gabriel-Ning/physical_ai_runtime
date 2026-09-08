#!/usr/bin/env bash
# Universal LeRobot gRPC policy_server launcher for remote / cloud GPU inference.
#
# Supports any LeRobot policy (fastwam, act, diffusion, pi0, etc.) without
# hardcoded policy type restrictions.
#
# Usage ON the cloud node / Docker container:
#   bash scripts/start_policy_server.sh
#
# Or pipe from laptop via SSH:
#   ssh modelarts-4d-assert 'bash -s' < scripts/start_policy_server.sh
#
# Connect from local evaluation client:
#   ssh -N -L 50051:127.0.0.1:50051 modelarts-4d-assert
#   python apps/eval.py ... --inference remote --remote-address 127.0.0.1:50051 \
#     --policy-type <policy_type> \
#     --checkpoint /path/to/pretrained_model/on/cloud

set -euo pipefail

HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-50051}"
FPS="${FPS:-30}"
INFERENCE_LATENCY="${INFERENCE_LATENCY:-0.5}"
OBS_QUEUE_TIMEOUT="${OBS_QUEUE_TIMEOUT:-60}"


if [ -d "/lerobot" ]; then
    cd /lerobot
fi

echo "[policy_server] host=${HOST} port=${PORT} fps=${FPS}"
echo "[policy_server] TRANSFORMERS_CACHE=${TRANSFORMERS_CACHE:-}"
echo "[policy_server] waiting for client SendPolicyInstructions to load policy weights on GPU..."
nvidia-smi -L 2>/dev/null || true

exec python - <<PY
from lerobot.async_inference import constants
from lerobot.async_inference.configs import PolicyServerConfig
from lerobot.async_inference.policy_server import serve

# Make SUPPORTED_POLICIES permissive so any policy resolvable by get_policy_class is accepted
class _PermissivePolicyList(list):
    def __contains__(self, item):
        return True

constants.SUPPORTED_POLICIES = _PermissivePolicyList(constants.SUPPORTED_POLICIES)
print("[policy_server] Enabled universal policy support (permissive dynamic resolution).")

cfg = PolicyServerConfig(
    host="${HOST}",
    port=int("${PORT}"),
    fps=int("${FPS}"),
    inference_latency=float("${INFERENCE_LATENCY}"),
    obs_queue_timeout=float("${OBS_QUEUE_TIMEOUT}"),
)
serve(cfg)
PY
