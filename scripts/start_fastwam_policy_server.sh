#!/usr/bin/env bash
# Start LeRobot gRPC policy_server on ModelArts for FastWAM remote inference.
#
# Run ON the cloud node (terminal 16 / ssh modelarts-4d-assert):
#   bash start_fastwam_policy_server.sh
#
# Or from the laptop:
#   ssh modelarts-4d-assert 'bash -s' < scripts/start_fastwam_policy_server.sh
#
# Local client then:
#   ssh -N -L 50051:127.0.0.1:50051 modelarts-4d-assert
#   python apps/eval.py ... --inference remote --remote-address 127.0.0.1:50051 \
#     --policy-type fastwam \
#     --checkpoint /mnt/dev/lerobot/outputs/libero_fastwam/checkpoints/100000/pretrained_model
#
# Notes:
# - Weights load on first client handshake (SendPolicyInstructions), not at listen time.
# - Official SUPPORTED_POLICIES omits fastwam; this script patches it in-process.

set -euo pipefail

HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-50051}"
FPS="${FPS:-30}"
# FastWAM / Wan forward can take seconds; keep queue timeout generous.
INFERENCE_LATENCY="${INFERENCE_LATENCY:-0.5}"
OBS_QUEUE_TIMEOUT="${OBS_QUEUE_TIMEOUT:-60}"

export HF_HOME="${HF_HOME:-/mnt/dev/lerobot/model_cache/huggingface}"
export TRANSFORMERS_CACHE="${TRANSFORMERS_CACHE:-$HF_HOME}"
export HUGGINGFACE_HUB_CACHE="${HUGGINGFACE_HUB_CACHE:-$HF_HOME/hub}"
# Prefer online if bases already cached; unset offline flags if present.
unset HF_HUB_OFFLINE TRANSFORMERS_OFFLINE || true

cd /lerobot

echo "[policy_server] host=${HOST} port=${PORT} fps=${FPS}"
echo "[policy_server] HF_HOME=${HF_HOME}"
echo "[policy_server] waiting for client SendPolicyInstructions to load FastWAM..."
nvidia-smi -L 2>/dev/null || true

exec python - <<PY
from lerobot.async_inference import constants
from lerobot.async_inference.configs import PolicyServerConfig
from lerobot.async_inference.policy_server import serve

class _PermissivePolicyList(list):
    def __contains__(self, item):
        return True

constants.SUPPORTED_POLICIES = _PermissivePolicyList(constants.SUPPORTED_POLICIES)
print("[policy_server] Permissive SUPPORTED_POLICIES enabled.")

cfg = PolicyServerConfig(
    host="${HOST}",
    port=int("${PORT}"),
    fps=int("${FPS}"),
    inference_latency=float("${INFERENCE_LATENCY}"),
    obs_queue_timeout=float("${OBS_QUEUE_TIMEOUT}"),
)
serve(cfg)
PY
