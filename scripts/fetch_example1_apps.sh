#!/usr/bin/env bash
# Place runtime_resources apps/ into src/apps/ (no /tmp clone; re-runnable).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "${ROOT}/src/apps"
curl -fsSL https://github.com/Gabriel-Ning/runtime_resources/archive/refs/heads/main.tar.gz \
  | tar -xz --strip-components=2 -C "${ROOT}/src/apps" runtime_resources-main/apps
echo "Installed runtime_resources apps/ → ${ROOT}/src/apps/"
ls -1 "${ROOT}/src/apps"
