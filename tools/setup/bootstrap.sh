#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$PROJECT_ROOT"

if ! command -v pixi >/dev/null 2>&1; then
    cat >&2 <<'EOF'
ERROR: pixi is not installed or not on PATH.
Install it with:
  curl -fsSL https://pixi.sh/install.sh | bash
EOF
    exit 1
fi

pixi install
pixi run init-submodules
pixi run rosdep-install
pixi run install-llama-cuda
pixi run install-whispercpp-cuda
pixi run install-crispasr
pixi run setup-models
pixi run build
pixi run test
