#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_ROOT"

if [[ -z "${PIXI_ENVIRONMENT_NAME:-}" ]]; then
    PIXI_BIN="${PIXI_BIN:-$(command -v pixi || true)}"
    if [[ -z "$PIXI_BIN" && -x "$HOME/.pixi/bin/pixi" ]]; then
        PIXI_BIN="$HOME/.pixi/bin/pixi"
    fi
    if [[ -z "$PIXI_BIN" ]]; then
        echo "ERROR: pixi was not found on PATH or at \$HOME/.pixi/bin/pixi." >&2
        exit 1
    fi
    exec "$PIXI_BIN" run bash "$0" "$@"
fi

if [[ ! -f install/setup.bash ]]; then
    echo "ERROR: install/setup.bash not found. Run 'pixi run build' first." >&2
    exit 1
fi

# Pixi activation sources install/setup.sh from pixi.toml. Source the bash
# overlay only as a fallback for manually activated environments.
if [[ ":${COLCON_PREFIX_PATH:-}:" != *":$PROJECT_ROOT/install:"* ]]; then
    set +u
    source install/setup.bash
    set -u
fi

exec "$@"
