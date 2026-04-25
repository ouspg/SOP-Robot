#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_ROOT"

if ! command -v basedpyright >/dev/null 2>&1; then
    exec pixi exec basedpyright --project pyrightconfig.json
fi

exec basedpyright --project pyrightconfig.json
