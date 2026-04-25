#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_ROOT"

if ! command -v ruff >/dev/null 2>&1; then
    exec pixi exec ruff check src tools testing
fi

exec ruff check src tools testing
