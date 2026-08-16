#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
MODEL_DIR="$REPO_ROOT/src/tts_package/resource"
MODEL_CONFIG="$MODEL_DIR/config.json"
MODEL_URL="https://github.com/ouspg/SOP-Robot/releases/download/model/model.zip"

mkdir -p "$MODEL_DIR"

if [[ -f "$MODEL_CONFIG" ]]; then
  echo "TTS model already present: $MODEL_CONFIG"
  exit 0
fi

tmpfile="$(mktemp)"
trap 'rm -f "$tmpfile"' EXIT

curl -fL "$MODEL_URL" -o "$tmpfile"
unzip -o "$tmpfile" -d "$MODEL_DIR"
