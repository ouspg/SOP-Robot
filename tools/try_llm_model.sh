#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_ROOT"

requires_llama_cpp=0
for arg in "$@"; do
    if [[ "$arg" == *"llama-cpp"* ]]; then
        requires_llama_cpp=1
        break
    fi
done

require_gpu="$SOP_ROBOT_LLM_BENCH_REQUIRE_GPU"
case "${require_gpu,,}" in
    0|false|no|off)
        requires_llama_cpp=0
        ;;
esac

if [[ "$requires_llama_cpp" == "1" ]]; then
    bash tools/setup/install_llama_cuda.sh
fi

exec bash tools/with_ros_env.sh python testing/llm/try_model.py "$@"
