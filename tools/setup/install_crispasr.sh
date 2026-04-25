#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$PROJECT_ROOT"

if [[ -z "${CONDA_PREFIX:-}" ]]; then
    if [[ -x "$PROJECT_ROOT/.pixi/envs/default/bin/python" ]]; then
        export CONDA_PREFIX="$PROJECT_ROOT/.pixi/envs/default"
    else
        cat >&2 <<'EOF'
ERROR: Pixi environment not found.
Run `pixi install` first, then rerun `pixi run install-crispasr`.
EOF
        exit 1
    fi
fi

PYTHON="$CONDA_PREFIX/bin/python"
if [[ ! -x "$PYTHON" ]]; then
    echo "ERROR: Python was not found at $PYTHON" >&2
    exit 1
fi

if ! command -v cmake >/dev/null 2>&1; then
    echo "ERROR: cmake was not found. Run 'pixi install' first." >&2
    exit 1
fi

if ! command -v git >/dev/null 2>&1; then
    echo "ERROR: git was not found. Run 'pixi install' first." >&2
    exit 1
fi

mapfile -t CONFIG_VALUES < <("$PYTHON" - <<'PY'
import json
from pathlib import Path

config_path = Path("config/voice_chatbot.json")
config = json.loads(config_path.read_text()) if config_path.is_file() else {}
root = Path(config.get("crispasr_root") or "/home/aapot/CrispASR").expanduser()
print(root)
print(Path(config.get("crispasr_python_path") or root / "python").expanduser())
print(Path(config.get("crispasr_lib_path") or root / "build/src/libcrispasr.so").expanduser())
print(Path(config.get("crispasr_model_path") or root / "parakeet.gguf").expanduser())
print(config.get("crispasr_backend") or "parakeet")
PY
)

CONFIG_ROOT="${CONFIG_VALUES[0]}"
CONFIG_PYTHON_PATH="${CONFIG_VALUES[1]}"
CONFIG_LIB_PATH="${CONFIG_VALUES[2]}"
CONFIG_MODEL_PATH="${CONFIG_VALUES[3]}"
CONFIG_BACKEND="${CONFIG_VALUES[4]}"

if [[ -n "${CRISPASR_ROOT:-}" ]]; then
    CRISPASR_ROOT="$(realpath -m "$CRISPASR_ROOT")"
    CRISPASR_PYTHON_PATH="${CRISPASR_PYTHON_PATH:-$CRISPASR_ROOT/python}"
    CRISPASR_LIB_PATH="${CRISPASR_LIB_PATH:-$CRISPASR_ROOT/build/src/libcrispasr.so}"
    CRISPASR_MODEL_PATH="${CRISPASR_MODEL_PATH:-$CRISPASR_ROOT/parakeet.gguf}"
else
    CRISPASR_ROOT="$CONFIG_ROOT"
    CRISPASR_PYTHON_PATH="${CRISPASR_PYTHON_PATH:-$CONFIG_PYTHON_PATH}"
    CRISPASR_LIB_PATH="${CRISPASR_LIB_PATH:-$CONFIG_LIB_PATH}"
    CRISPASR_MODEL_PATH="${CRISPASR_MODEL_PATH:-$CONFIG_MODEL_PATH}"
fi

: "${CRISPASR_REPO_URL:?CRISPASR_REPO_URL is set by pixi.toml; run this through pixi}"
CRISPASR_BACKEND="${CRISPASR_BACKEND:-$CONFIG_BACKEND}"
CRISPASR_BUILD_DIR="${CRISPASR_BUILD_DIR:-$CRISPASR_ROOT/build}"
: "${CRISPASR_MODEL_URL:?CRISPASR_MODEL_URL is set by pixi.toml; run this through pixi}"

clone_or_prepare_repo() {
    if [[ -d "$CRISPASR_ROOT/.git" ]]; then
        if [[ "$CRISPASR_UPDATE" == "1" ]]; then
            git -C "$CRISPASR_ROOT" pull --ff-only
        fi
        git -C "$CRISPASR_ROOT" submodule update --init --recursive
        return
    fi

    if [[ -f "$CRISPASR_ROOT/CMakeLists.txt" ]]; then
        echo "Using existing non-git CrispASR source tree: $CRISPASR_ROOT"
        return
    fi

    if [[ -e "$CRISPASR_ROOT" ]]; then
        echo "ERROR: $CRISPASR_ROOT exists but is not a CrispASR source tree." >&2
        exit 1
    fi

    mkdir -p "$(dirname "$CRISPASR_ROOT")"
    git clone "$CRISPASR_REPO_URL" "$CRISPASR_ROOT"
    git -C "$CRISPASR_ROOT" submodule update --init --recursive
}

cuda_is_available() {
    "$PYTHON" - <<'PY'
try:
    import torch
except Exception:
    raise SystemExit(1)
raise SystemExit(0 if torch.cuda.is_available() else 1)
PY
}

cuda_arch() {
    "$PYTHON" - <<'PY'
try:
    import torch
    major, minor = torch.cuda.get_device_capability(0)
except Exception:
    raise SystemExit(1)
print(f"{major}{minor}")
PY
}

configure_cuda_environment() {
    : "${CUDA_HOME:?CUDA_HOME is set by pixi.toml; run this through pixi}"

    if [[ -z "$CUDA_HOME" || ! -x "$CUDA_HOME/bin/nvcc" ]]; then
        echo "ERROR: CUDA was requested, but nvcc was not found." >&2
        echo "Set CUDA_HOME or run 'pixi install' so the Pixi CUDA toolkit is available." >&2
        exit 1
    fi

    if [[ -d "$CUDA_HOME/lib" && ! -e "$CUDA_HOME/lib64" ]]; then
        ln -s lib "$CUDA_HOME/lib64"
    fi

    local cuda_lib_dir="$CUDA_HOME/lib"
    if [[ -d "$CUDA_HOME/targets/x86_64-linux/lib" ]]; then
        cuda_lib_dir="$CUDA_HOME/targets/x86_64-linux/lib"
    elif [[ -d "$CUDA_HOME/lib64" ]]; then
        cuda_lib_dir="$CUDA_HOME/lib64"
    fi

    local driver_lib_dir=""
    if [[ -e /usr/lib/wsl/lib/libcuda.so.1 ]]; then
        driver_lib_dir="/usr/lib/wsl/lib"
    elif ldconfig -p 2>/dev/null | grep -q "libcuda.so.1"; then
        driver_lib_dir="$(dirname "$(ldconfig -p | awk '/libcuda\.so\.1[[:space:]]/ { print $NF; exit }')")"
    fi

    export CUDA_HOME
    export CUDAToolkit_ROOT="$CUDA_HOME"
    export CUDACXX="$CUDA_HOME/bin/nvcc"
    export CUDAHOSTCXX="${CUDAHOSTCXX:-/usr/bin/c++}"
    export PATH="$CUDA_HOME/bin:/usr/bin:/bin:$CONDA_PREFIX/bin:$PATH"
    export LD_LIBRARY_PATH="$cuda_lib_dir${driver_lib_dir:+:$driver_lib_dir}:${CONDA_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
    export LIBRARY_PATH="$cuda_lib_dir${driver_lib_dir:+:$driver_lib_dir}:${LIBRARY_PATH:-}"
}

download_model() {
    if [[ -s "$CRISPASR_MODEL_PATH" ]]; then
        size_mb=$(du -m "$CRISPASR_MODEL_PATH" | awk '{print $1}')
        echo "CrispASR model already exists: $CRISPASR_MODEL_PATH (${size_mb} MiB)"
        return
    fi

    mkdir -p "$(dirname "$CRISPASR_MODEL_PATH")"
    tmp_path="${CRISPASR_MODEL_PATH}.tmp.$$"
    rm -f "$tmp_path"
    trap 'rm -f "$tmp_path"' EXIT

    echo "Downloading CrispASR Parakeet model to: $CRISPASR_MODEL_PATH"
    if command -v curl >/dev/null 2>&1; then
        curl -L --fail --retry 3 -o "$tmp_path" "$CRISPASR_MODEL_URL"
    elif command -v wget >/dev/null 2>&1; then
        wget -O "$tmp_path" "$CRISPASR_MODEL_URL"
    else
        CRISPASR_MODEL_URL="$CRISPASR_MODEL_URL" \
        CRISPASR_TMP_MODEL_PATH="$tmp_path" \
        "$PYTHON" - <<'PY'
import os
import urllib.request

urllib.request.urlretrieve(
    os.environ["CRISPASR_MODEL_URL"],
    os.environ["CRISPASR_TMP_MODEL_PATH"],
)
PY
    fi
    mv "$tmp_path" "$CRISPASR_MODEL_PATH"
    trap - EXIT
}

artifact_ok() {
    local binary="$CRISPASR_BUILD_DIR/bin/crispasr"
    [[ -x "$binary" ]] || return 1
    [[ -e "$CRISPASR_LIB_PATH" ]] || return 1
    local backend_list
    backend_list="$("$binary" --list-backends 2>/dev/null)" || return 1
    grep -Eq "^[[:space:]]*$CRISPASR_BACKEND[[:space:]]" <<<"$backend_list"
}

build_crispasr() {
    local enable_cuda="$CRISPASR_ENABLE_CUDA"
    local use_cuda=0
    case "$enable_cuda" in
        1|true|TRUE|on|ON|yes|YES)
            use_cuda=1
            ;;
        0|false|FALSE|off|OFF|no|NO)
            use_cuda=0
            ;;
        auto)
            if cuda_is_available; then
                use_cuda=1
            fi
            ;;
        *)
            echo "ERROR: CRISPASR_ENABLE_CUDA must be auto, 1, or 0." >&2
            exit 1
            ;;
    esac

    if [[ "$use_cuda" -eq 1 ]]; then
        configure_cuda_environment
    fi

    if [[ "$CRISPASR_FORCE_REBUILD" != "1" ]] && artifact_ok; then
        echo "CrispASR build already looks valid: $CRISPASR_BUILD_DIR"
        return
    fi

    local cmake_args=(
        -S "$CRISPASR_ROOT"
        -B "$CRISPASR_BUILD_DIR"
        -DCMAKE_BUILD_TYPE=Release
    )

    if [[ "$CRISPASR_USE_SYSTEM_TOOLCHAIN" == "1" ]]; then
        if [[ -x /usr/bin/cc && -x /usr/bin/c++ && -x /usr/bin/ld ]]; then
            cmake_args+=(
                -DCMAKE_C_COMPILER=/usr/bin/cc
                -DCMAKE_CXX_COMPILER=/usr/bin/c++
                -DCMAKE_LINKER=/usr/bin/ld
            )
            if [[ "$use_cuda" -eq 1 ]]; then
                cmake_args+=(-DCMAKE_CUDA_HOST_COMPILER=/usr/bin/c++)
            fi
        fi
    fi

    if [[ "$use_cuda" -eq 1 ]]; then
        local arch="${CRISPASR_CUDA_ARCHITECTURES:-}"
        if [[ -z "$arch" ]]; then
            arch="$(cuda_arch || true)"
        fi
        cmake_args+=(
            -DGGML_CUDA=ON
            -DGGML_CUDA_ENABLE_UNIFIED_MEMORY=1
        )
        if [[ -n "$arch" ]]; then
            cmake_args+=("-DCMAKE_CUDA_ARCHITECTURES=$arch")
            echo "Building CrispASR with CUDA for sm_$arch."
        else
            echo "Building CrispASR with CUDA using CMake's default CUDA architecture detection."
        fi
    else
        cmake_args+=(-DGGML_CUDA=OFF)
        echo "Building CrispASR without CUDA because no CUDA GPU/toolkit was detected."
    fi

    if [[ -n "${CRISPASR_CMAKE_ARGS:-}" ]]; then
        # shellcheck disable=SC2206
        local extra_args=($CRISPASR_CMAKE_ARGS)
        cmake_args+=("${extra_args[@]}")
    fi

    cmake "${cmake_args[@]}"

    local build_target=()
    local target_help
    target_help="$(cmake --build "$CRISPASR_BUILD_DIR" --target help 2>/dev/null || true)"
    if grep -Eq "^[.][.][.] crispasr([[:space:]]|$)" <<<"$target_help"; then
        build_target=(--target crispasr)
    elif grep -Eq "^[.][.][.] whisper-cli([[:space:]]|$)" <<<"$target_help"; then
        build_target=(--target whisper-cli)
    fi

    cmake --build "$CRISPASR_BUILD_DIR" --parallel "${CRISPASR_JOBS:-$(nproc)}" "${build_target[@]}"
}

verify_python_binding() {
    CRISPASR_VERIFY_PYTHON_PATH="$CRISPASR_PYTHON_PATH" \
    CRISPASR_VERIFY_LIB_PATH="$CRISPASR_LIB_PATH" \
    CRISPASR_VERIFY_MODEL_PATH="$CRISPASR_MODEL_PATH" \
    CRISPASR_VERIFY_BACKEND="$CRISPASR_BACKEND" \
    "$PYTHON" - <<'PY'
import importlib
import os
import sys
from pathlib import Path

python_path = Path(os.environ["CRISPASR_VERIFY_PYTHON_PATH"])
lib_path = Path(os.environ["CRISPASR_VERIFY_LIB_PATH"])
model_path = Path(os.environ["CRISPASR_VERIFY_MODEL_PATH"])
backend = os.environ["CRISPASR_VERIFY_BACKEND"].strip()

if not python_path.is_dir():
    raise SystemExit(f"CrispASR Python binding path not found: {python_path}")
if not lib_path.exists():
    raise SystemExit(f"CrispASR shared library not found: {lib_path}")
if not model_path.is_file():
    raise SystemExit(f"CrispASR model not found: {model_path}")

sys.path.insert(0, str(python_path))
module = importlib.import_module("crispasr")
session_cls = module.Session
available = session_cls.available_backends(lib_path=str(lib_path))
if backend and backend not in available:
    raise SystemExit(
        f"CrispASR backend {backend!r} is not available in {lib_path}. "
        f"Available backends: {available}"
    )

session = session_cls(
    str(model_path),
    lib_path=str(lib_path),
    n_threads=1,
    backend=backend or None,
)
try:
    print(
        "CrispASR Python binding verified: "
        f"backend={session.backend}, model={model_path}, lib={lib_path}"
    )
finally:
    session.close()
PY
}

clone_or_prepare_repo
download_model
build_crispasr
verify_python_binding
