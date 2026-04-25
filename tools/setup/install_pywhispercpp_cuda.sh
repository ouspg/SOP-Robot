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
Run `pixi install` first, then rerun `pixi run install-whispercpp-cuda`.
EOF
        exit 1
    fi
fi

PYTHON="$CONDA_PREFIX/bin/python"
if [[ ! -x "$PYTHON" ]]; then
    echo "ERROR: Python was not found at $PYTHON" >&2
    exit 1
fi

if ! "$PYTHON" -c "import torch; raise SystemExit(0 if torch.cuda.is_available() else 1)" >/dev/null; then
    cat >&2 <<'EOF'
ERROR: Torch cannot see a CUDA GPU from this Pixi environment.
Check `nvidia-smi` in WSL2 and reinstall the Pixi environment if needed.
EOF
    exit 1
fi

SITE_PACKAGES="$("$PYTHON" -c "import sysconfig; print(sysconfig.get_paths()['purelib'])")"
NVIDIA_ROOT="$SITE_PACKAGES/nvidia"
: "${CUDA_HOME:?CUDA_HOME is set by pixi.toml; run this through pixi}"

if [[ ! -x "$CUDA_HOME/bin/nvcc" ]]; then
    cat >&2 <<EOF
ERROR: CUDA nvcc was not found at $CUDA_HOME/bin/nvcc.
Run `pixi install`; the manifest should install nvidia-cuda-nvcc.
EOF
    exit 1
fi

if [[ ! -d "$CUDA_HOME/include/cccl" ]]; then
    cat >&2 <<EOF
ERROR: CUDA CCCL headers were not found at $CUDA_HOME/include/cccl.
Run `pixi install`; the manifest should install nvidia-cuda-cccl.
EOF
    exit 1
fi

if [[ ! -e "$CUDA_HOME/lib64" ]]; then
    ln -s lib "$CUDA_HOME/lib64"
fi

link_unversioned_cuda_lib() {
    local name="$1"
    local versioned="$CUDA_HOME/lib/${name}.so.13"
    local unversioned="$CUDA_HOME/lib/${name}.so"
    if [[ -f "$versioned" && ! -e "$unversioned" ]]; then
        ln -s "$(basename "$versioned")" "$unversioned"
    fi
}

link_unversioned_cuda_lib libcudart
link_unversioned_cuda_lib libcublas
link_unversioned_cuda_lib libcublasLt

find_real_libcuda() {
    if [[ -e /usr/lib/wsl/lib/libcuda.so.1 ]]; then
        readlink -f /usr/lib/wsl/lib/libcuda.so.1
        return 0
    fi

    local ldconfig_path
    ldconfig_path="$(ldconfig -p 2>/dev/null | awk '/libcuda\.so\.1[[:space:]]/ { print $NF; exit }')"
    if [[ -n "$ldconfig_path" && -e "$ldconfig_path" ]]; then
        readlink -f "$ldconfig_path"
        return 0
    fi

    find /usr/lib /usr/local/cuda -name 'libcuda.so.1' -print -quit 2>/dev/null
}

REAL_LIBCUDA="$(find_real_libcuda)"
if [[ -z "$REAL_LIBCUDA" || ! -e "$REAL_LIBCUDA" ]]; then
    cat >&2 <<'EOF'
ERROR: Could not find the real NVIDIA libcuda.so.1 driver library.
On WSL2, check that `nvidia-smi` works inside Ubuntu.
EOF
    exit 1
fi

pywhispercpp_has_cuda_backend() {
    "$PYTHON" - <<'PY'
from pathlib import Path
import sysconfig

libs_dir = Path(sysconfig.get_paths()["purelib"]) / "pywhispercpp.libs"
raise SystemExit(0 if any(libs_dir.glob("libggml-cuda*.so*")) else 1)
PY
}

if ! pywhispercpp_has_cuda_backend; then
    CUDA_ARCH="$("$PYTHON" -c "import torch; major, minor = torch.cuda.get_device_capability(0); print(f'{major}{minor}')")"
    GPU_NAME="$("$PYTHON" -c "import torch; print(torch.cuda.get_device_name(0))")"

    export CMAKE_ARGS="${CMAKE_ARGS:-} -DGGML_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=$CUDA_ARCH"

    echo "Rebuilding pywhispercpp with CUDA for $GPU_NAME (sm_$CUDA_ARCH)..."
    "$PYTHON" -m pip install \
        --force-reinstall \
        --no-cache-dir \
        --no-deps \
        --no-binary pywhispercpp \
        "git+https://github.com/absadiki/pywhispercpp"
fi

LIBS_DIR="$SITE_PACKAGES/pywhispercpp.libs"
if [[ ! -d "$LIBS_DIR" ]]; then
    echo "ERROR: pywhispercpp shared library directory was not found: $LIBS_DIR" >&2
    exit 1
fi

patched=0
shopt -s nullglob
for lib in "$LIBS_DIR"/libcuda*.so*; do
    base="$(basename "$lib")"
    if [[ "$base" == libcudart* || "$base" == *.bundled-backup ]]; then
        continue
    fi

    if [[ -L "$lib" && "$(readlink -f "$lib")" == "$REAL_LIBCUDA" ]]; then
        continue
    fi

    if [[ -e "$lib" && ! -L "$lib" && ! -e "$lib.bundled-backup" ]]; then
        mv "$lib" "$lib.bundled-backup"
    else
        rm -f "$lib"
    fi
    ln -sf "$REAL_LIBCUDA" "$lib"
    patched=1
done
shopt -u nullglob

if [[ "$patched" -eq 1 ]]; then
    echo "Linked pywhispercpp bundled libcuda name to real driver: $REAL_LIBCUDA"
else
    echo "pywhispercpp already uses the real libcuda driver: $REAL_LIBCUDA"
fi

MODEL_PATH="$("$PYTHON" - <<'PY'
from pathlib import Path
import json

config_path = Path("config/voice_chatbot.json")
if config_path.is_file():
    config = json.loads(config_path.read_text())
else:
    config = {}

model_path = config.get("whisper_cpp_model_path")
if not model_path:
    models_dir = config.get("models_dir", "models")
    filename = config.get("whisper_cpp_model_filename", "ggml-model-fi-medium.bin")
    model_path = str(Path(models_dir) / filename)
print(model_path)
PY
)"

if [[ -f "$MODEL_PATH" ]]; then
    "$PYTHON" - <<PY
from pywhispercpp.model import Model

Model("$MODEL_PATH", n_threads=1, redirect_whispercpp_logs_to=False)
print("pywhispercpp CUDA model load verified.")
PY
else
    echo "Skipping pywhispercpp model-load verification; model not found: $MODEL_PATH"
fi
