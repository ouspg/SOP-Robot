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
Run `pixi install` first, then rerun `pixi run install-llama-cuda`.
EOF
        exit 1
    fi
fi

PYTHON="$CONDA_PREFIX/bin/python"
if [[ ! -x "$PYTHON" ]]; then
    echo "ERROR: Python was not found at $PYTHON" >&2
    exit 1
fi

: "${LLAMA_CPP_PYTHON_VERSION:?LLAMA_CPP_PYTHON_VERSION is set by pixi.toml; run this through pixi}"

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

if "$PYTHON" - "$LLAMA_CPP_PYTHON_VERSION" >/dev/null 2>&1 <<'PY'
import sys
from importlib.metadata import version

from packaging.version import Version

import llama_cpp.llama_cpp as backend

target = Version(sys.argv[1])
installed = Version(version("llama-cpp-python"))
raise SystemExit(0 if installed >= target and backend.llama_supports_gpu_offload() else 1)
PY
then
    "$PYTHON" - <<'PY'
from importlib.metadata import version

import llama_cpp.llama_cpp as backend

print(f"llama-cpp-python {version('llama-cpp-python')} already supports CUDA GPU offload.")
info = backend.llama_print_system_info()
if isinstance(info, bytes):
    info = info.decode(errors="replace")
print(info)
PY
    exit 0
fi

CUDA_ARCH="$("$PYTHON" -c "import torch; major, minor = torch.cuda.get_device_capability(0); print(f'{major}{minor}')")"
GPU_NAME="$("$PYTHON" -c "import torch; print(torch.cuda.get_device_name(0))")"

export CMAKE_ARGS="${CMAKE_ARGS:-} -DGGML_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=$CUDA_ARCH"

echo "Rebuilding llama-cpp-python $LLAMA_CPP_PYTHON_VERSION with CUDA for $GPU_NAME (sm_$CUDA_ARCH)..."
WHEELHOUSE="$PROJECT_ROOT/.cache/llama-cpp-python-cuda/sm_${CUDA_ARCH}"
mkdir -p "$WHEELHOUSE"

CUDA_WHEEL="$(
    find "$WHEELHOUSE" -maxdepth 1 -type f \
        -name "llama_cpp_python-${LLAMA_CPP_PYTHON_VERSION}-*.whl" \
        -print -quit
)"
if [[ -z "$CUDA_WHEEL" ]]; then
    "$PYTHON" -m pip wheel \
        --no-cache-dir \
        --no-deps \
        --no-binary llama-cpp-python \
        --wheel-dir "$WHEELHOUSE" \
        "llama-cpp-python==$LLAMA_CPP_PYTHON_VERSION"
    CUDA_WHEEL="$(
        find "$WHEELHOUSE" -maxdepth 1 -type f \
            -name "llama_cpp_python-${LLAMA_CPP_PYTHON_VERSION}-*.whl" \
            -print -quit
    )"
fi

if [[ -z "$CUDA_WHEEL" ]]; then
    echo "ERROR: llama-cpp-python CUDA wheel was not created in $WHEELHOUSE" >&2
    exit 1
fi

"$PYTHON" -m pip install \
    --force-reinstall \
    --no-deps \
    "$CUDA_WHEEL"

LLAMA_LIB_DIR="$SITE_PACKAGES/llama_cpp/lib"
if [[ -d "$LLAMA_LIB_DIR" ]]; then
    for name in libcudart.so.13 libcublas.so.13 libcublasLt.so.13; do
        if [[ -f "$CUDA_HOME/lib/$name" ]]; then
            ln -sf "$CUDA_HOME/lib/$name" "$LLAMA_LIB_DIR/$name"
        fi
    done
fi

"$PYTHON" -c "import llama_cpp.llama_cpp as backend; raise SystemExit(0 if backend.llama_supports_gpu_offload() else 1)"
echo "llama-cpp-python CUDA GPU offload is available."
