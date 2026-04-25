#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "$REPO_ROOT"

if [[ ! -f .gitmodules ]]; then
    exit 0
fi

git config --file .gitmodules --get-regexp '^submodule\..*\.path$' | while read -r key path; do
    name="${key#submodule.}"
    name="${name%.path}"
    url="$(git config --file .gitmodules --get "submodule.${name}.url" || true)"
    branch="$(git config --file .gitmodules --get "submodule.${name}.branch" || true)"

    if git ls-files --error-unmatch "$path" >/dev/null 2>&1; then
        git submodule update --init --recursive -- "$path"
        continue
    fi

    if [[ -d "$path/.git" || -f "$path/.git" ]]; then
        echo "Using existing git checkout not registered as a submodule yet: $path"
        git -C "$path" submodule update --init --recursive || true
        continue
    fi

    if [[ -d "$path" && -n "$(find "$path" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null)" ]]; then
        echo "Using existing directory not registered as a submodule yet: $path"
        continue
    fi

    if [[ -z "$url" ]]; then
        echo "Missing URL for submodule path '$path' in .gitmodules" >&2
        exit 1
    fi

    clone_args=(clone --recursive)
    if [[ -n "$branch" ]]; then
        clone_args+=(--branch "$branch")
    fi
    git "${clone_args[@]}" "$url" "$path"
done
