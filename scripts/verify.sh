#!/usr/bin/env bash
# verify.sh — the ONLY sanctioned source of an "all green" claim (Linux).
#
#   1. delete build/linux-release       (kills stale-relink false greens)
#   2. cmake --preset linux-gcc-release
#   3. cmake --build --preset linux     (full build, every target)
#   4. ctest  --preset test-linux       (full suite)
#
# Exit 0  <=>  every step succeeded  <=>  "all green".
# Incremental/label-filtered runs are for iteration only — NOT authoritative.
# See docs/breakthrough_plan.md track B8.
#
# USAGE:
#   bash scripts/verify.sh                # authoritative full gate
#   bash scripts/verify.sh --keep-build   # NON-AUTHORITATIVE incremental

set -euo pipefail
cd "$(dirname "$0")/.."

KEEP_BUILD=0
[[ "${1:-}" == "--keep-build" ]] && KEEP_BUILD=1

# CMakePresets v6 needs CMake >= 3.25.
ver=$(cmake --version | head -1 | grep -oE '[0-9]+\.[0-9]+' | head -1)
major=${ver%%.*}; minor=${ver##*.}
if (( major < 3 || (major == 3 && minor < 25) )); then
    echo "ERROR: CMake >= 3.25 required for CMakePresets v6 (found $ver)" >&2
    exit 2
fi

if (( KEEP_BUILD )); then
    echo "verify: --keep-build — INCREMENTAL run, result is NOT authoritative"
else
    echo "verify: removing build/linux-release (clean authoritative build)"
    rm -rf build/linux-release
fi

cmake --preset linux-gcc-release
# Mass-parallel linking of ~800 test exes can collide in post-link runtime
# copy steps; retry at reduced parallelism (see verify.ps1 for the Windows
# applocal.ps1 DLL-copy collision this guards against).
cmake --build --preset linux || cmake --build --preset linux -- -j 2
ctest --preset test-linux --output-on-failure

if (( KEEP_BUILD )); then
    echo "verify: PASSED (incremental — not authoritative)"
else
    echo "verify: ALL GREEN (authoritative: clean full build + full ctest)"
fi
