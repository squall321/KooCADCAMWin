#!/usr/bin/env bash
# build-occt.sh
# Clones, checks out, builds, and installs OCCT 8.0.0 from source (Linux/macOS).
#
# USAGE EXAMPLES:
#   # Standard release build (defaults):
#   ./scripts/build-occt.sh
#
#   # Custom install prefix, debug build, dry-run preview:
#   ./scripts/build-occt.sh --install-prefix=/opt/occt --build-type=Debug --dry-run
#
#   # Force re-checkout with explicit job count:
#   ./scripts/build-occt.sh --tag=V8_0_0 --jobs=16 --force

set -euo pipefail

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
TAG="V8_0_0"
SOURCE_DIR="$HOME/src/OCCT"
INSTALL_PREFIX="$HOME/occt-8.0.0"
BUILD_TYPE="Release"
JOBS="$(nproc 2>/dev/null || sysctl -n hw.logicalcpu 2>/dev/null || echo 4)"
DRY_RUN=0
FORCE=0

# ---------------------------------------------------------------------------
# Usage
# ---------------------------------------------------------------------------
usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --tag=TAG                  Git tag to checkout        (default: $TAG)
  --source-dir=DIR           Local clone directory      (default: \$HOME/src/OCCT)
  --install-prefix=DIR       CMake install prefix       (default: \$HOME/occt-8.0.0)
  --build-type=TYPE          Debug|Release|RelWithDebInfo (default: Release)
  --jobs=N                   Parallel build jobs        (default: nproc)
  --dry-run                  Print commands without executing
  --force                    Force re-checkout even if tag already current
  --help                     Show this help and exit
EOF
    exit 0
}

# ---------------------------------------------------------------------------
# Argument parsing (--name=value style)
# ---------------------------------------------------------------------------
for arg in "$@"; do
    case "$arg" in
        --tag=*)            TAG="${arg#--tag=}" ;;
        --source-dir=*)     SOURCE_DIR="${arg#--source-dir=}" ;;
        --install-prefix=*) INSTALL_PREFIX="${arg#--install-prefix=}" ;;
        --build-type=*)     BUILD_TYPE="${arg#--build-type=}" ;;
        --jobs=*)           JOBS="${arg#--jobs=}" ;;
        --dry-run)          DRY_RUN=1 ;;
        --force)            FORCE=1 ;;
        --help)             usage ;;
        *)
            echo "Unknown argument: $arg" >&2
            usage
            ;;
    esac
done

# Validate BUILD_TYPE
case "$BUILD_TYPE" in
    Debug|Release|RelWithDebInfo) ;;
    *)
        echo "ERROR: --build-type must be Debug, Release, or RelWithDebInfo." >&2
        exit 1
        ;;
esac

BUILD_DIR="${SOURCE_DIR}/build"

# ---------------------------------------------------------------------------
# Helper: run or echo depending on --dry-run
# ---------------------------------------------------------------------------
run() {
    if [ "$DRY_RUN" -eq 1 ]; then
        echo "[DRY-RUN] $*"
    else
        echo "+ $*"
        "$@"
    fi
}

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
echo "=== KooCADCAM OCCT Build Script ==="
echo "Tag          : $TAG"
echo "SourceDir    : $SOURCE_DIR"
echo "InstallPrefix: $INSTALL_PREFIX"
echo "BuildType    : $BUILD_TYPE"
echo "Jobs         : $JOBS"
echo "DryRun       : $DRY_RUN"
echo "Force        : $FORCE"
echo ""

# ---------------------------------------------------------------------------
# Step 1: Clone if source dir does not exist
# ---------------------------------------------------------------------------
if [ ! -d "$SOURCE_DIR" ]; then
    echo "Cloning OCCT repository..."
    run git clone "https://github.com/Open-Cascade-SAS/OCCT.git" "$SOURCE_DIR"
else
    echo "Source directory already exists: $SOURCE_DIR"
fi

# ---------------------------------------------------------------------------
# Step 2: Fetch tags
# ---------------------------------------------------------------------------
echo "Fetching tags..."
run git -C "$SOURCE_DIR" fetch --tags

# ---------------------------------------------------------------------------
# Step 3: Checkout tag
# ---------------------------------------------------------------------------
echo "Checking out tag: $TAG ..."
if [ "$FORCE" -eq 1 ]; then
    run git -C "$SOURCE_DIR" checkout --force "$TAG"
else
    run git -C "$SOURCE_DIR" checkout "$TAG"
fi

# ---------------------------------------------------------------------------
# Step 4: CMake configure
# ---------------------------------------------------------------------------
echo "Configuring CMake..."
run cmake \
    -S "$SOURCE_DIR" \
    -B "$BUILD_DIR" \
    -G Ninja \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DBUILD_LIBRARY_TYPE=Shared \
    -DBUILD_MODULE_ApplicationFramework=ON \
    -DBUILD_MODULE_Visualization=ON \
    -DBUILD_MODULE_DataExchange=ON \
    -DBUILD_MODULE_Draw=OFF \
    -DUSE_TBB=ON \
    -DUSE_FREETYPE=ON \
    -DUSE_FREEIMAGE=ON \
    "-DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX"

# ---------------------------------------------------------------------------
# Step 5: Build
# ---------------------------------------------------------------------------
echo "Building OCCT (jobs=$JOBS)..."
run cmake --build "$BUILD_DIR" -j "$JOBS"

# ---------------------------------------------------------------------------
# Step 6: Install
# ---------------------------------------------------------------------------
echo "Installing OCCT to $INSTALL_PREFIX ..."
run cmake --install "$BUILD_DIR"

# ---------------------------------------------------------------------------
# Step 7: Post-install guidance
# ---------------------------------------------------------------------------
if [ "$DRY_RUN" -eq 0 ]; then
    echo ""
    echo "=== Build complete! ==="
    echo ""
    echo "Add to your shell profile (or run scripts/setup-env.sh):"
    echo ""
    echo "  export OCCT_INSTALL_DIR=\"$INSTALL_PREFIX\""
    echo "  export LD_LIBRARY_PATH=\"\$OCCT_INSTALL_DIR/lib:\${LD_LIBRARY_PATH:-}\""
    echo "  export PATH=\"\$OCCT_INSTALL_DIR/bin:\$PATH\""
    echo ""
    echo "Or run the project setup script for idempotent shell profile integration:"
    echo "  ./scripts/setup-env.sh --occt-install-dir=\"$INSTALL_PREFIX\""
fi
