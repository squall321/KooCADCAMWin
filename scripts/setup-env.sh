#!/usr/bin/env bash
# setup-env.sh
# Idempotently writes KooCADCAM export lines to ~/.bashrc or ~/.zshrc,
# guarded by sentinel comments so re-running never duplicates entries.
#
# USAGE EXAMPLES:
#   # Apply defaults (auto-detects bash/zsh):
#   ./scripts/setup-env.sh
#
#   # Custom OCCT prefix:
#   ./scripts/setup-env.sh --occt-install-dir=/opt/occt-8.0.0
#
#   # Print current values without modifying shell profile:
#   ./scripts/setup-env.sh --print
#
#   # Show help:
#   ./scripts/setup-env.sh --help

set -euo pipefail

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
OCCT_INSTALL_DIR_DEFAULT="$HOME/occt-8.0.0"
VCPKG_ROOT_DEFAULT="$HOME/vcpkg"
OCCT_INSTALL_DIR_PARAM=""
VCPKG_ROOT_PARAM=""
PRINT_ONLY=0

SENTINEL_START="# KOOCADCAM env START"
SENTINEL_END="# KOOCADCAM env END"

# ---------------------------------------------------------------------------
# Usage
# ---------------------------------------------------------------------------
usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --occt-install-dir=DIR   OCCT install prefix   (default: \$HOME/occt-8.0.0)
  --vcpkg-root=DIR         vcpkg root directory  (default: \$HOME/vcpkg)
  --print                  Show current exported values and exit
  --help                   Show this help and exit
EOF
    exit 0
}

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
for arg in "$@"; do
    case "$arg" in
        --occt-install-dir=*) OCCT_INSTALL_DIR_PARAM="${arg#--occt-install-dir=}" ;;
        --vcpkg-root=*)       VCPKG_ROOT_PARAM="${arg#--vcpkg-root=}" ;;
        --print)              PRINT_ONLY=1 ;;
        --help)               usage ;;
        *)
            echo "Unknown argument: $arg" >&2
            usage
            ;;
    esac
done

OCCT_DIR="${OCCT_INSTALL_DIR_PARAM:-$OCCT_INSTALL_DIR_DEFAULT}"
VCPKG_DIR="${VCPKG_ROOT_PARAM:-$VCPKG_ROOT_DEFAULT}"

# ---------------------------------------------------------------------------
# --print: show current environment values
# ---------------------------------------------------------------------------
if [ "$PRINT_ONLY" -eq 1 ]; then
    echo "=== Current KooCADCAM Environment ==="
    echo "OCCT_INSTALL_DIR : ${OCCT_INSTALL_DIR:-'(not set)'}"
    echo "VCPKG_ROOT       : ${VCPKG_ROOT:-'(not set)'}"
    echo "LD_LIBRARY_PATH  : ${LD_LIBRARY_PATH:-'(not set)'}"
    echo "PATH snippet     : $(echo "$PATH" | tr ':' '\n' | grep -i occt || echo '(occt not found in PATH)')"
    exit 0
fi

# ---------------------------------------------------------------------------
# Detect target shell profile
# ---------------------------------------------------------------------------
detect_profile() {
    # Prefer the running shell; fall back to existence checks.
    local shell_name
    shell_name="$(basename "${SHELL:-bash}")"

    case "$shell_name" in
        zsh)
            echo "$HOME/.zshrc"
            ;;
        bash)
            echo "$HOME/.bashrc"
            ;;
        *)
            # Fallback: pick whichever exists, prefer .bashrc
            if [ -f "$HOME/.bashrc" ]; then
                echo "$HOME/.bashrc"
            elif [ -f "$HOME/.zshrc" ]; then
                echo "$HOME/.zshrc"
            else
                echo "$HOME/.bashrc"   # will be created
            fi
            ;;
    esac
}

PROFILE="$(detect_profile)"
echo "Target profile: $PROFILE"

# ---------------------------------------------------------------------------
# Build the env block to inject
# ---------------------------------------------------------------------------
ENV_BLOCK="${SENTINEL_START}
export OCCT_INSTALL_DIR=\"\${OCCT_INSTALL_DIR:-${OCCT_DIR}}\"
export VCPKG_ROOT=\"\${VCPKG_ROOT:-${VCPKG_DIR}}\"
export LD_LIBRARY_PATH=\"\$OCCT_INSTALL_DIR/lib:\${LD_LIBRARY_PATH:-}\"
export PATH=\"\$OCCT_INSTALL_DIR/bin:\$PATH\"
${SENTINEL_END}"

# ---------------------------------------------------------------------------
# Idempotent injection: replace between sentinels, or append if absent
# ---------------------------------------------------------------------------
if [ -f "$PROFILE" ] && grep -qF "$SENTINEL_START" "$PROFILE"; then
    # Replace the block between (and including) sentinels.
    # Use awk for portability across Linux and macOS (no GNU sed required).
    echo "Replacing existing KooCADCAM env block in $PROFILE ..."
    awk -v block="$ENV_BLOCK" '
        /# KOOCADCAM env START/ { in_block=1; print block; next }
        /# KOOCADCAM env END/   { in_block=0; next }
        !in_block               { print }
    ' "$PROFILE" > "${PROFILE}.koocadcam_tmp"
    mv "${PROFILE}.koocadcam_tmp" "$PROFILE"
else
    # Append a blank line separator then the block.
    echo "Appending KooCADCAM env block to $PROFILE ..."
    {
        echo ""
        echo "$ENV_BLOCK"
    } >> "$PROFILE"
fi

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
echo ""
echo "=== setup-env.sh complete ==="
echo "Profile updated : $PROFILE"
echo "OCCT_INSTALL_DIR: $OCCT_DIR"
echo "VCPKG_ROOT      : $VCPKG_DIR"
echo ""
echo "Reload your shell or run:"
echo "  source $PROFILE"
