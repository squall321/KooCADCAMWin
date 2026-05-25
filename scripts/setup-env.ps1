# setup-env.ps1
# Configures User-level environment variables for KooCADCAM development.
# Sets OCCT_INSTALL_DIR, VCPKG_ROOT, optionally Qt6_DIR, and prepends OCCT bin
# (and Qt bin if given) to User PATH.
# Changes take effect in new shells / processes (User env vars are persistent).
#
# USAGE EXAMPLES:
#   # Apply defaults (no Qt6_DIR change):
#   .\scripts\setup-env.ps1
#
#   # Full set-up after Qt 6 install:
#   .\scripts\setup-env.ps1 -Qt6Dir "C:\Qt\6.8.0\msvc2022_64"
#
#   # Custom OCCT prefix and Qt path:
#   .\scripts\setup-env.ps1 -OcctInstallDir "C:\occt-8.0.0" `
#                            -Qt6Dir         "D:\Qt\6.7.3\msvc2022_64"
#
#   # Show current values without modifying:
#   .\scripts\setup-env.ps1 -Print

param(
    [string]$OcctInstallDir = "$HOME\occt-8.0.0",
    [string]$VcpkgRoot      = "$HOME\vcpkg",
    [string]$Qt6Dir         = "",
    [switch]$Print
)

$ErrorActionPreference = 'Stop'

# ---------------------------------------------------------------------------
# -Print: show current state and exit
# ---------------------------------------------------------------------------
if ($Print) {
    Write-Host "=== Current KooCADCAM User Environment ===" -ForegroundColor Cyan
    $cur_occt   = [Environment]::GetEnvironmentVariable("OCCT_INSTALL_DIR", "User")
    $cur_vcpkg  = [Environment]::GetEnvironmentVariable("VCPKG_ROOT",       "User")
    $cur_qt6    = [Environment]::GetEnvironmentVariable("Qt6_DIR",          "User")
    $cur_path   = [Environment]::GetEnvironmentVariable("PATH",             "User")
    # PowerShell 5.1: ternary not available — use full if/else.
    if ($cur_occt)  { $occtShow  = $cur_occt }  else { $occtShow  = '(not set)' }
    if ($cur_vcpkg) { $vcpkgShow = $cur_vcpkg } else { $vcpkgShow = '(not set)' }
    if ($cur_qt6)   { $qt6Show   = $cur_qt6 }   else { $qt6Show   = '(not set)' }
    if ($cur_path)  { $pathShow  = $cur_path }  else { $pathShow  = '(not set)' }
    Write-Host "OCCT_INSTALL_DIR : $occtShow"
    Write-Host "VCPKG_ROOT       : $vcpkgShow"
    Write-Host "Qt6_DIR          : $qt6Show"
    Write-Host "PATH (User)      : $pathShow"
    exit 0
}

# ---------------------------------------------------------------------------
# Set OCCT_INSTALL_DIR
# ---------------------------------------------------------------------------
Write-Host "Setting OCCT_INSTALL_DIR = $OcctInstallDir" -ForegroundColor Yellow
[Environment]::SetEnvironmentVariable("OCCT_INSTALL_DIR", $OcctInstallDir, "User")

# ---------------------------------------------------------------------------
# Set VCPKG_ROOT
# ---------------------------------------------------------------------------
Write-Host "Setting VCPKG_ROOT = $VcpkgRoot" -ForegroundColor Yellow
[Environment]::SetEnvironmentVariable("VCPKG_ROOT", $VcpkgRoot, "User")

# ---------------------------------------------------------------------------
# Prepend OCCT bin to User PATH (without duplicating)
# ---------------------------------------------------------------------------
# OCCT install layout varies: Windows-native (win64\vc14\bin) vs Generic (bin)
# depending on -DINSTALL_DIR_LAYOUT choice. Auto-detect by probing for TKernel.dll.
$occtBin = $null
$candidates = @(
    (Join-Path $OcctInstallDir "win64\vc14\bin"),
    (Join-Path $OcctInstallDir "win64\vc143\bin"),
    (Join-Path $OcctInstallDir "win64\vc142\bin"),
    (Join-Path $OcctInstallDir "bin")
)
foreach ($cand in $candidates) {
    if (Test-Path (Join-Path $cand "TKernel.dll")) { $occtBin = $cand; break }
}
if (-not $occtBin) {
    Write-Warning "Could not detect OCCT bin under $OcctInstallDir (looked for TKernel.dll). Falling back to \bin."
    $occtBin = Join-Path $OcctInstallDir "bin"
} else {
    Write-Host "Detected OCCT bin: $occtBin" -ForegroundColor Green
}
$userPath  = [Environment]::GetEnvironmentVariable("PATH", "User")
if (-not $userPath) { $userPath = "" }

# Split on semicolon, trim whitespace, filter empties
$pathParts = $userPath -split ';' | ForEach-Object { $_.Trim() } | Where-Object { $_ -ne "" }

if ($pathParts -contains $occtBin) {
    Write-Host "PATH already contains: $occtBin — skipping." -ForegroundColor DarkYellow
} else {
    Write-Host "Prepending to User PATH: $occtBin" -ForegroundColor Yellow
    $newPath = ($occtBin + ";" + ($pathParts -join ";")).TrimEnd(";")
    [Environment]::SetEnvironmentVariable("PATH", $newPath, "User")
}

# ---------------------------------------------------------------------------
# Qt6_DIR (optional) — set persistent Qt 6 CMake package path + Qt bin on PATH
# ---------------------------------------------------------------------------
if ($Qt6Dir -ne "") {
    if (-not (Test-Path $Qt6Dir)) {
        Write-Warning "Qt6Dir '$Qt6Dir' does not exist; setting anyway (intended for later install)."
    }
    Write-Host "Setting Qt6_DIR = $Qt6Dir" -ForegroundColor Yellow
    [Environment]::SetEnvironmentVariable("Qt6_DIR", $Qt6Dir, "User")
    $env:Qt6_DIR = $Qt6Dir
    # Prepend Qt's bin to User PATH so Qt6 DLLs and windeployqt6 are visible.
    $qtBin = Join-Path $Qt6Dir "bin"
    if (Test-Path $qtBin) {
        $userPath2  = [Environment]::GetEnvironmentVariable("PATH", "User")
        $pathParts2 = $userPath2 -split ';' | ForEach-Object { $_.Trim() } | Where-Object { $_ -ne "" }
        if (-not ($pathParts2 -contains $qtBin)) {
            Write-Host "Prepending to User PATH: $qtBin" -ForegroundColor Yellow
            $newPath2 = ($qtBin + ";" + ($pathParts2 -join ";")).TrimEnd(";")
            [Environment]::SetEnvironmentVariable("PATH", $newPath2, "User")
        }
        if ($env:PATH -notlike "*$qtBin*") { $env:PATH = "$qtBin;$env:PATH" }
    } else {
        Write-Warning "Qt6 bin not found at $qtBin — DLL runtime PATH not updated."
    }
}

# ---------------------------------------------------------------------------
# Also apply to current session for immediate use
# ---------------------------------------------------------------------------
$env:OCCT_INSTALL_DIR = $OcctInstallDir
$env:VCPKG_ROOT       = $VcpkgRoot
if ($env:PATH -notlike "*$occtBin*") {
    $env:PATH = "$occtBin;$env:PATH"
}

# ---------------------------------------------------------------------------
# Print final state
# ---------------------------------------------------------------------------
Write-Host ""
Write-Host "=== Environment configured ===" -ForegroundColor Green
Write-Host "OCCT_INSTALL_DIR : $([Environment]::GetEnvironmentVariable('OCCT_INSTALL_DIR', 'User'))"
Write-Host "VCPKG_ROOT       : $([Environment]::GetEnvironmentVariable('VCPKG_ROOT',       'User'))"
$qt6Now = [Environment]::GetEnvironmentVariable('Qt6_DIR', 'User')
if ($qt6Now) { Write-Host "Qt6_DIR          : $qt6Now" }
Write-Host ""
Write-Host "These User env vars are now persistent. Open a new terminal to pick them up." -ForegroundColor Cyan
Write-Host "Current session has been updated as well."
