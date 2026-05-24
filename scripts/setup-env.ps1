# setup-env.ps1
# Configures User-level environment variables for KooCADCAM development.
# Sets OCCT_INSTALL_DIR, VCPKG_ROOT, and prepends OCCT bin to User PATH.
# Changes take effect in new shells / processes (User env vars are persistent).
#
# USAGE EXAMPLES:
#   # Apply defaults:
#   .\scripts\setup-env.ps1
#
#   # Custom OCCT prefix:
#   .\scripts\setup-env.ps1 -OcctInstallDir "C:\occt-8.0.0"
#
#   # Show current values without modifying:
#   .\scripts\setup-env.ps1 -Print

param(
    [string]$OcctInstallDir = "$HOME\occt-8.0.0",
    [string]$VcpkgRoot      = "$HOME\vcpkg",
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
    $cur_path   = [Environment]::GetEnvironmentVariable("PATH",             "User")
    Write-Host "OCCT_INSTALL_DIR : $(if ($cur_occt)  { $cur_occt  } else { '(not set)' })"
    Write-Host "VCPKG_ROOT       : $(if ($cur_vcpkg) { $cur_vcpkg } else { '(not set)' })"
    Write-Host "PATH (User)      : $(if ($cur_path)  { $cur_path  } else { '(not set)' })"
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
Write-Host ""
Write-Host "These User env vars are now persistent. Open a new terminal to pick them up." -ForegroundColor Cyan
Write-Host "Current session has been updated as well."
