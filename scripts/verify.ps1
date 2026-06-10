# verify.ps1 — the ONLY sanctioned source of an "all green" claim (Windows).
#
# Performs, in order:
#   1. delete build/<flavor>           (kills stale-relink false greens)
#   2. cmake --preset windows-msvc-<flavor>
#   3. cmake --build --preset <flavor> (full build, every target)
#   4. ctest  --preset test-<flavor>   (full suite)
#
# Exit code 0  <=>  every step succeeded  <=>  "all green".
#
# Anything less (single-target builds, label-filtered ctest, incremental
# rebuilds) is fine for ITERATION but is NOT authoritative — see
# docs/breakthrough_plan.md track B8.
#
# USAGE:
#   .\scripts\verify.ps1                 # debug flavor (default)
#   .\scripts\verify.ps1 -Flavor release
#   .\scripts\verify.ps1 -KeepBuild     # NON-AUTHORITATIVE incremental mode

param(
    [ValidateSet("debug", "release")]
    [string]$Flavor = "debug",
    [switch]$KeepBuild
)

$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
Set-Location $repo

# ── 1. MSVC environment (VS Dev Shell) ─────────────────────────────────────
$vsCandidates = @(
    "D:\Program Files\Microsoft Visual Studio\2022\Community",
    "C:\Program Files\Microsoft Visual Studio\2022\Community",
    "C:\Program Files\Microsoft Visual Studio\2022\Professional",
    "C:\Program Files\Microsoft Visual Studio\2022\Enterprise"
)
$vs = $vsCandidates | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $vs) { Write-Error "Visual Studio 2022 not found in known locations"; exit 2 }
Import-Module (Join-Path $vs "Common7\Tools\Microsoft.VisualStudio.DevShell.dll")
Enter-VsDevShell -VsInstallPath $vs -DevCmdArguments "-arch=x64 -host_arch=x64" -SkipAutomaticLocation | Out-Null
Set-Location $repo   # Dev Shell may reset cwd

# ── 2. A presets-capable CMake (>= 3.25 for CMakePresets v6) ───────────────
# A stale standalone CMake (e.g. 3.20 from a Python install) may shadow PATH;
# prefer the VS-bundled one explicitly.
$cmakeExe = Join-Path $vs "Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
if (-not (Test-Path $cmakeExe)) { $cmakeExe = "cmake" }
$verLine = (& $cmakeExe --version | Select-Object -First 1)
if ($verLine -match "(\d+)\.(\d+)") {
    $major = [int]$Matches[1]; $minor = [int]$Matches[2]
    if ($major -lt 3 -or ($major -eq 3 -and $minor -lt 25)) {
        Write-Error "CMake >= 3.25 required for CMakePresets v6 (found: $verLine). VS-bundled CMake missing?"
        exit 2
    }
}
$ctestExe = Join-Path (Split-Path $cmakeExe) "ctest.exe"
if (-not (Test-Path $ctestExe)) { $ctestExe = "ctest" }
Write-Host "verify: using $verLine at $cmakeExe" -ForegroundColor Cyan

$configurePreset = "windows-msvc-$Flavor"
$buildDir        = Join-Path $repo "build\$Flavor"

# ── 3. Clean (authoritative mode) ──────────────────────────────────────────
if ($KeepBuild) {
    Write-Host "verify: -KeepBuild — INCREMENTAL run, result is NOT authoritative" -ForegroundColor Yellow
} elseif (Test-Path $buildDir) {
    Write-Host "verify: removing $buildDir (clean authoritative build)" -ForegroundColor Cyan
    Remove-Item -Recurse -Force $buildDir
}

# ── 4. Configure + full build + full test ──────────────────────────────────
& $cmakeExe --preset $configurePreset
if ($LASTEXITCODE -ne 0) { Write-Error "configure failed"; exit 1 }

# Mass-parallel linking of ~800 test exes collides in vcpkg's applocal.ps1
# post-link step: every exe copies the SAME runtime DLLs (gtest.dll, ...)
# into tests\, and concurrent copies to one destination throw "Access is
# denied".  Only clean builds link enough exes at once to hit it.  The
# retry resumes at the failed target with reduced parallelism (-j 2) so the
# remaining links serialize; a failure that survives that is real.
& $cmakeExe --build --preset $Flavor
if ($LASTEXITCODE -ne 0) {
    Write-Host "verify: build failed — retrying at -j 2 (applocal DLL-copy collision guard)" -ForegroundColor Yellow
    & $cmakeExe --build --preset $Flavor -- -j 2
    if ($LASTEXITCODE -ne 0) { Write-Error "build failed (after retry)"; exit 1 }
}

& $ctestExe --preset "test-$Flavor" --output-on-failure
if ($LASTEXITCODE -ne 0) { Write-Error "ctest failed"; exit 1 }

if ($KeepBuild) {
    Write-Host "verify: PASSED (incremental — not authoritative)" -ForegroundColor Yellow
} else {
    Write-Host "verify: ALL GREEN (authoritative: clean full build + full ctest)" -ForegroundColor Green
}
exit 0
