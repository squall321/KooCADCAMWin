# build-occt.ps1
# Clones, checks out, builds, and installs OCCT 8.0.0 from source.
#
# USAGE EXAMPLES:
#   # Standard release build (default settings):
#   .\build-occt.ps1
#
#   # Custom install prefix, debug build, dry-run preview:
#   .\build-occt.ps1 -InstallPrefix "C:\occt" -BuildType Debug -DryRun
#
#   # Force re-checkout with 16 parallel jobs:
#   .\build-occt.ps1 -Tag V8_0_0 -Jobs 16 -Force

param(
    [string]$Tag            = "V8_0_0",
    [string]$SourceDir      = "$HOME\src\OCCT",
    [string]$InstallPrefix  = "$HOME\occt-8.0.0",
    [ValidateSet("Debug", "Release", "RelWithDebInfo")]
    [string]$BuildType      = "Release",
    [int]$Jobs              = [Environment]::ProcessorCount,
    [switch]$DryRun,
    [switch]$Force
)

$ErrorActionPreference = 'Stop'

# ---------------------------------------------------------------------------
# Helper: run or echo a command depending on -DryRun
# ---------------------------------------------------------------------------
function Invoke-OrEcho {
    param([string[]]$Cmd)
    $line = $Cmd -join ' '
    if ($DryRun) {
        Write-Host "[DRY-RUN] $line" -ForegroundColor Cyan
    } else {
        Write-Host "+ $line" -ForegroundColor DarkGray
        & $Cmd[0] $Cmd[1..($Cmd.Length - 1)]
        if ($LASTEXITCODE -ne 0) {
            Write-Error "Command failed with exit code ${LASTEXITCODE}: $line"
            exit 1
        }
    }
}

Write-Host "=== KooCADCAM OCCT Build Script ===" -ForegroundColor Green
Write-Host "Tag          : $Tag"
Write-Host "SourceDir    : $SourceDir"
Write-Host "InstallPrefix: $InstallPrefix"
Write-Host "BuildType    : $BuildType"
Write-Host "Jobs         : $Jobs"
Write-Host "DryRun       : $($DryRun.IsPresent)"
Write-Host "Force        : $($Force.IsPresent)"
Write-Host ""

# ---------------------------------------------------------------------------
# Step 1: Clone if source dir does not exist
# ---------------------------------------------------------------------------
if (-not (Test-Path $SourceDir)) {
    Write-Host "Cloning OCCT repository..." -ForegroundColor Yellow
    Invoke-OrEcho @(
        "git", "clone",
        "https://github.com/Open-Cascade-SAS/OCCT.git",
        $SourceDir
    )
} else {
    Write-Host "Source directory already exists: $SourceDir" -ForegroundColor DarkYellow
}

# ---------------------------------------------------------------------------
# Step 2: Fetch tags
# ---------------------------------------------------------------------------
Write-Host "Fetching tags..." -ForegroundColor Yellow
Invoke-OrEcho @("git", "-C", $SourceDir, "fetch", "--tags")

# ---------------------------------------------------------------------------
# Step 3: Checkout tag
# ---------------------------------------------------------------------------
Write-Host "Checking out tag: $Tag ..." -ForegroundColor Yellow
if ($Force) {
    Invoke-OrEcho @("git", "-C", $SourceDir, "checkout", "--force", $Tag)
} else {
    Invoke-OrEcho @("git", "-C", $SourceDir, "checkout", $Tag)
}

# ---------------------------------------------------------------------------
# Step 4: CMake configure
# ---------------------------------------------------------------------------
$BuildDir = Join-Path $SourceDir "build"
Write-Host "Configuring CMake..." -ForegroundColor Yellow
Invoke-OrEcho @(
    "cmake",
    "-S", $SourceDir,
    "-B", $BuildDir,
    "-G", "Ninja",
    "-DCMAKE_BUILD_TYPE=$BuildType",
    "-DBUILD_LIBRARY_TYPE=Shared",
    "-DBUILD_MODULE_ApplicationFramework=ON",
    "-DBUILD_MODULE_Visualization=ON",
    "-DBUILD_MODULE_DataExchange=ON",
    "-DBUILD_MODULE_Draw=OFF",
    # M0: third-party deps disabled — install them and flip to ON when needed.
    # USE_TBB=ON requires TBB install; USE_FREETYPE/FREEIMAGE need system libs.
    "-DUSE_TBB=OFF",
    "-DUSE_FREETYPE=OFF",
    "-DUSE_FREEIMAGE=OFF",
    "-DCMAKE_INSTALL_PREFIX=$InstallPrefix"
)

# ---------------------------------------------------------------------------
# Step 5: Build
# ---------------------------------------------------------------------------
Write-Host "Building OCCT (jobs=$Jobs)..." -ForegroundColor Yellow
Invoke-OrEcho @(
    "cmake",
    "--build", $BuildDir,
    "--parallel", "$Jobs"
)

# ---------------------------------------------------------------------------
# Step 6: Install
# ---------------------------------------------------------------------------
Write-Host "Installing OCCT to $InstallPrefix ..." -ForegroundColor Yellow
Invoke-OrEcho @("cmake", "--install", $BuildDir)

# ---------------------------------------------------------------------------
# Step 7: Post-install instructions
# ---------------------------------------------------------------------------
if (-not $DryRun) {
    Write-Host ""
    Write-Host "=== Build complete! ===" -ForegroundColor Green
    Write-Host ""
    Write-Host "Run the following to persist environment variables for this session:" -ForegroundColor Cyan
    Write-Host "  `$env:OCCT_INSTALL_DIR = `"$InstallPrefix`""
    Write-Host "  `$env:PATH = `"$InstallPrefix\bin;`$env:PATH`""
    Write-Host ""
    Write-Host "Or run the project environment setup script for permanent User env vars:" -ForegroundColor Cyan
    Write-Host "  .\scripts\setup-env.ps1 -OcctInstallDir `"$InstallPrefix`""
    Write-Host ""
    Write-Host "Verify OCCT DLLs are on PATH:" -ForegroundColor Cyan
    Write-Host "  Get-ChildItem `"$InstallPrefix\bin\TK*.dll`""
}
