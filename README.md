# KooCADCAM

KooCADCAM is a parametric CAD design-automation tool for watch and smartphone metal front panels.
It generates B-rep geometry with OCCT 8.0.0, renders it via a Qt 6 OpenGL viewport, and exports/imports
STEP files bidirectionally. The authoritative architecture and design decisions live in
[`lat.md/index.md`](lat.md/index.md) — that knowledge graph is the **source of truth** for this project.
The frozen master specification is [`smartphone_metal_cad_project.md`](smartphone_metal_cad_project.md).

---

## Prerequisites

| Component | Windows | Linux |
|---|---|---|
| Compiler | MSVC 2022 17.8+ | GCC 12+ or Clang 14+ |
| CMake | 3.27+ | 3.27+ |
| Build backend | Ninja 1.11+ | Ninja 1.11+ |
| Qt | 6.6+ (Qt official installer) | 6.6+ (Qt official installer) |
| OCCT | 8.0.0 source-built via `scripts/build-occt.ps1` | 8.0.0 source-built via `scripts/build-occt.sh` |
| vcpkg | latest (`VCPKG_ROOT` env var set) | latest (`VCPKG_ROOT` env var set) |

---

## Quick Start

### Windows

```powershell
# 1. Clone
git clone https://github.com/your-org/KooCADCAM.git
cd KooCADCAM

# 2. Install Qt 6.6+ via the Qt online installer (https://www.qt.io/download)
#    Set Qt6_DIR or add Qt bin to PATH.

# 3. Build OCCT 8.0.0 and set the install directory
.\scripts\build-occt.ps1 -InstallDir "C:\occt800"
$env:OCCT_INSTALL_DIR = "C:\occt800"
$env:PATH = "$env:OCCT_INSTALL_DIR\bin;$env:PATH"

# 4. Configure, build, and run
cmake --preset windows-msvc-debug
cmake --build --preset debug
.\build\debug\KooCADCAM.exe
```

### Linux

```bash
# 1. Clone
git clone https://github.com/your-org/KooCADCAM.git
cd KooCADCAM

# 2. Install system dependencies (see "Linux extra packages" section below)

# 3. Install Qt 6.6+ via the Qt online installer

# 4. Build OCCT 8.0.0 and set the install directory
bash scripts/build-occt.sh --install-dir "$HOME/occt800"
export OCCT_INSTALL_DIR="$HOME/occt800"
export LD_LIBRARY_PATH="$OCCT_INSTALL_DIR/lib:$LD_LIBRARY_PATH"

# 5. Configure, build, and run
cmake --preset linux-gcc-release
cmake --build --preset linux
./build/linux-release/KooCADCAM
```

---

## OCCT 8.0.0 Build

OCCT is **not** managed by vcpkg. It is built from source using the helper scripts:

- Windows: `scripts/build-occt.ps1`
- Linux: `scripts/build-occt.sh`

Both scripts clone the OCCT repository, check out `V8_0_0`, configure with
`-DBUILD_LIBRARY_TYPE=Shared` (required for LGPL compliance), and install to the path you specify.

After building, set the environment variable:

```
OCCT_INSTALL_DIR=/path/to/occt800  (Linux)
$env:OCCT_INSTALL_DIR = "C:\occt800"  (Windows PowerShell)
```

The CMake presets automatically resolve `OpenCASCADE_DIR` as
`$OCCT_INSTALL_DIR/lib/cmake/opencascade`.

**Windows runtime path**: `%OCCT_INSTALL_DIR%\bin` must be on `PATH` so the DLLs are found at launch.

**Linux runtime path**: `$OCCT_INSTALL_DIR/lib` must be on `LD_LIBRARY_PATH`.

---

## Linux Extra Packages

### Ubuntu / Debian

```bash
sudo apt install build-essential ninja-build \
    libgl1-mesa-dev libxkbcommon-dev libxcb-cursor-dev \
    libfontconfig1-dev libfreetype-dev libtbb-dev
```

- Fedora / RHEL: `sudo dnf install gcc-c++ ninja-build mesa-libGL-devel libxkbcommon-devel freetype-devel tbb-devel`
- Arch Linux: `sudo pacman -S base-devel ninja mesa libxkbcommon freetype2 tbb`

---

## Build & Run

```bash
# Configure (choose one preset)
cmake --preset windows-msvc-debug
cmake --preset windows-msvc-release
cmake --preset linux-gcc-release

# Build
cmake --build --preset debug
cmake --build --preset release
cmake --build --preset linux
```

---

## Tests

```bash
ctest --preset test-debug
ctest --preset test-release
ctest --preset test-linux
```

---

## What is M0

M0 is the initial spike milestone. Acceptance criteria:

- The project configures and compiles on Windows (MSVC) and Linux (GCC/Clang) without errors.
- A placeholder cylinder solid (diameter 30 mm, thickness 5 mm) is created using OCCT BRep primitives.
- The cylinder is exported to a STEP file via `koo_io`.
- The exported STEP file is reimported and verified to be non-null.
- All unit tests pass under `ctest`.

---

## Project Structure

```
KooCADCAM/
├── cmake/              # Helper CMake modules (KooCompilerFlags, CheckOcctDynamic)
├── scripts/            # build-occt.ps1 / build-occt.sh
├── src/
│   ├── gui/            # koo_gui — Qt viewport, main window
│   ├── engine/         # koo_engine — parametric modelling
│   ├── io/             # koo_io — STEP/IGES import & export
│   ├── dfm/            # koo_dfm — DFM validation rules
│   ├── re/             # koo_re — reverse-engineering pipeline
│   ├── cam/            # koo_cam — CAM toolpath stubs
│   └── main.cpp
├── tests/              # GTest-based unit tests
├── licenses/           # Third-party license texts
├── .github/            # CI workflows (Windows + Linux)
├── CMakeLists.txt
├── CMakePresets.json
└── vcpkg.json
```

---

## License

Internal / proprietary. Not for redistribution.
See `licenses/` for third-party component licenses.

OCCT 8.0.0 is used under **LGPL 2.1** via dynamic linking (`BUILD_LIBRARY_TYPE=Shared`).
Qt 6 is used under **LGPL 3.0** via dynamic linking.
No modifications to OCCT or Qt sources are made in this project.

---

## Documentation

- Architecture knowledge graph: [`lat.md/index.md`](lat.md/index.md) — primary source of truth (read-only).
- Master Specification: [`smartphone_metal_cad_project.md`](smartphone_metal_cad_project.md) — frozen, do not modify.
