# Build & Dependencies

KooCADCAM의 빌드 시스템은 CMake + vcpkg 매니페스트 모드를 기반으로 한다.
**Windows + Linux 멀티플랫폼 동등 지원** — Windows는 Visual Studio 2022(MSVC),
Linux는 CMake CLI + Ninja + GCC/Clang. macOS는 1차 스코프 아님.
**이식성 목표**: 양쪽 OS 모두에서 네이티브 빌드가 통과해야 하며, CI는 두 OS를
동등한 가드로 본다 ([[process/test-strategy#CI Stage Table]]).
진짜 cross-compile (예: Linux→Windows MinGW 빌드)은 비목표 — OCCT/Qt가 OS-specific
런타임 의존성을 가지므로 양쪽 OS에서 네이티브 빌드를 표준 경로로 한다.

---

## 필수 도구 체인

| 컴포넌트 | 버전 요구사항 | 비고 |
|---|---|---|
| MSVC | 2022 17.8+ | C++17 (`/std:c++17`), CMake 생성기 |
| CMake | 3.27+ | `cmake_path()`, Presets v6 지원 |
| Qt | 6.6+ | OpenGLWidgets 모듈 필수 |
| OCCT | 8.0.0 | **소스 직접 빌드** (vcpkg 미사용), LGPL 2.1 동적 링크 — [[process/occt8-migration-cookbook#M8: OCCT 8.0.0 소스 빌드 (vcpkg 미사용)]] 참조 |
| Python | 3.10+ | CMake 스크립트 보조 (선택) |
| Ninja | 1.11+ | 권장 빌드 백엔드 |

---

## 선택적 의존성 (마일스톤별)

| 패키지 | vcpkg 이름 | 도입 시점 | 라이선스 주의 |
|---|---|---|---|
| GoogleTest | `gtest` | M1 (CI) | BSD-3 ✅ |
| spdlog | `spdlog` | M1 | MIT ✅ |
| CGAL | `cgal` + `cgal-shape-detection` | M3 (RE) | GPL/LGPL 혼용 — **동적 링크 필수** |
| OpenCAMLib | 서브모듈 수동 빌드 | M3 (CAM) | LGPL ✅ |
| nlohmann-json | `nlohmann-json` | M1 | MIT ✅ |

CGAL 라이선스 확인: `cgal-shape-detection`은 LGPL. 그러나 CGAL 일부
알고리즘은 GPL이므로 사용 전 헤더 단위 라이선스를 반드시 확인한다.
[[process/coding-standards]] 참조.

---

## `vcpkg.json` 매니페스트 (예시)

```json
{
  "$schema": "https://raw.githubusercontent.com/microsoft/vcpkg/master/scripts/vcpkg.schema.json",
  "name": "koo-cadcam",
  "version": "0.1.0",
  "dependencies": [
    "nlohmann-json",
    "spdlog",
    "gtest"
  ],
  "builtin-baseline": "2024.11.16"
}
```

**OCCT 8.0.0은 vcpkg를 사용하지 않고 소스에서 직접 빌드한다** — 사용자 정책.
빌드 절차는 Master Spec §8.2–8.3 및
[[process/occt8-migration-cookbook#M8: OCCT 8.0.0 소스 빌드 (vcpkg 미사용)]] 참조.
빌드 산출물의 install 트리(`<OCCT_INSTALL_DIR>`) 를 `OpenCASCADE_DIR`
(또는 `CMAKE_PREFIX_PATH`) 로 KooCADCAM 빌드에 전달한다.

Qt 6는 Qt 공식 인스톨러로 별도 설치한다 (Qt 자체 배포 인프라).

---

## 최상위 `CMakeLists.txt` 뼈대 (문서 전용 — 디스크에 쓰지 말 것)

```cmake
cmake_minimum_required(VERSION 3.27)
project(KooCADCAM VERSION 0.1.0 LANGUAGES CXX)

# ── C++ 표준 ───────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── 엄격 경고 ──────────────────────────────────────────────────────────
if (MSVC)
  add_compile_options(/W4 /WX /permissive- /utf-8)
else()
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# ── vcpkg 통합 (toolchain file로 활성화됨) ─────────────────────────────
# OCCT 8.0.0 — 소스에서 직접 빌드한 install 트리를 OpenCASCADE_DIR 또는
# CMAKE_PREFIX_PATH 로 지정한다. vcpkg 포트는 사용하지 않는다.
find_package(OpenCASCADE 8.0 REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Widgets OpenGLWidgets)
find_package(nlohmann_json REQUIRED)
find_package(spdlog REQUIRED)

# ── LGPL 준수: OCCT 동적 링크 강제 ────────────────────────────────────
get_target_property(_occt_type TKernel TYPE)
if (NOT _occt_type STREQUAL "SHARED_LIBRARY")
  message(FATAL_ERROR
    "OCCT must be linked dynamically (LGPL compliance). "
    "Configure OCCT with -DBUILD_LIBRARY_TYPE=Shared (not Static).")
endif()

# ── 서브디렉토리 ───────────────────────────────────────────────────────
add_subdirectory(src/gui)
add_subdirectory(src/engine)
add_subdirectory(src/io)
add_subdirectory(src/dfm)
add_subdirectory(src/re)
add_subdirectory(src/cam)

# ── 메인 실행 파일 ────────────────────────────────────────────────────
add_executable(KooCADCAM WIN32 src/main.cpp)
target_link_libraries(KooCADCAM PRIVATE
  koo_gui koo_engine koo_io koo_dfm koo_re koo_cam
  Qt6::Core Qt6::Widgets Qt6::OpenGLWidgets
  TKernel TKMath TKBRep TKGeomAlgo TKTopAlgo
  TKV3d TKAIS TKOpenGl
  TKStep TKXCAF TKXSBase
  nlohmann_json::nlohmann_json
  spdlog::spdlog
)
qt_finalize_executable(KooCADCAM)

# ── 테스트 ────────────────────────────────────────────────────────────
enable_testing()
add_subdirectory(tests)
```

---

## CMake Presets (`CMakePresets.json` 뼈대)

```json
{
  "version": 6,
  "configurePresets": [
    {
      "name": "windows-msvc-debug",
      "displayName": "Windows MSVC Debug",
      "generator": "Ninja",
      "binaryDir": "${sourceDir}/build/debug",
      "toolchainFile": "$env{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Debug",
        "VCPKG_TARGET_TRIPLET": "x64-windows",
        "OpenCASCADE_DIR": "$env{OCCT_INSTALL_DIR}/lib/cmake/opencascade"
      }
    },
    {
      "name": "windows-msvc-release",
      "displayName": "Windows MSVC Release",
      "generator": "Ninja",
      "binaryDir": "${sourceDir}/build/release",
      "toolchainFile": "$env{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Release",
        "VCPKG_TARGET_TRIPLET": "x64-windows",
        "OpenCASCADE_DIR": "$env{OCCT_INSTALL_DIR}/lib/cmake/opencascade"
      }
    },
    {
      "name": "linux-gcc-release",
      "displayName": "Linux GCC Release",
      "generator": "Ninja",
      "binaryDir": "${sourceDir}/build/linux-release",
      "toolchainFile": "$env{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Release",
        "VCPKG_TARGET_TRIPLET": "x64-linux",
        "OpenCASCADE_DIR": "$env{OCCT_INSTALL_DIR}/lib/cmake/opencascade"
      }
    }
  ],
  "buildPresets": [
    { "name": "debug",   "configurePreset": "windows-msvc-debug" },
    { "name": "release", "configurePreset": "windows-msvc-release" },
    { "name": "linux",   "configurePreset": "linux-gcc-release" }
  ]
}
```

---

## 엄격 경고 정책

| 컴파일러 | 플래그 | CI 처리 |
|---|---|---|
| MSVC | `/W4 /WX /permissive-` | 경고 = 오류 |
| GCC | `-Wall -Wextra -Wpedantic -Werror` | 경고 = 오류 |
| Clang | `-Wall -Wextra -Wpedantic -Werror` | 경고 = 오류 |

`/permissive-`는 MSVC의 표준 준수 모드 강제 (non-conforming 확장 금지).
OCCT 헤더에서 발생하는 외부 경고는 `target_compile_options`의
`SYSTEM` 인클루드 경로 지정으로 억제한다.

```cmake
# OCCT 헤더 경고 억제 (외부 라이브러리)
target_include_directories(koo_engine SYSTEM PRIVATE
  ${OpenCASCADE_INCLUDE_DIR}
)
```

---

## LGPL 준수 검증 (CI 단계)

```cmake
# CI 검증 스크립트 (tests/check_lgpl_compliance.cmake)
# OCCT가 공유 라이브러리로 링크되었는지 확인
function(check_occt_dynamic_link target)
  get_target_property(libs ${target} LINK_LIBRARIES)
  foreach(lib ${libs})
    if (lib MATCHES "^TK")
      get_target_property(type ${lib} TYPE)
      if (NOT type STREQUAL "SHARED_LIBRARY")
        message(FATAL_ERROR "LGPL violation: ${lib} is not dynamically linked")
      endif()
    endif()
  endforeach()
endfunction()
```

배포 시 OCCT `.dll`(Windows) / `.so`(Linux)를 실행 파일과 함께 패키징하고,
LGPL 2.1 라이선스 텍스트를 `licenses/OCCT_LGPL.txt`로 포함한다.

---

## Qt 배포

Qt는 자체 `windeployqt6.exe` / `linuxdeployqt` 도구로 배포한다.
vcpkg Qt 포트를 사용하지 않는 이유: Qt 공식 인스톨러가 플러그인
(OpenGL, 플랫폼 드라이버)을 올바르게 패키징하며, 라이선스 관리가 명확하다.

---

## OCCT 8.0.0 빌드 & 배포 (vcpkg 미사용)

**정책**: OCCT는 vcpkg 포트를 사용하지 않고 소스에서 직접 빌드한다.
사유:
1. OCCT 8.0.0은 메이저 변경 (Standard_Failure, EvalRep, NCollection 등) 으로 vcpkg 포트의 안정성/최신성 미확인
2. Master Spec §8.4 의 마이그레이션 함정을 빌드 옵션으로 직접 제어 필요
3. 사내 빌드 정책 일관성 — 모든 팀원이 동일한 OCCT 빌드 산출물을 사용한다고 보장
4. OCCT 패치/포팅이 필요할 경우 vcpkg 포트는 패치 적용이 번거로움

빌드 절차 요약 (상세는 [[process/occt8-migration-cookbook#M8: OCCT 8.0.0 소스 빌드 (vcpkg 미사용)]]):

```bash
git clone https://github.com/Open-Cascade-SAS/OCCT.git
cd OCCT && git checkout V8_0_0

cmake -B build -G "Ninja" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_LIBRARY_TYPE=Shared \
  -DBUILD_MODULE_ApplicationFramework=ON \
  -DBUILD_MODULE_Visualization=ON \
  -DBUILD_MODULE_DataExchange=ON \
  -DBUILD_MODULE_Draw=OFF \
  -DUSE_TBB=ON \
  -DUSE_FREETYPE=ON \
  -DUSE_FREEIMAGE=ON \
  -DCMAKE_INSTALL_PREFIX=<OCCT_INSTALL_DIR>

cmake --build build --parallel
cmake --install build
```

설치 후 `OCCT_INSTALL_DIR` 환경 변수를 export. CMakePresets 가
`OpenCASCADE_DIR=$env{OCCT_INSTALL_DIR}/lib/cmake/opencascade` 를 자동으로 잡는다.
Windows 런타임은 `<OCCT_INSTALL_DIR>/bin` 을 PATH 에 포함해야 DLL 로딩이 성공한다.

배포 시 OCCT `.dll` (Windows) / `.so` (Linux) 들과 `licenses/OCCT_LGPL.txt` 를
KooCADCAM 실행 파일과 함께 패키징한다.

---

## M0 구현 상태 (2026-05-25)

본 문서의 CMake / CMakePresets / vcpkg 골격이 실제 파일로 구현되어 있다:

- 루트: `CMakeLists.txt`, `CMakePresets.json`, `vcpkg.json`, `.gitignore`, `.gitattributes`, `.clang-format`, `.clang-tidy`, `README.md`, `LICENSE`, `licenses/`
- 헬퍼: `cmake/KooCompilerFlags.cmake`, `cmake/CheckOcctDynamic.cmake`
- OCCT 8.0.0 빌드 자동화: `scripts/build-occt.{ps1,sh}`, `scripts/setup-env.{ps1,sh}`
- 모듈 라이브러리: `src/{gui,engine,io,dfm,re,cam}/CMakeLists.txt` — 모든 `koo_*` 정적 라이브러리
- 실행 파일: `src/main.cpp` (`KooCADCAM`)
- 테스트: `tests/CMakeLists.txt` + `tests/smoke/smoke_test.cpp` + `tests/step_io/step_io_test.cpp` + `tests/tools/make_sample_step.cpp` (빌드 시점에 `tests/data/golden_cylinder.step` 자동 생성)
- CI: `.github/workflows/ci.yml` (Windows + Linux 매트릭스, OCCT 캐싱)
- IDE: `.vscode/settings.json`, `.vscode/launch.json`

**M0 시점 빌드 합격 기준 (사용자 환경에서 검증)**:
1. `scripts/build-occt.{ps1,sh}` 로 OCCT 8.0.0 빌드/설치 성공
2. `OCCT_INSTALL_DIR` 환경 변수 export 후 `cmake --preset windows-msvc-debug` (또는 linux-gcc-release) 구성 성공
3. `cmake --build --preset <name>` 빌드 성공
4. `KooCADCAM` 실행 시 빈 viewer + File 메뉴 (New Sample Cylinder / Save STEP / Open STEP / Exit) 표시
5. `ctest --preset <name>` 통과 (smoke + step_io 라운드트립)

**M0.5 후속 작업**: CI 의 OCCT 캐시 무효화 정책 정교화, Qt 6 멀티 버전 매트릭스, clang-tidy CI 게이트.

---

## 관련 문서

- [[architecture/overview]] — 모듈 분해 (src/ 경로 목록)
- [[process/occt8-migration-cookbook]] — OCCT 8.0 특유 빌드 변경점
- [[process/coding-standards]] — 경고 정책 상세, CGAL 라이선스 체크
