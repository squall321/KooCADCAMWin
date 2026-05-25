# OCCT 7.x → 8.0 Migration Cookbook

OCCT 7.x 기반 코드를 8.0.0으로 올릴 때 반드시 처리해야 할 변경사항을 정리한다.
마이그레이션 전 체크리스트 및 진행 트래커 템플릿을 포함한다.
코딩 규약과의 연계는 [[process/coding-standards]] 참고.

---

## §8.4 포팅 — 마이그레이션 주의사항 (Master Spec §8.4 원문 이식)

> 출처: `smartphone_metal_cad_project.md` §8.4 "마이그레이션 주의사항 (7.x → 8.0)"

### 1. C++17 모드 활성화 필수
```cmake
# CMakeLists.txt
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

### 2. Standard_Failure 예외 처리 변경
```cpp
// Before (7.x)
Standard_Failure::Raise("error");
Standard_Failure::Instance();

// After (8.0)
throw Standard_Failure("error");
// Instance() 제거 — 더 이상 존재하지 않음
```

### 3. 수학 함수 치환
OCCT 공식 마이그레이션 스크립트: `adm/scripts/migration_800/replace_typedefs.py`

### 4. NCollection_Array1/2::Assign 동작 변경
런타임 테스트 필수 (아래 상세 참고).

### 5. Standard_UNUSED → `[[maybe_unused]]`
수동 마이그레이션 필요 (위치 규칙 엄격화 — 아래 상세 참고).

### 6. 소스 디렉토리 구조 변경
`src/Module/Toolkit/Package/File` 구조. include 경로 업데이트 필요.

---

## 항목별 실전 가이드

---

### M1: C++17 필수화

**영향 범위**: CMakeLists.txt, 컴파일러 플래그

**VS regex 검색** (구버전 CMake 설정 탐지):
```
CXX_STANDARD\s+14
CXX_STANDARD\s+11
```

**조치**:
```cmake
# 모든 타겟에 일괄 적용
set_property(TARGET KooCADCAM PROPERTY CXX_STANDARD 17)
set_property(TARGET KooCADCAM PROPERTY CXX_STANDARD_REQUIRED ON)
```

---

### M2: Standard_Failure — throw 전환

**영향 범위**: `Raise(` 호출 전체, `Instance()` 호출 전체

**sed 패턴** (Linux/macOS 정방향 참조 필요):
```bash
# Raise( → throw Standard_Failure(  (인자가 있는 경우)
sed -i 's/Standard_Failure::Raise(\(.*\))/throw Standard_Failure(\1)/g' **/*.cxx

# 인자 없는 단순 Raise
sed -i 's/Standard_Failure::Raise()/throw Standard_Failure("unspecified")/g' **/*.cxx
```

**VS 정규식** (Find & Replace):
```
검색:  Standard_Failure::Raise\((.+)\)
치환:  throw Standard_Failure($1)
```

**8.0의 핵심 변화**: `Standard_Failure`가 이제 `std::exception`에서 파생된다.
따라서 API 경계 캐처를 `const std::exception&` 단일 핸들러로 통합 가능하다.

```cpp
// 권장 캐처 패턴 (Standard_Failure + std::exception 동시 처리)
try {
    doOcctOperation();
} catch (const std::exception& ex) {
    // ex.what() 으로 메시지 접근 가능
    spdlog::error("OCCT failure: {}", ex.what());
}
```

> **함정**: 7.x 코드에 `catch (Standard_Failure& e)` 가 있으면
> 8.0에서도 컴파일되지만, 상위의 `catch (std::exception&)` 가 먼저
> 잡을 수 있다. 상속 순서를 확인하고 캐처 순서를 정리할 것.

---

### M3: 수학 함수 std::* 전환

**전체 치환 표**

| 7.x (deprecated) | 8.0 표준 | sed/VS 검색 패턴 |
|---|---|---|
| `ACos(` | `std::acos(` | `\bACos\(` |
| `ASin(` | `std::asin(` | `\bASin\(` |
| `ATan(` | `std::atan(` | `\bATan\(` |
| `ATan2(` | `std::atan2(` | `\bATan2\(` |
| `Cos(` | `std::cos(` | `\bCos\(` |
| `Sin(` | `std::sin(` | `\bSin\(` |
| `Sqrt(` | `std::sqrt(` | `\bSqrt\(` |
| `Abs(` | `std::abs(` | `\bAbs\(` |
| `Pow(` | `std::pow(` | `\bPow\(` |
| `Exp(` | `std::exp(` | `\bExp\(` |
| `Log(` | `std::log(` | `\bLog\(` |

**일괄 sed 스크립트**:
```bash
#!/usr/bin/env bash
# migrate_math.sh — OCCT 7.x 수학 함수 일괄 치환
FILES=$(find src -name "*.cxx" -o -name "*.hxx" -o -name "*.cpp" -o -name "*.hpp")
for F in $FILES; do
    sed -i \
        -e 's/\bACos(/std::acos(/g' \
        -e 's/\bASin(/std::asin(/g' \
        -e 's/\bATan2(/std::atan2(/g' \
        -e 's/\bATan(/std::atan(/g' \
        -e 's/\bCos(/std::cos(/g' \
        -e 's/\bSin(/std::sin(/g' \
        -e 's/\bSqrt(/std::sqrt(/g' \
        -e 's/\bAbs(/std::abs(/g' \
        -e 's/\bPow(/std::pow(/g' \
        -e 's/\bExp(/std::exp(/g' \
        -e 's/\bLog(/std::log(/g' \
        "$F"
done
```

> **함정 — 네임스페이스 충돌**: `using namespace std;` 가 전역에 있는 레거시
> 파일에서 `std::abs` 가 `abs` 와 중복될 수 있다. 치환 후 빌드 오류를
> 확인하고, `using namespace std;` 를 제거하는 것을 권장한다.

---

### M4: Standard_UNUSED → `[[maybe_unused]]`

**변경 전후**:
```cpp
// 7.x (매크로)
void foo(Standard_UNUSED int bar) { }

// 8.0 (C++17 attribute)
void foo([[maybe_unused]] int bar) { }
```

**핵심 주의사항 — 위치 규칙**:

C++17 attribute는 **타입 앞(선행 한정자)이 아니라 변수 이름 앞**에 위치해야 한다.

```cpp
// ❌ 잘못된 위치 (매크로를 그대로 복사한 경우)
[[maybe_unused]] int x = compute();  // 이건 사실 OK — 선언에는 OK

// ⚠️  파라미터에서 주의:
void foo(int [[maybe_unused]] bar) { }   // ❌ 컴파일 오류
void foo([[maybe_unused]] int bar) { }   // ✅ 올바른 위치

// 람다 캡처에서도 주의:
auto lam = [](auto&& [[maybe_unused]] x) { };  // ❌
auto lam = []([[maybe_unused]] auto&& x) { }; // ✅
```

**VS regex 검색** (마이그레이션 잔재 탐지):
```
Standard_UNUSED
```

---

### M5: NCollection_Array1/2::Assign — 사일런트 Breaking Change

#### 변경 내용

7.x에서 `Assign(src)` 는 destination 배열이 자신의 범위(lower/upper bound)를
유지했다. 8.0에서는 **destination이 source의 범위를 채택**한다.

```
7.x: dst.Assign(src)
  → dst 인덱스 범위: 변경 없음 (dst.Lower() 유지)
  → dst 원소: src 원소로 교체

8.0: dst.Assign(src)
  → dst 인덱스 범위: src.Lower() ~ src.Upper() 로 변경 ← Breaking!
  → dst 원소: src 원소로 교체
```

이 변경은 **컴파일 오류 없이 통과**하며 런타임에만 드러난다.

#### 영향 패턴 — 위험한 코드

```cpp
NCollection_Array1<gp_Pnt> dst(1, 10);  // lower=1, upper=10
NCollection_Array1<gp_Pnt> src(0, 5);   // lower=0, upper=5

dst.Assign(src);

// 7.x: dst.Lower() == 1, dst.Upper() == 10 (범위 유지)
// 8.0: dst.Lower() == 0, dst.Upper() == 5  (범위 변경! ← 버그 가능성)

// 이후 코드가 dst(7) 를 참조하면 8.0에서 범위 초과 예외 발생
```

#### 런타임 테스트 픽스처

```cpp
TEST(NCollectionAssignMigration, RangeAdoptsSourceAfterAssign) {
    NCollection_Array1<int> dst(1, 10);
    NCollection_Array1<int> src(0, 5);
    for (int i = src.Lower(); i <= src.Upper(); ++i)
        src(i) = i * 2;

    dst.Assign(src);

    // 8.0 기대 동작: 범위가 src를 따름
    EXPECT_EQ(dst.Lower(), src.Lower());   // 0
    EXPECT_EQ(dst.Upper(), src.Upper());   // 5
    EXPECT_EQ(dst(0), 0);
    EXPECT_EQ(dst(5), 10);
}
```

#### 점검 grep 패턴

```bash
grep -rn "\.Assign(" src/ --include="*.cpp" --include="*.hpp" --include="*.cxx"
```

발견 위치마다 `Assign` 호출 전후 인덱스 범위 사용을 수동 검토한다.

---

### M6: EvalRep 시스템 — 마이그레이션 필요 여부 판단

#### 개요

8.0에서 `Geom_Curve` / `Geom_Surface` 의 가상 `D0/D1/D2/D3` 메서드가
EvalRep descriptor 시스템으로 교체되었다.

#### 마이그레이션이 필요한 경우

다음 중 하나라도 해당하면 EvalRep를 검토해야 한다:

```cpp
// 해당하면 마이그레이션 필요
class MyCustomCurve : public Geom_Curve {    // ← Geom_Curve 직접 서브클래스
    void D0(Standard_Real U, gp_Pnt& P) const override { ... }
    void D1(Standard_Real U, gp_Pnt& P, gp_Vec& V1) const override { ... }
};
```

#### 마이그레이션이 불필요한 경우

KooCADCAM의 대부분 코드는 **OCCT 고수준 API를 소비**하므로 해당 없음:

```cpp
// 이런 코드는 EvalRep 영향 없음 — 그냥 컴파일된다
BRepAdaptor_Curve adaptor(edge);
gp_Pnt pt = adaptor.Value(0.5);   // 내부적으로 EvalRep 사용, 외부 API 동일

Handle(Geom_BSplineCurve) bsp = GeomAPI_PointsToBSpline(pts).Curve();
```

#### 확인 명령어

```bash
# Geom_Curve / Geom_Surface 서브클래스 탐지
grep -rn "public Geom_Curve\|public Geom_Surface" src/ \
     --include="*.hpp" --include="*.hxx"
```

결과가 없으면 EvalRep 마이그레이션 불필요.

---

### M7: 소스 트리 재구성 — include 경로

#### 새 구조

```
src/
  BRep/
    TKBRep/
      BRepBuilderAPI/
        BRepBuilderAPI_MakeBox.hxx
```

#### include 경로 영향

OCCT는 기존 단일 평탄 include 서페이스를 **레거시 호환 레이어로 유지**하므로
대부분의 경우 아래 코드는 8.0에서도 그대로 컴파일된다:

```cpp
#include <BRepBuilderAPI_MakeBox.hxx>   // 7.x 방식 — 8.0에서 대부분 호환
```

#### 직접 빌드 install 트리 (vcpkg 미사용)

KooCADCAM은 OCCT를 소스에서 직접 빌드한다
([[architecture/build-and-deps#OCCT 8.0.0 빌드 & 배포 (vcpkg 미사용)]]).
설치 트리의 표준 형태는 다음과 같다:

```
<OCCT_INSTALL_DIR>/
├── include/opencascade/   # 플랫 헤더 — #include <BRepBuilderAPI_MakeBox.hxx> 그대로 사용
├── lib/                   # .lib (Windows) / .a (Linux) 임포트 라이브러리
├── lib/cmake/opencascade/ # CMake 패키지 (find_package(OpenCASCADE) 가 여기를 잡음)
├── bin/                   # .dll (Windows) — 런타임 PATH 필요
└── share/opencascade/     # 리소스 (Standard, UnitsAPI, ShapeProcess 등)
```

CMakeLists.txt 에서 경로 검증:

```cmake
get_target_property(OCCT_INCS OpenCASCADE::FoundationClasses
    INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS "OCCT include dirs: ${OCCT_INCS}")
# 정상: <OCCT_INSTALL_DIR>/include/opencascade
```

Windows 런타임 환경에서는 `<OCCT_INSTALL_DIR>/bin` 이 PATH 에 포함되어야
OCCT DLL 들이 로드된다. 배포 시 별도 패키징
([[architecture/build-and-deps#OCCT 8.0.0 빌드 & 배포 (vcpkg 미사용)]]).

#### 빌드 파손 탐지 레시피

```cmake
# 모든 OCCT 헤더 파일을 명시적으로 include 하는 더미 타겟으로 탐지
add_executable(occt_include_probe tools/occt_include_probe.cpp)
target_link_libraries(occt_include_probe PRIVATE OCCT::TKBRep OCCT::TKMath)
# 빌드 실패 시 경로 문제
```

---

### M8: OCCT 8.0.0 소스 빌드 (vcpkg 미사용)

> **정책**: OCCT는 vcpkg 포트를 사용하지 않고 [Open-Cascade-SAS/OCCT](https://github.com/Open-Cascade-SAS/OCCT)
> 의 `V8_0_0` 태그에서 직접 빌드한다. 사유는
> [[architecture/build-and-deps#OCCT 8.0.0 빌드 & 배포 (vcpkg 미사용)]] 참조.

#### 사전 요구사항 (Master Spec §8.1 기반)

| 항목 | 요구사항 |
|---|---|
| C++ 컴파일러 | MSVC 2019+, GCC 9+, Clang 10+ (C++17 지원) |
| CMake | 3.14+ (KooCADCAM 자체는 3.27+) |
| OpenGL | 3.2+ |
| 선택 의존성 | FreeType, FreeImage, TBB, VTK |

#### 소스 취득 & 체크아웃

```bash
git clone https://github.com/Open-Cascade-SAS/OCCT.git
cd OCCT
git checkout V8_0_0
```

#### CMake 구성 (KooCADCAM 권장 옵션)

```bash
mkdir build && cd build

cmake .. \
  -G "Ninja" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_LIBRARY_TYPE=Shared \
  -DBUILD_MODULE_ApplicationFramework=ON \
  -DBUILD_MODULE_Visualization=ON \
  -DBUILD_MODULE_DataExchange=ON \
  -DBUILD_MODULE_Draw=OFF \
  -DUSE_TBB=ON \
  -DUSE_FREETYPE=ON \
  -DUSE_FREEIMAGE=ON \
  -DCMAKE_INSTALL_PREFIX=C:/dev/occt-8.0.0

cmake --build . --parallel
cmake --install .
```

> **중요**: `BUILD_LIBRARY_TYPE=Shared` 필수 (LGPL 2.1 동적 링크 준수 — 정적 빌드는
> 사내 도구 전체에 LGPL 전염).

#### KooCADCAM 빌드에 연결

설치 후 환경 변수 설정:

```powershell
# Windows (PowerShell)
$env:OCCT_INSTALL_DIR = "C:\dev\occt-8.0.0"
$env:PATH = "$env:OCCT_INSTALL_DIR\bin;$env:PATH"
```

```bash
# Linux / macOS
export OCCT_INSTALL_DIR=/opt/occt-8.0.0
export LD_LIBRARY_PATH=$OCCT_INSTALL_DIR/lib:$LD_LIBRARY_PATH
```

KooCADCAM의 `CMakePresets.json` 은
`OpenCASCADE_DIR=$env{OCCT_INSTALL_DIR}/lib/cmake/opencascade` 를 자동으로 매핑한다
([[architecture/build-and-deps]]).

#### 모듈 선택 가이드

| OCCT 모듈 | KooCADCAM 필요 여부 |
|---|---|
| `FoundationClasses` (기본) | 필수 |
| `ModelingData` | 필수 (`TopoDS`, `gp`, `BRep` 등) |
| `ModelingAlgorithms` | 필수 (`BRepBuilderAPI`, `BRepAlgoAPI`, `BRepFilletAPI`, `BRepExtrema`) |
| `Visualization` | 필수 (`V3d`, `AIS`, `OpenGl`) |
| `ApplicationFramework` | 필수 (`TDocStd`, `TDF` — OCAF 향후 도입용) |
| `DataExchange` | 필수 (`STEPControl`, `IGES`, `XCAF`) |
| `Draw` | 불필요 (커맨드라인 TCL 인터프리터; CI 시간 절약) |

#### 빌드 결과 검증

```bash
# 1. 핵심 DLL 존재 확인
ls $OCCT_INSTALL_DIR/bin/TKernel.dll
ls $OCCT_INSTALL_DIR/bin/TKV3d.dll
ls $OCCT_INSTALL_DIR/bin/TKSTEP.dll

# 2. CMake 패키지 검증
cmake -P - <<'CMK'
find_package(OpenCASCADE 8.0 REQUIRED PATHS $ENV{OCCT_INSTALL_DIR}/lib/cmake/opencascade)
message(STATUS "OCCT version: ${OpenCASCADE_VERSION}")
CMK
```

---

## 컴파일타임 트립와이어 체크리스트

7.x 코드를 8.0으로 올리기 전에 아래 패턴을 grep으로 전수 검색한다.

```bash
#!/usr/bin/env bash
# pre_migration_check.sh
echo "=== OCCT 8.0 Migration Tripwires ==="

echo "[M2] Standard_Failure::Raise calls:"
grep -rn "Standard_Failure::Raise\|::Instance()" src/ || echo "  CLEAN"

echo "[M3] Deprecated math functions:"
grep -rn "\bACos(\|\bASin(\|\bATan(\|\bSqrt(\|\bAbs(\|\bCos(\|\bSin(\|\bPow(\|\bExp(\|\bLog(" \
     src/ --include="*.cpp" --include="*.hpp" || echo "  CLEAN"

echo "[M4] Standard_UNUSED macro:"
grep -rn "Standard_UNUSED" src/ || echo "  CLEAN"

echo "[M5] NCollection Assign usage:"
grep -rn "\.Assign(" src/ --include="*.cpp" --include="*.hpp" || echo "  CLEAN"

echo "[M6] Geom_Curve/Surface subclasses:"
grep -rn "public Geom_Curve\|public Geom_Surface" src/ || echo "  CLEAN"

echo "[M7] Legacy D0/D1/D2/D3 overrides:"
grep -rn "void D0.*override\|void D1.*override\|void D2.*override\|void D3.*override" \
     src/ || echo "  CLEAN"
```

---

## Migration Progress Tracker

팀원이 각 항목을 처리할 때마다 상태를 업데이트한다.

| # | 항목 | 상태 | 담당자 | 완료 날짜 | 비고 |
|---|---|---|---|---|---|
| M1 | C++17 CMake 설정 | ⬜ 미착수 | — | — | |
| M2 | `Raise()` → `throw` 전환 | ⬜ 미착수 | — | — | sed 스크립트 준비됨 |
| M3 | 수학 함수 `std::*` 치환 | ⬜ 미착수 | — | — | `migrate_math.sh` 준비됨 |
| M4 | `Standard_UNUSED` → `[[maybe_unused]]` | ⬜ 미착수 | — | — | 위치 규칙 주의 |
| M5 | `NCollection::Assign` 범위 변경 확인 | ⬜ 미착수 | — | — | 런타임 테스트 필수 |
| M6 | EvalRep 서브클래스 확인 | ⬜ 미착수 | — | — | 해당 없으면 생략 |
| M7 | include 경로 재검증 | ⬜ 미착수 | — | — | vcpkg vs 직접 빌드 |
| M8 | OCCT 8.0.0 소스 빌드 (vcpkg 미사용) | ⬜ 미착수 | — | — | `BUILD_LIBRARY_TYPE=Shared` 필수 |
| — | `pre_migration_check.sh` 전체 통과 | ⬜ 미착수 | — | — | 최종 확인 |

상태 아이콘: ⬜ 미착수 / 🔄 진행 중 / ✅ 완료 / ⚠️ 이슈

---

## Build Troubleshooting (real-world hits 2026-05-25)

> Verified on Windows 11 / MSVC 2022 Community host (Sonic's PC) during M0/M1.1 bring-up.
> See also [[architecture/build-and-deps]] for the canonical build policy and [[process/coding-standards]] for script style rules.

---

### BT-1: Intel oneAPI `icx` picked instead of `cl.exe`

**Symptom**: CMake configures successfully but the ninja link step fails with exit code 1104 (Intel LLVM linker error) even though VS 2022 is the intended toolchain.

**Root cause**: `D:\Program Files (x86)\intel\oneAPI\compiler\latest\windows\bin\intel64` appears before the VS Dev Shell entries in user PATH, so CMake's compiler detection finds `icx` first.

**Fix**:

```powershell
# Launch a clean VS 2022 Dev Shell, then strip oneAPI from the inherited PATH before cmake:
$env:PATH = ($env:PATH -split ';' | Where-Object { $_ -notmatch 'oneAPI' }) -join ';'
cmake --preset windows-msvc-release
```

Or set `CC` / `CXX` explicitly in `CMakePresets.json`:

```json
"cacheVariables": { "CMAKE_C_COMPILER": "cl", "CMAKE_CXX_COMPILER": "cl" }
```

**Prevention**: `CMakePresets.json` `windows-msvc-release` preset pins `CMAKE_C_COMPILER` and `CMAKE_CXX_COMPILER` to `cl` — see [[architecture/build-and-deps]].

---

### BT-2: Stale OCCT 7.7.0 config picked over OCCT 8.0.0

**Symptom**: `find_package(OpenCASCADE)` resolves to `D:\OpenCASCADE-7.7.0-vc14-64\...\build\OpenCASCADEConfig.cmake` and immediately errors that component target files are missing.

**Root cause**: A leftover OCCT 7.7.0 install at that path exposes an incomplete `build/` config directory; CMake's search order finds it before the correct 8.0.0 install at `C:\Users\Sonic\occt-8.0.0\cmake\`.

**Fix**:

```powershell
# Point cmake directly at the 8.0.0 cmake directory:
$env:OpenCASCADE_DIR = "C:\Users\Sonic\occt-8.0.0\cmake"
cmake --preset windows-msvc-release
# Or add to preset cacheVariables: "OpenCASCADE_DIR": "$env{OCCT_INSTALL_DIR}/cmake"
```

**Prevention**: `CMakePresets.json` sets `OpenCASCADE_DIR` via `$env{OCCT_INSTALL_DIR}/cmake`; `OCCT_INSTALL_DIR` must be set to the 8.0.0 root before invoking cmake — documented in [[architecture/build-and-deps]].

---

### BT-3: VS-bundled vcpkg overrides project vcpkg via `VCPKG_ROOT`

**Symptom**: vcpkg reports a baseline SHA resolution failure, or installs packages from the wrong baseline, because `Launch-VsDevShell.ps1` silently sets `VCPKG_ROOT` to `D:\Program Files\Microsoft Visual Studio\2022\Community\VC\vcpkg`.

**Root cause**: The VS-bundled vcpkg clone is often a shallow clone (`--depth 1`); its baseline SHA diverges from `C:\Users\Sonic\vcpkg`, so cross-referencing fails.

**Fix**:

```powershell
# After launching Dev Shell, override VCPKG_ROOT:
$env:VCPKG_ROOT = "C:\Users\Sonic\vcpkg"
$env:PATH = "$env:VCPKG_ROOT;$env:PATH"
cmake --preset windows-msvc-release -DVCPKG_ROOT="$env:VCPKG_ROOT"
```

**Prevention**: The project's `init-env.ps1` bootstrap script explicitly sets `VCPKG_ROOT` after Dev Shell activation — see [[architecture/build-and-deps]].

---

### BT-4: PowerShell 5.1 parser — `$LASTEXITCODE:` and inline ternary

**Symptom**: Script fails with `A drive with the name 'LASTEXITCODE' does not exist` or `The term '...' is not recognized` when using `$LASTEXITCODE:` or an inline `(if (...) {...} else {...})` expression.

**Root cause**: PowerShell 5.1 parses `$LASTEXITCODE:` as a PSDrive reference (colon is the drive separator); it also has no ternary operator — `(if ...)` is a statement, not an expression.

**Fix**:

```powershell
# Brace the variable name to prevent PSDrive mis-parse:
if (${LASTEXITCODE} -ne 0) { throw "Build failed" }

# Replace inline ternary with explicit if/else + temp variable:
$msg = if ($ok) { "success" } else { "failure" }   # PS 7+
# PS 5.1 equivalent:
$msg = "failure"; if ($ok) { $msg = "success" }
```

**Prevention**: CI scripts target PS 5.1 explicitly; [[process/coding-standards]] bans ternary syntax and requires `${LASTEXITCODE}` form in all shell snippets.

---

### BT-5: vcpkg manifest `builtin-baseline` set to a date string

**Symptom**: `vcpkg install` refuses with `error: the baseline ... is not a valid commit SHA`.

**Root cause**: `vcpkg.json` contains `"builtin-baseline": "2024.11.16"` (a date, not a hex SHA); vcpkg requires the exact 40-character git commit SHA of the local vcpkg HEAD.

**Fix**:

```powershell
# Retrieve the correct SHA from your vcpkg clone and patch vcpkg.json:
$sha = git -C $env:VCPKG_ROOT rev-parse HEAD
(Get-Content vcpkg.json -Raw) -replace '"builtin-baseline":\s*"[^"]*"',
    "`"builtin-baseline`": `"$sha`"" | Set-Content vcpkg.json -Encoding utf8
```

**Prevention**: `scripts/update-vcpkg-baseline.ps1` regenerates this field automatically; it must be run after any `git pull` inside the vcpkg clone — see [[architecture/build-and-deps]].

---

### BT-6: OCCT Windows install layout — `OpenCASCADE_DIR` must point at `cmake/`

**Symptom**: `find_package(OpenCASCADE)` fails with "could not find a package configuration file" even though OCCT is installed, because `OpenCASCADE_DIR` is set to `<install>/lib/cmake/opencascade` (Generic layout) which does not exist.

**Root cause**: The OCCT Windows install does NOT use the standard `bin/lib/include/share` Generic layout; it uses `cmake/` + `inc/` + `win64/vc14/{bin,lib}` + `src/`. The CMake config lives at `<install>/cmake/`, not `<install>/lib/cmake/opencascade/`.

**Fix**:

```powershell
# Correct:
$env:OpenCASCADE_DIR = "C:\Users\Sonic\occt-8.0.0\cmake"
# Wrong (Generic layout assumption — does not exist on Windows):
# $env:OpenCASCADE_DIR = "C:\Users\Sonic\occt-8.0.0\lib\cmake\opencascade"
```

In `CMakePresets.json`:

```json
"OpenCASCADE_DIR": "$env{OCCT_INSTALL_DIR}/cmake"
```

**Prevention**: `CMakePresets.json` and `init-env.ps1` both use the `cmake/` suffix; the M7 section above documents this layout — see [[architecture/build-and-deps]].

---

### BT-7: `CreateProcess: Access is denied` during parallel ninja build

**Symptom**: Ninja aborts mid-build (~2107/5646 or similar) with `CreateProcess: Access is denied`; restarting the build resumes cleanly from the same point.

**Root cause**: Windows Defender (or another AV process) locks a freshly-compiled `.obj` or `.dll` file at the moment ninja tries to spawn the next process against it; this is a known Windows AV interference pattern documented by the OpenCASCADE community.

**Fix**:

```powershell
# Simply re-run; ninja resumes from the failure point:
cmake --build build --preset windows-msvc-release

# Reduce hit rate by limiting parallelism:
cmake --build build --parallel 8   # instead of --parallel 12 or CPU count
```

For a permanent fix: add `C:\Users\Sonic\occt-8.0.0` and the KooCADCAM `build/` directory to Windows Defender exclusions.

**Prevention**: `CMakePresets.json` caps `CMAKE_BUILD_PARALLEL_LEVEL` at `8` on Windows to reduce collision rate — see [[architecture/build-and-deps]].

---

## 크로스-링크 요약

- [[process/coding-standards]] — throw 정책, std::* 수학 함수, [[maybe_unused]] 규약
- [[architecture/build-and-deps]] — OCCT 소스 빌드 정책, CMake CXX_STANDARD, CI 빌드 행렬
