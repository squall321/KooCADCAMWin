# Parametric Templates

파라메트릭 루트는 KooCADCAM의 **1차 생산 경로**다. JSON 스펙 → `FeatureGraph` 실행 → STEP 출력의 흐름은
결정론적이어야 한다(동일 JSON → 동일 SHA-256). 이 문서는 모든 제품 모델이 상속해야 하는 공통 기반
`ProductFrontModel`과 그 계약을 정의한다.

- 대상 독자: 엔진 개발자, 리뷰어
- 1차 대상 제품: 시계 (→ [[engine/feature-watch]])
- 2차 대상 제품: 스마트폰 (→ [[engine/feature-phone]])
- DFM 검증 규칙 카탈로그: [[engine/dfm-rules]]
- 리버스 엔지니어링 브리지: [[engine/reverse-route]]

---

## 기하 프리미티브 레이어 (`koocadcam::engine::prim`) — M1.2-refactor

빌더 스텝의 **중간 과정을 재사용 가능한 캡슐로 분해**한 헤더-온리 레이어다.
워치만이 아니라 폰/태블릿 등 향후 모든 제품 모델이 같은 프리미티브를 호출해
스텝을 작성한다.  스텝 본체는 도메인 어휘(베젤, 크라운, 사이드버튼)로 짧게
표현되고, 예외/검증/로그/Boolean 디테일은 프리미티브가 흡수한다.

```
src/engine/primitives/
 ├─ Bbox.hpp        Bbox3d  (dx/dy/dz, outerRadiusXY, center/topCenter/bottomCenter)
 │                  optimalBbox(shape) — BRepBndLib::AddOptimal 래퍼
 ├─ Frames.hpp      SideFrame  (center, inwardRadial, tangentCCW, axialZ, ax2InwardRadial())
 │                  sideFrameAt(R, angleDeg, z) — 각도 0°=+X, CCW 표준
 │                  axisAtZ(point), offsetPoint(base, dx, xDir, dy, yDir)
 ├─ Tools.hpp       cylinder / coneFrustum / box  (gp_Ax2 기반 팩토리)
 │                  annularRing(ax, outerR, innerR, h)
 │                  annularConeRing(ax, r1bottom, r2top, innerR, h)  ← 베젤 taper
 │                  sidePocketBox(frame, depth, length, width)       ← 측면 버튼/SIM 트레이
 │                  roundedRectPocketTool(...)                       ← 폰/태블릿 화면 포켓
 │                  domeSolid(ax, baseR, h, nSections)               ← freeform 돔 솔리드
 ├─ Cuts.hpp        cut(base, tool)
 │                  cutMany(base, [tool…])  ← compound + 단일 Boolean (배열 피처용)
 ├─ Fillets.hpp     EdgePredicate / FilletSpec
 │                  filletEdges(shape, r, predicate)
 │                  filletEdgesMulti(shape, {{r1, pred1}, {r2, pred2}, …})  ← 단일 패스
 │                  edgesAtZ(z, tol), verticalCornerEdges(hx, hy, thickness, tol)
 └─ StepGuard.hpp   runStep(name, body, checkValidity=true) → StepResult
                    body 시그니처: TopoDS_Shape(std::vector<BuildWarning>&)
                    try/catch → E_OCCT, BRepCheck → W_BREPCHECK, bbox spdlog::debug
```

### 의도 — "어디까지를 캡슐화 했나"

| 레이어 | 책임 | 사용처 |
|---|---|---|
| **prim/** | OCCT 호출 시퀀스, 예외→경고, BRepCheck, 로그, 좌표계 표준 | 모든 제품 모델 |
| **{Product}FrontModel::stepX** | 스펙 키 ↔ 도메인 의미 매핑, 어떤 프리미티브를 어떻게 조합할지 | 제품별 |
| **{Product}FrontModel::buildAll** | 스텝 순서 + warning 합산 + abort 정책 | 제품별 |

스텝 본체에는 더 이상 `try/catch`, `BRepCheck_Analyzer`, `Bnd_Box.Get(...)`,
`gp_Ax2(...)`, `BRepPrimAPI_Make…`, `BRepAlgoAPI_Cut`, `TopoDS_Compound` 같은
OCCT 골격이 들어가지 않는다.  스텝은 "버튼 위치 → sideFrameAt → sidePocketBox →
cutMany" 식의 4–5 줄로 압축된다.  자세한 비교는 [[engine/feature-watch]] 참조.

### 호출 패턴 예시

```cpp
// 워치 측면 버튼 — addSideButtons의 본질만 남김
return pr::runStep("WatchFrontModel::addSideButtons",
    [&](std::vector<BuildWarning>& w) {
        const double R = pr::optimalBbox(in).outerRadiusXY();
        std::vector<TopoDS_Shape> tools;
        for (const auto& btn : spec["side_buttons"]) {
            const auto frame = pr::sideFrameAt(R,
                btn["angle_deg"].get<double>(),
                btn["height_mm"].get<double>());
            tools.push_back(pr::sidePocketBox(frame,
                btn["depth_mm"].get<double>(),
                btn["length_mm"].get<double>(),
                btn["width_mm"].get<double>()));
        }
        return pr::cutMany(in, tools);
    });
```

### 좌표/규약 표준 (모든 프리미티브 공통)

- 각도 0° = +X (3시), CCW.  워치 12시 = 90°, 폰 카메라 패턴도 동일.
- 측정은 항상 `prim::optimalBbox` (NURBS 컨트롤 폴리곤 아닌 실제 표면 bbox).
  근거: [[process/occt8-migration-cookbook#BT-6 BRepBndLib::Add vs AddOptimal]]
- 예외는 `Standard_Failure(const char*)` 만 던진다.  `runStep`이 `BuildWarning`
  으로 변환한다.
- 셋업 단계에서는 `checkValidity=false` (예: `buildBase`의 cylinder/box는
  태생부터 valid).  Boolean 후속 스텝은 기본값 `true`.

### 비목표 (M1.2-refactor에서 의도적으로 미포함)

- 스텝 자체의 추상화 (`FeatureGraph`, `FeatureNode`) — 현 문서 아래 §공통
  기반 클래스 에서 **RESERVED**.  M3+ 인크리멘털 재실행 + RE 통합 시점 도입.
- DFM hook injection — M1.5 step 11 runDFM 도입 시.
- 멀티 제품 — `PhoneFrontModel`이 같은 프리미티브로 첫 스텝을 작성하는
  시점 ([[engine/feature-phone]]) 까지는 워치 한 제품만 사용한다.

---

## primitives — Tool shapes

`Tools.hpp` (소스 [[src/engine/primitives/Tools.hpp]]) 가 워크피스에 Boolean
"tool" 로 쓰이는 표준 프리미티브를 모은다.  각 함수는 빌드 실패 시
`Standard_Failure` 를 던지고 `prim::runStep` 이 `BuildWarning` 으로 변환한다.

| 팩토리 | 형상 | 비고 |
|---|---|---|
| `cylinder / coneFrustum / box` | 직선 원통/원뿔대/박스 | `gp_Ax2` 기반 |
| `annularRing(outerR, innerR, h)` | 직선 환형 링 | 외경−내경 cut |
| `annularConeRing(r1, r2, innerR, h)` | 테이퍼 외벽 환형 | 베젤 taper |
| `sidePocketBox(frame, depth, length, width)` | 측면 직사각 포켓 | 사이드 버튼/SIM |
| `roundedRectPocketTool(center, sx, sy, depth, cornerR)` | 라운드 직사각 포켓 | 폰/태블릿 화면 |
| `domeSolid(ax, baseR, h, nSections)` | **freeform 돔 솔리드** | 아래 참조 |

### `domeSolid` — 첫 재사용 freeform 솔리드 블록 (commit a993001)

`BRepOffsetAPI_ThruSections(isSolid=true, ruled=false)` 로 구면 캡(spherical-cap)
프로파일을 따르는 원형 단면 stack 을 로프트해 **watertight 솔리드**(자유-서있는
face 가 아님)를 만든다.  옆면은 곡면(BSpline) 이므로 출력이 진짜 freeform 지오메트리를
담는다.  `baseR` 에서 시작해 apex 는 작은 양수 반경(`max(0.3, baseR*0.04)`)으로 클램프
하여 ThruSections 가 degenerate top wire 없이 닫힌 캡을 만든다.  `nSections>=3` 가
프로파일 충실도를 결정하고, `axis` Z 가 돔 축·`axis` location 이 base-circle 중심이다.

이는 머시닝 스킬 라이브러리([[engine/skills]])의 freeform 트랙과 동일 OCCT 패밀리
(`BRepOffsetAPI_ThruSections` 로프트)를 공유하는 **첫 product-side 소비자**다 — 곡면
엔진이 비로소 실제 제품 피처(돔 글래스/센서 돔/카메라 범프 등 protruding 피처의
fuse-before-cut 재료)로 쓰인다.  소비처:
[[engine/feature-watch#스텝 4 — buildDisplayPocket]] (돔 사파이어 글래스),
[[engine/feature-watch#스텝 8 — addRearSensors]] (후면 센서 돔).

---

## 공통 기반 클래스: `ProductFrontModel`

> **예약 경로** — 아래 헤더 블록의 실제 소스 위치는 `[[src/engine/ProductFrontModel.hpp]]`입니다.

```cpp
// [[src/engine/ProductFrontModel.hpp]]
// C++17 / OCCT 8.0.0
#pragma once

#include <TopoDS_Shape.hxx>
#include <Standard_Handle.hxx>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <functional>
#include <optional>

namespace koo {

// ── 빌더 스텝 결과 ─────────────────────────────────────────────
struct StepResult {
    TopoDS_Shape shape;
    std::vector<std::string> warnings;   // 비치명적 DFM 경고
};

// ── DFM 게이트 훅 시그니처 ────────────────────────────────────
// 리턴 false → 치명적 DFM 실패: 빌드 중단
using DFMHook = std::function<bool(const TopoDS_Shape&,
                                   const nlohmann::json& stepParams,
                                   std::vector<std::string>& outWarnings)>;

struct DFMHookSet {
    std::vector<DFMHook> beforeStep;   // 스텝 실행 전 게이트
    std::vector<DFMHook> afterStep;    // 스텝 실행 후 게이트
};

// ── FeatureGraph 노드 ──────────────────────────────────────────
struct FeatureNode {
    std::string      stepId;        // e.g. "buildBase", "applyCornerRadius"
    nlohmann::json   params;        // 스텝별 파라미터 (JSON)
    DFMHookSet       hooks;         // 이 스텝에 등록된 DFM 게이트
};

// ── FeatureGraph ───────────────────────────────────────────────
// 순서 보존 리스트. 파라미터 변경 시 해당 노드부터 재실행.
class FeatureGraph {
public:
    void            append(FeatureNode node);
    void            updateParams(const std::string& stepId,
                                 const nlohmann::json& params);
    const std::vector<FeatureNode>& nodes() const { return nodes_; }

private:
    std::vector<FeatureNode> nodes_;
};

// ── DFM 리포트 ────────────────────────────────────────────────
struct DFMReport {
    bool                     passed;
    std::vector<std::string> failures;    // 치명적 위반
    std::vector<std::string> warnings;    // 경고 (통과 가능)
};

// ── ProductFrontModel — CRTP 기반 공통 기반 ───────────────────
//
// 설계 트레이드오프: virtual vs CRTP vs Strategy
//
//   • virtual dispatch:
//       장점 — 단순, 런타임 다형성, 플러그인 확장 용이
//       단점 — 빌드 루프 내 가상 호출 오버헤드, LTO 최적화 제한
//
//   • CRTP (Curiously Recurring Template Pattern):  ← 채택
//       장점 — 인라인 가능, 컴파일-타임 다형성, 빌드 루프에서 0 오버헤드
//       단점 — 코드가 복잡해지고 이진 크기 증가
//       근거 — buildBase/applyCornerRadius는 매 파라미터 변경마다 재호출됨.
//              OCCT 형상 연산이 ms 단위이므로 vtable 오버헤드를 없애는 것이 중요.
//
//   • Strategy (함수 객체 주입):
//       장점 — 런타임 교체 가능
//       단점 — 상태 캡처·수명 관리 복잡, 현재 요구사항 초과
//
// → 결론: 빌드 성능이 중요한 스텝(buildBase, fillet)은 CRTP로 정적 디스패치.
//         DFM 게이트는 런타임 교체가 필요하므로 std::function (Strategy) 유지.

template<typename Derived>
class ProductFrontModel {
public:
    // ── 필수 구현 스텝 (Derived에서 오버라이드) ─────────────────

    /// 1. 기본 바디 생성 — 스펙에 따라 원형/직사각형 블록
    StepResult buildBase(const nlohmann::json& spec);

    /// 2. 외곽 코너 필렛 적용 — BRepFilletAPI_MakeFillet
    StepResult applyCornerRadius(const TopoDS_Shape& in,
                                 const nlohmann::json& spec);

    /// 3. 제품별 세부 피처 추가 (시계 베젤, 스마트폰 카메라홀 등)
    StepResult addFeatures(const TopoDS_Shape& in,
                           const nlohmann::json& spec);

    /// 4. DFM 게이트 일괄 실행 — DFM 카탈로그([[engine/dfm-rules]]) 호출
    DFMReport  runDFM(const TopoDS_Shape& shape,
                      const nlohmann::json& fullSpec);

    /// 5. STEP 출력 — STEPControl_Writer, 결정론적 헤더 고정
    bool       exportSTEP(const TopoDS_Shape& shape,
                          const std::string& outPath,
                          const nlohmann::json& metadata);

    // ── FeatureGraph 실행 엔진 ──────────────────────────────────

    /// 전체 그래프를 순서대로 실행. 중간 실패 시 즉시 중단.
    std::optional<TopoDS_Shape> execute(const FeatureGraph& graph,
                                        const nlohmann::json& rootSpec);

    /// 특정 스텝부터 재실행 (파라미터 변경 시 incremental rebuild)
    std::optional<TopoDS_Shape> reexecuteFrom(
        const FeatureGraph& graph,
        const std::string&  fromStepId,
        const TopoDS_Shape& cachedShapeBefore,
        const nlohmann::json& rootSpec);

    // ── DFM 훅 등록 ─────────────────────────────────────────────

    /// 특정 스텝 ID에 BeforeStep DFM 훅 등록
    void registerBeforeHook(const std::string& stepId, DFMHook hook);

    /// 특정 스텝 ID에 AfterStep DFM 훅 등록
    void registerAfterHook(const std::string& stepId, DFMHook hook);

    /// 기본 DFM 검증기를 모든 스텝에 등록 (생성자에서 호출 권장)
    void installDefaultDFMGates();

protected:
    FeatureGraph graph_;
    // 스텝별 중간 형상 캐시 (stepId → shape)
    std::unordered_map<std::string, TopoDS_Shape> shapeCache_;

private:
    Derived& derived() { return static_cast<Derived&>(*this); }
    std::unordered_map<std::string, DFMHookSet> hookRegistry_;
};

} // namespace koo
```

---

## 피처 스텝 계약

각 빌더 스텝은 다음 계약을 준수해야 한다.

### 입력/출력 계약

```
입력  : (const nlohmann::json& stepParams, const TopoDS_Shape& currentShape)
출력  : StepResult { TopoDS_Shape newShape, vector<string> warnings }
```

- `stepParams` — 해당 스텝에 필요한 파라미터만 포함한 JSON 서브트리
- `currentShape` — 이전 스텝 결과 또는 Null Shape (첫 번째 스텝)
- `warnings` — 비치명적 경고. 빈 벡터 = 경고 없음
- 예외 시 `throw Standard_Failure(...)` (OCCT 8.0 표준; `Raise()` 금지)

### 순수성(Purity) 요구사항

스텝은 **순수(pure)** 해야 한다.

| 금지 사항 | 대체 |
|-----------|------|
| 전역 변수 읽기/쓰기 | 스텝 파라미터로 주입 |
| `time()`, `rand()` 등 비결정적 소스 | 시드 고정 PRNG — `std::mt19937(seed)` (seed는 spec JSON에서 읽음) |
| 파일시스템 직접 접근 | `exportSTEP` 전용 스텝에서만 허용 |
| OCCT 스레드-로컬 상태 변형 | 진입 전 저장/복원 |

표면 노이즈가 필요한 텍스처 스텝에서는:
```cpp
// 결정론적 PRNG 사용 예시
std::mt19937 rng(spec.at("noise_seed").get<uint32_t>());
std::uniform_real_distribution<double> dist(-0.005, 0.005); // ±5μm
```

---

## DFM 게이트 삽입

`ProductFrontModel`은 플러그형 `BeforeStep`/`AfterStep` 훅 목록을 유지한다.
기본 DFM 검증기는 `installDefaultDFMGates()` 호출 시 모든 스텝에 자동 등록된다.

```
스텝 실행 흐름:
  ① beforeStep 훅 목록 순차 실행
      실패(false 반환) → StepResult에 오류 추가 후 execute() 중단
  ② 스텝 본체 실행 (OCCT 연산)
  ③ afterStep 훅 목록 순차 실행
      경고 → warnings에 추가, 계속 진행
      실패 → execute() 중단
```

```cpp
// 훅 등록 예시 — applyCornerRadius 후 최소 R 검증
model.registerAfterHook("applyCornerRadius",
    [](const TopoDS_Shape& s, const nlohmann::json& p,
       std::vector<std::string>& w) -> bool {
        // DFM-001: 외곽 모서리 R 최솟값 0.2 mm
        // BRepCheck_Analyzer로 곡면 연속성 확인
        BRepCheck_Analyzer ana(s);
        if (!ana.IsValid()) {
            w.push_back("DFM-001: fillet produced invalid shape");
            return false;
        }
        return true;
    });
```

DFM 규칙 ID 및 기준값은 [[engine/dfm-rules]]를 참조한다.

---

## Doc Object 패턴

각 문서(Doc)는 `FeatureGraph`와 루트 JSON 스펙을 소유한다.
파라미터 변경 시 변경된 노드부터 그래프를 재실행(incremental rebuild)한다.

```
KooDoc
 ├─ rootSpec : nlohmann::json        ← 전체 스펙 트리
 ├─ graph    : FeatureGraph          ← 순서 있는 스텝 노드 목록
 ├─ model    : ProductFrontModel*    ← CRTP 인스턴스 (WatchFrontModel 등)
 └─ shapeCache : map<stepId, Shape>  ← 중간 형상 캐시

파라미터 변경 시 흐름:
  GUI → KooDoc::updateParam(stepId, key, value)
      → rootSpec 갱신
      → model.reexecuteFrom(graph, stepId, cachedShape, rootSpec)
      → AIS_Shape 업데이트 → [[architecture/multi-view]] 뷰 갱신
```

Multi-document 아키텍처 상세는 [[architecture/multi-document]] 참조.
문서 모델 세부 사항은 [[architecture/document-model]] 참조.

---

## 결정론적 STEP 출력

동일 JSON 스펙 → 동일 STEP → 동일 SHA-256.

```cpp
// exportSTEP 구현 요구사항
STEPControl_Writer writer;
writer.Transfer(shape, STEPControl_AsIs);

// 헤더를 스펙 해시로 고정 (타임스탬프 제거)
Interface_Static::SetCVal("write.step.product.name",
    spec.at("product_name").get<std::string>().c_str());
Interface_Static::SetCVal("write.step.header.author", "KooCADCAM");
// ⚠️ write.step.header.timestamp = "" 로 설정 (결정론 보장)
Interface_Static::SetCVal("write.step.header.timestamp", "");

writer.Write(outPath.c_str());
```

STEP 파일 생성 후 SHA-256 검증은 CI 파이프라인에서 수행한다 ([[process/test-strategy]] 참조).

---

## 스펙 영속성 (JSON Schema 위치)

스펙은 JSON으로 직렬화된다. 스키마 정의는 각 제품 문서에 위치한다.

- 시계 스펙 스키마: [[engine/feature-watch#JSON Schema]]
- 스마트폰 스펙 스키마: [[engine/feature-phone#JSON Schema 스켈레톤]]

스펙 버전 관리: `"schema_version"` 필드 (semver). 마이그레이션은
[[process/coding-standards]] §스펙 버전 정책 참조.

---

## 리버스 엔지니어링 브리지

RE 파이프라인([[engine/reverse-route]])은 기존 STEP/STL에서 피처를 인식하여
**동일한 `FeatureGraph` 구조**로 복원한다. 이것이 RE와 파라메트릭 루트를
별도 앱이 아닌 형제(sibling) 루트로 취급하는 핵심 근거다.

```
RE 파이프라인 출력:
  FeatureGraph {
    FeatureNode { stepId="buildBase",      params={diameter=44, ...} }
    FeatureNode { stepId="applyCornerRadius", params={r_top=3.0, ...} }
    ...
  }
  ↓
  KooDoc에 로드
  ↓
  model.execute(graph, spec)  ← 파라메트릭 재실행 가능
```

복원된 그래프는 파라미터 편집, DFM 재검증, STEP 재출력이 모두 가능하다.

---

## 참고 링크

- [[engine/feature-watch]] — WatchFrontModel 구현
- [[engine/feature-phone]] — PhoneFrontModel 구현 (M2/M7+)
- [[engine/dfm-rules]] — DFM 규칙 카탈로그 (22+ 규칙)
- [[engine/reverse-route]] — RE 파이프라인 및 FeatureGraph 복원
- [[architecture/multi-document]] — 멀티 문서 아키텍처
- [[architecture/multi-view]] — 멀티 뷰 구성
- [[architecture/document-model]] — Doc Object 세부 구조
- [[process/coding-standards]] — C++17 코딩 표준, 스펙 버전 정책
- [[process/test-strategy]] — 결정론적 SHA-256 검증 CI
