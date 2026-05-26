# WatchFrontModel

시계 전면 케이스의 파라메트릭 모델. `ProductFrontModel` CRTP 기반을 상속하여
10개 빌드 스텝을 순서대로 실행하고 별도의 `runDFM` 게이트로 DFM 검증한다.
1차 납품 대상(`[[scope/milestones-and-krs#M1]]`).

- 공통 기반: [[engine/parametric-templates]]
- DFM 규칙: [[engine/dfm-rules]]
- 스마트폰 포팅 참조: [[engine/feature-phone]]

> **M1.5 SHIPPED (2026-05-27)** — 10/10 build steps + runDFM 게이트, 모두 `koocadcam::engine::prim` 헬퍼만 조합.
> ctest `watch` + `watch_features` 14 sub-test PASS (M1.2 7 + M1.5 7).  KooCADCAM.exe 시각 검증 통과.

---

## Build Sequence (M1.5, shipped)

| # | 메서드 | 도메인 책임 | 주요 프리미티브 |
|---|---|---|---|
| 1 | `buildBase` | 원형/사각 케이스 바디 + 사각 폼팩터의 수직 코너 필렛 | `prim::cylinder` 또는 `prim::box` + `filletEdges` + `verticalCornerEdges` |
| 2 | `applyCornerRadius` | 상면(rTop) + 저면(rSide) 동시 필렛 | `prim::filletEdgesMulti` + `edgesAtZ` |
| 3 | `buildBezel` | 베젤 환형 포켓; taper>0 → cone-frustum 외벽 | `prim::annularRing` 또는 `annularConeRing` + `cut` |
| 4 | `buildDisplayPocket` | 상면 원형 디스플레이 안착 홈 (glass_offset 보정) | `prim::cylinder` + `cut` |
| 5 | `addCrownCavity` | 측면 캐비티 + 관통 샤프트 (3시 표준 0°) | `prim::sideFrameAt` + `cylinder` × 2 + `cutMany` |
| 6 | `addSideButtons` | 측면 직사각 버튼 포켓 배열 | `prim::sideFrameAt` + `sidePocketBox` + `cutMany` |
| 7 | `addSpeakerGrille` | 측면 rows×cols 그릴 홀 패턴 (compound + 1-pass) | `prim::sideFrameAt` + `cylinder` × N + `offsetPoint` + `cutMany` |
| 8 | `addRearSensors` | 후면 HR/PPG 광학 창 (+Z 축 원홀 배열) | `prim::cylinder` + `cutMany` |
| 9 | `addLugs` | 12/6시 스트랩 러그 — **유일한 fuse 스텝** + 선택적 핀 홀 | `prim::sideFrameAt` + `box` + `fuseMany` + 선택적 `cylinder` + `cutMany` |
| 10 | `addSecondaryFillets` | 디스플레이 림 + 베젤 안쪽 단차의 마무리 필렛 (forgiving) | `prim::edgesAtZ` + `filletEdges` (try/catch per band) |

`buildAll` = 1→10 순차, 각 스텝은 `prim::runStep` 으로 `E_OCCT`/`W_BREPCHECK`/bbox-debug 동일 처리.

### Step 11 — `runDFM` (별도 정적 메서드)

`static DFMReport WatchFrontModel::runDFM(shape, spec)` — buildAll 과 분리된 게이트.
현재 구현 규칙 ([[engine/dfm-rules]]):

| Rule | Severity | Check |
|---|---|---|
| DFM-002 | Error | crown_cavity / speaker_grille / rear_sensors / lugs[].pin_hole 모든 홀 직경 ≥ 0.8 mm |
| DFM-009 | Error | bezel.width_mm ≥ 0.6 mm |
| DFM-014 | Error | speaker_grille pin thickness = min(row_sp, col_sp) − hole_dia ≥ 0.25 mm |
| DFM-020 | Warning | `BRepCheck_Analyzer` IsValid — open shells / tolerance 이슈 |
| DFM-022 | Warning | bbox dz(두께)가 dx/dy 중 최솟값과 같은지 확인 |

`DFMReport { passed, findings: [{code, severity, message}] }` 반환. `passed == false` 이면 STEP export 차단 (M3 이후 게이팅 정책).

### 좌표계 표준 (재확인)
- 각도 0° = +X (3시), CCW.  90° = 12시, 180° = 9시, 270° = 6시.
- `gp_Ax2(P, V, Vx)`: **V = 메인(Z) 방향, Vx = X 방향**.  이 규약을 무시하면 `box` dim이 직교 축으로 회전하여 측정이 어긋난다.  과거 commit (M1.5) 에서 lug box 의 V/Vx 혼동으로 dz=21.5mm 가 측정된 사례 있음 — 수정 후 dz=10.

---

## 클래스 API

> **예약 경로** — 아래 헤더 블록의 실제 소스 위치는 `[[src/engine/WatchFrontModel.hpp]]`입니다.

```cpp
// [[src/engine/WatchFrontModel.hpp]]
// C++17 / OCCT 8.0.0
#pragma once

#include "ProductFrontModel.hpp"
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gce_MakePln.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopExp_Explorer.hxx>
#include <STEPControl_Writer.hxx>
#include <Interface_Static.hxx>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>

namespace koo {

// ── 스펙 구조체 ────────────────────────────────────────────────

/// 측면 버튼 하나의 스펙
struct SideButton {
    double angle_deg;   ///< 측면 방위각 (0°= 3시 방향, CCW), [0, 360)
    double height_mm;   ///< 케이스 바닥면 기준 Z 위치 (mm)
    double length_mm;   ///< 버튼 포켓 길이 (mm), 권장 4–8
    double width_mm;    ///< 버튼 포켓 폭 (mm), 권장 2–4
    double depth_mm;    ///< 포켓 깊이 (mm), DFM-009: ≥ 0.4 최소 살두께 확보
    double taper_deg;   ///< 포켓 측벽 기울기 (°), 0 = 수직, 권장 1–3
};

/// 후면 센서 홀 스펙 (HR/PPG 광학 창)
struct SensorHole {
    double offset_x_mm; ///< 중심 X 오프셋 (케이스 중심 기준, mm)
    double offset_y_mm; ///< 중심 Y 오프셋 (케이스 중심 기준, mm)
    double diameter_mm; ///< 홀 직경 (mm), DFM-008: ≥ 0.8
    double depth_mm;    ///< 홀 깊이 (mm), 관통 = 전체 두께
    bool   through;     ///< true = 관통홀, false = 맹홀
};

/// 밴드 연결 러그 스펙 (상단/하단 한 쌍)
struct LugSpec {
    double width_mm;       ///< 러그 폭 (mm), 표준: 20, 22, 24
    double projection_mm;  ///< 케이스 외곽 돌출 길이 (mm)
    double thickness_mm;   ///< 러그 두께 (mm)
    double pin_hole_dia_mm;///< 스프링바 핀 홀 직경 (mm), 표준 1.8
    double fillet_r_mm;    ///< 러그 루트 필렛 반경 (mm)
};

// ── WatchFrontModel ────────────────────────────────────────────

class WatchFrontModel : public ProductFrontModel<WatchFrontModel> {
public:
    explicit WatchFrontModel();

    // ── 빌드 스텝 (순서 중요) ────────────────────────────────

    /// 스텝 1: 기본 바디 — 원형 또는 라운드스퀘어 퍽(puck)
    /// @param diameter_mm     원형 케이스 직경 (mm); round 폼팩터 전용
    /// @param width_mm        사각 케이스 폭 (mm); square 폼팩터 전용
    /// @param height_mm       사각 케이스 높이 (mm); square 폼팩터 전용
    /// @param thickness_mm    케이스 전체 두께 (mm)
    /// @param form_factor     "round" | "square"
    /// OCCT: BRepPrimAPI_MakeCylinder (round),
    ///       BRepPrimAPI_MakeBox + BRepFilletAPI_MakeFillet (square)
    StepResult buildBase(const nlohmann::json& spec);

    /// 스텝 2: 상단/측면 외곽 모서리 필렛
    /// @param r_top_mm    상면 테두리 필렛 반경 (mm)
    /// @param r_side_mm   측면-하단 접합부 필렛 반경 (mm)
    /// OCCT: BRepFilletAPI_MakeFillet — TopExp_Explorer로 엣지 순회 후 AddEdge
    StepResult applyCornerRadius(const TopoDS_Shape& in,
                                 const nlohmann::json& spec);

    /// 스텝 3: 베젤 포켓 — 상면 환형(annular) 홈
    /// @param width_mm   베젤 폭 (mm)
    /// @param depth_mm   포켓 깊이 (mm)
    /// @param taper_deg  내벽 테이퍼 각도 (°)
    /// OCCT: BRepPrimAPI_MakeCylinder(outer) cut BRepPrimAPI_MakeCylinder(inner)
    ///       → BRepAlgoAPI_Cut로 기본 바디에서 제거
    StepResult buildBezel(const TopoDS_Shape& in,
                          const nlohmann::json& spec);

    /// 스텝 4: 디스플레이 포켓 — 베젤 내부 유리 안착 홈
    /// @param d_pocket_mm      포켓 직경 또는 폭 (mm)
    /// @param depth_pocket_mm  포켓 깊이 (mm)
    /// @param glass_offset_mm  유리 상면과 케이스 상면의 단차 (mm, 음수=함몰)
    /// OCCT: BRepPrimAPI_MakeCylinder (포켓 툴) → BRepAlgoAPI_Cut
    StepResult buildDisplayPocket(const TopoDS_Shape& in,
                                  const nlohmann::json& spec);

    /// 스텝 5: 용두 캐비티 — 측면 원형 포켓 + 샤프트 홀
    /// @param side_angle_deg  용두 위치 방위각 (°), 보통 90° (3시)
    /// @param height_pos_mm   Z 위치 (mm)
    /// @param depth_mm        포켓 깊이 (mm)
    /// @param diameter_mm     캐비티 직경 (mm)
    /// @param shaft_dia_mm    용두 샤프트 홀 직경 (mm)
    /// OCCT: gp_Ax2로 로컬 좌표계 설정 →
    ///       BRepPrimAPI_MakeCylinder (캐비티) + BRepAlgoAPI_Cut
    StepResult addCrownCavity(const TopoDS_Shape& in,
                              const nlohmann::json& spec);

    /// 스텝 6: 측면 버튼 포켓 배열
    /// @param buttons  SideButton 스펙 벡터
    /// OCCT: 각 버튼마다 BRepPrimAPI_MakeBox (테이퍼 없을 때) 또는
    ///       BRepOffsetAPI_ThruSections (테이퍼 있을 때) → BRepAlgoAPI_Cut
    StepResult addSideButtons(const TopoDS_Shape& in,
                              const std::vector<SideButton>& buttons,
                              const nlohmann::json& spec);

    /// 스텝 7: 스피커 그릴 — 미세 원통 핀 배열 부울 절삭
    /// @param angle_deg    그릴 위치 방위각 (°)
    /// @param height_mm    Z 위치 (mm)
    /// @param pin_count    핀 홀 개수
    /// @param pin_dia_mm   핀 홀 직경 (mm), DFM-008: ≥ 0.8
    /// @param depth_mm     홀 깊이 (mm)
    /// OCCT: BRepPrimAPI_MakeCylinder × pin_count → BRepAlgoAPI_Cut 누적
    ///       (핀 수가 많을 경우 BRep_Builder::Add로 Compound 후 단일 Cut)
    StepResult addSpeakerGrille(const TopoDS_Shape& in,
                                const nlohmann::json& spec);

    /// 스텝 8: 후면 센서 홀 배열 (HR/PPG 광학 창)
    /// @param holes  SensorHole 스펙 벡터
    /// OCCT: BRepPrimAPI_MakeCylinder → BRepAlgoAPI_Cut (관통 또는 맹홀)
    ///       맹홀: 깊이 제한, 후면 기준 좌표계(gp_Ax2 반전)
    StepResult addRearSensorHoles(const TopoDS_Shape& in,
                                  const std::vector<SensorHole>& holes,
                                  const nlohmann::json& spec);

    /// 스텝 9: 러그 쌍 — 밴드 연결부 (상단 + 하단)
    /// @param lug_pair  LugSpec 2개 (index 0=top, 1=bottom)
    /// OCCT: BRepPrimAPI_MakeBox → BRepFilletAPI_MakeFillet (루트 필렛) →
    ///       BRepPrimAPI_MakeCylinder (핀 홀) → BRepAlgoAPI_Fuse (케이스에 합산)
    StepResult addLug(const TopoDS_Shape& in,
                      const std::array<LugSpec, 2>& lug_pair,
                      const nlohmann::json& spec);

    /// 스텝 10: 보조 엣지 라운드오버 (2차 필렛)
    /// @param secondary_r_mm  2차 필렛 반경 (mm), 보통 0.2–0.5
    /// OCCT: BRepFilletAPI_MakeFillet — 잔여 샤프 엣지 순회 후 AddEdge
    StepResult applyEdgeRoundovers(const TopoDS_Shape& in,
                                   const nlohmann::json& spec);

    /// 스텝 11: DFM 전체 검증
    /// DFM-001(외곽 R), DFM-008(홀 직경), DFM-009(살두께),
    /// DFM-010(홀 간격), DFM-019(러그 강도) 등 포함
    /// OCCT: BRepExtrema_DistShapeShape, BRepAdaptor_Surface,
    ///       BRepCheck_Analyzer, BRepGProp
    DFMReport  runDFM(const TopoDS_Shape& shape,
                      const nlohmann::json& fullSpec);
};

} // namespace koo
```

---

## 빌드 시퀀스 상세

### 스텝 1 — `buildBase`

원형 폼팩터: `BRepPrimAPI_MakeCylinder(gp_Ax2(origin, gp::DZ()), radius, thickness)`
사각 폼팩터: `BRepPrimAPI_MakeBox(width, height, thickness)` 후 4개 수직 엣지에
`BRepFilletAPI_MakeFillet`로 초기 R 적용.

결정론적 보장: 원점과 축 방향을 항상 `gp::Origin()`, `gp::DZ()`로 고정.

### 스텝 2 — `applyCornerRadius`

```cpp
BRepFilletAPI_MakeFillet fillet(base_shape);
TopExp_Explorer exp(base_shape, TopAbs_EDGE);
for (; exp.More(); exp.Next()) {
    TopoDS_Edge e = TopoDS::Edge(exp.Current());
    // 상면/측면 접합 엣지만 선택 (Z 좌표 필터)
    fillet.Add(r_top_mm, e);
}
fillet.Build();
// DFM-001: BRepCheck_Analyzer(fillet.Shape()).IsValid() 확인
```

### 스텝 3 — `buildBezel`

외경 원통 - 내경 원통 = 환형 툴 → `BRepAlgoAPI_Cut(base, annular_tool)`.
테이퍼: 내벽에 `BRepOffsetAPI_ThruSections`으로 상/하단 와이어 단면 차이 적용.

### 스텝 4 — `buildDisplayPocket`

`BRepPrimAPI_MakeCylinder(pocket_radius, depth_pocket + glass_offset)` →
케이스 상면 기준 좌표 배치 → `BRepAlgoAPI_Cut`. `glass_offset < 0`이면 함몰(recessed).

### 스텝 5 — `addCrownCavity`

```cpp
// 로컬 좌표계: 케이스 측면 접선 방향
gp_Ax2 ax(gp_Pnt(x_side, 0, height_pos_mm),
           gp_Dir(std::cos(angle_rad), std::sin(angle_rad), 0));
BRepPrimAPI_MakeCylinder cavity(ax, diameter_mm / 2.0, depth_mm);
BRepAlgoAPI_Cut cut(current_shape, cavity.Shape());
// 샤프트 홀: 더 작은 직경으로 동일 축 관통 절삭
```

### 스텝 6 — `addSideButtons`

버튼 루프에서 각 `SideButton`마다:
1. `gp_Ax2`로 측면 로컬 좌표계 계산
2. `taper_deg == 0`: `BRepPrimAPI_MakeBox` (직각 포켓)
3. `taper_deg > 0`: `BRepOffsetAPI_ThruSections` (상하 단면이 다른 테이퍼 포켓)
4. `BRepAlgoAPI_Cut(shape, pocket_tool)` 누적

DFM 게이트: `afterStep` 훅에서 최소 살두께 확인 (DFM-009, `BRepExtrema_DistShapeShape`).

### 스텝 7 — `addSpeakerGrille`

핀 홀이 많을 경우 `BRep_Builder::Add`로 Compound 생성 후 단일 `BRepAlgoAPI_Cut` 호출.
이는 다중 Cut 반복 대비 Boolean 연산 횟수를 최소화한다.

```cpp
TopoDS_Compound pinCompound;
BRep_Builder builder;
builder.MakeCompound(pinCompound);
for (int i = 0; i < pin_count; ++i) {
    // 핀 위치 계산 (등간격 배열)
    BRepPrimAPI_MakeCylinder pin(ax_i, pin_dia / 2.0, depth_mm);
    builder.Add(pinCompound, pin.Shape());
}
BRepAlgoAPI_Cut grilleCut(current_shape, pinCompound);
```

DFM 게이트: DFM-008 (핀 직경 ≥ 0.8 mm), DFM-010 (핀 간 최소 거리 ≥ 1.5 mm).

### 스텝 8 — `addRearSensorHoles`

후면 기준 좌표계 (Z축 반전): `gp_Ax2(rear_center, -gp::DZ())`.
관통홀(`through=true`): 전체 두께를 깊이로 설정.
맹홀(`through=false`): `depth_mm` 지정. `BRepExtrema_DistShapeShape`로 후면까지
최소 잔류 살두께 DFM-009 검증.

### 스텝 9 — `addLug`

```
러그 블록 생성 (BRepPrimAPI_MakeBox)
    → 루트 필렛 (BRepFilletAPI_MakeFillet, lug_spec.fillet_r_mm)
    → 핀 홀 절삭 (BRepPrimAPI_MakeCylinder → BRepAlgoAPI_Cut)
    → 케이스에 융합 (BRepAlgoAPI_Fuse)
    → 대칭 배치: 상단 Y+ 방향, 하단 Y- 방향
```

### 스텝 10 — `applyEdgeRoundovers`

`TopExp_Explorer`로 잔여 샤프 엣지 탐색. 곡률 연속성이 없는 엣지에만
`BRepFilletAPI_MakeFillet::Add(secondary_r_mm, edge)` 적용.

### 스텝 11 — `runDFM`

| DFM 규칙 ID | 검증 항목 | OCCT API |
|-------------|-----------|----------|
| DFM-001 | 외곽 모서리 R ≥ 0.2 mm | `BRepAdaptor_Surface` 곡률 분석 |
| DFM-008 | 홀 직경 ≥ 0.8 mm | 피처 파라미터 직접 비교 |
| DFM-009 | 최소 살두께 ≥ 0.4 mm | `BRepExtrema_DistShapeShape` |
| DFM-010 | 홀 간 최소 거리 ≥ 1.5 mm | `BRepExtrema_DistShapeShape` |
| DFM-019 | 러그 루트 강도 (단면적 ≥ 기준) | `BRepGProp::SurfaceProperties` |

자세한 DFM 규칙: [[engine/dfm-rules]].

---

## 스펙 구조체 타입 정의

```cpp
// 단위: mm, degrees (명시된 경우 외)
// SideButton — 스텝 6 입력
struct SideButton {
    double angle_deg;    // [0, 360)
    double height_mm;    // [0, case_thickness]
    double length_mm;    // [2.0, 15.0]
    double width_mm;     // [1.5, 6.0]
    double depth_mm;     // [0.3, 2.0]   — DFM-009 하한 고려
    double taper_deg;    // [0.0, 5.0]
};

// SensorHole — 스텝 8 입력
struct SensorHole {
    double offset_x_mm;  // [-diameter/2 + 1, +diameter/2 - 1]
    double offset_y_mm;  // [-diameter/2 + 1, +diameter/2 - 1]
    double diameter_mm;  // [0.8, 8.0]  — DFM-008 하한
    double depth_mm;     // [0, case_thickness]
    bool   through;      // true = 관통
};

// LugSpec — 스텝 9 입력 (쌍으로 사용)
struct LugSpec {
    double width_mm;        // [18.0, 26.0]  표준 밴드 폭 기준
    double projection_mm;   // [3.0, 8.0]
    double thickness_mm;    // [1.5, 4.0]
    double pin_hole_dia_mm; // [1.5, 2.0]  표준 스프링바
    double fillet_r_mm;     // [0.3, 1.5]
};
```

---

## JSON Schema (Draft 2020-12)

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://koo-cadcam.local/schemas/watch-spec/v1.0.0",
  "title": "WatchFrontSpec",
  "description": "시계 전면 케이스 파라메트릭 빌드 스펙. 단위: mm, degrees.",
  "type": "object",
  "required": [
    "schema_version", "product_name", "form_factor",
    "base", "corner_radius", "bezel", "display_pocket"
  ],
  "properties": {
    "schema_version": {
      "type": "string",
      "const": "1.0.0",
      "description": "스펙 스키마 버전 (semver). 마이그레이션은 coding-standards §스펙 버전 정책 참조."
    },
    "product_name": {
      "type": "string",
      "maxLength": 64,
      "description": "STEP 헤더 product name 필드에 삽입."
    },
    "form_factor": {
      "type": "string",
      "enum": ["round", "square"],
      "description": "round = 원형 퍽, square = 라운드 사각형 퍽."
    },
    "noise_seed": {
      "type": "integer",
      "minimum": 0,
      "default": 0,
      "description": "표면 노이즈 PRNG 시드. 0 = 노이즈 없음. 결정론 보장을 위해 명시 권장."
    },
    "base": {
      "type": "object",
      "description": "스텝 1: 기본 바디 치수.",
      "required": ["thickness_mm"],
      "properties": {
        "diameter_mm": {
          "type": "number", "minimum": 30.0, "maximum": 55.0,
          "description": "원형 케이스 직경 (mm). form_factor=round 시 필수."
        },
        "width_mm": {
          "type": "number", "minimum": 30.0, "maximum": 55.0,
          "description": "사각 케이스 폭 (mm). form_factor=square 시 필수."
        },
        "height_mm": {
          "type": "number", "minimum": 30.0, "maximum": 60.0,
          "description": "사각 케이스 높이 (mm). form_factor=square 시 필수."
        },
        "thickness_mm": {
          "type": "number", "minimum": 5.0, "maximum": 20.0,
          "default": 10.0,
          "description": "케이스 전체 두께 (mm)."
        }
      }
    },
    "corner_radius": {
      "type": "object",
      "description": "스텝 2: 외곽 필렛 반경.",
      "required": ["r_top_mm", "r_side_mm"],
      "properties": {
        "r_top_mm": {
          "type": "number", "minimum": 0.2, "maximum": 5.0,
          "description": "상면 테두리 필렛 (mm). DFM-001: ≥ 0.2."
        },
        "r_side_mm": {
          "type": "number", "minimum": 0.2, "maximum": 3.0,
          "description": "측면-하단 접합 필렛 (mm)."
        }
      }
    },
    "bezel": {
      "type": "object",
      "description": "스텝 3: 베젤 환형 포켓.",
      "required": ["width_mm", "depth_mm"],
      "properties": {
        "width_mm": {
          "type": "number", "minimum": 1.0, "maximum": 8.0,
          "description": "베젤 폭 (mm)."
        },
        "depth_mm": {
          "type": "number", "minimum": 0.3, "maximum": 3.0,
          "description": "포켓 깊이 (mm)."
        },
        "taper_deg": {
          "type": "number", "minimum": 0.0, "maximum": 10.0, "default": 0.0,
          "description": "내벽 테이퍼 각도 (degrees)."
        }
      }
    },
    "display_pocket": {
      "type": "object",
      "description": "스텝 4: 디스플레이 유리 안착 포켓.",
      "required": ["d_pocket_mm", "depth_pocket_mm"],
      "properties": {
        "d_pocket_mm": {
          "type": "number", "minimum": 20.0, "maximum": 50.0,
          "description": "포켓 직경 (round) 또는 내접 폭 (mm)."
        },
        "depth_pocket_mm": {
          "type": "number", "minimum": 0.3, "maximum": 2.0,
          "description": "포켓 깊이 (mm)."
        },
        "glass_offset_mm": {
          "type": "number", "minimum": -1.0, "maximum": 0.5, "default": -0.1,
          "description": "유리 상면과 케이스 상면의 단차 (mm). 음수 = 함몰."
        }
      }
    },
    "crown_cavity": {
      "type": "object",
      "description": "스텝 5: 용두 캐비티 (선택).",
      "properties": {
        "side_angle_deg": {
          "type": "number", "minimum": 0, "maximum": 360, "default": 90,
          "description": "방위각 (°). 90 = 3시 방향."
        },
        "height_pos_mm": {
          "type": "number", "minimum": 0, "maximum": 20,
          "description": "케이스 바닥 기준 Z 위치 (mm)."
        },
        "depth_mm": {
          "type": "number", "minimum": 1.0, "maximum": 5.0,
          "description": "캐비티 깊이 (mm)."
        },
        "diameter_mm": {
          "type": "number", "minimum": 2.0, "maximum": 10.0,
          "description": "캐비티 직경 (mm)."
        },
        "shaft_dia_mm": {
          "type": "number", "minimum": 0.8, "maximum": 3.0, "default": 1.5,
          "description": "용두 샤프트 홀 직경 (mm)."
        }
      }
    },
    "side_buttons": {
      "type": "array",
      "description": "스텝 6: 측면 버튼 포켓 배열.",
      "items": {
        "type": "object",
        "required": ["angle_deg", "height_mm", "length_mm", "width_mm", "depth_mm"],
        "properties": {
          "angle_deg":  { "type": "number", "minimum": 0, "maximum": 360 },
          "height_mm":  { "type": "number", "minimum": 0, "maximum": 20 },
          "length_mm":  { "type": "number", "minimum": 2.0, "maximum": 15.0 },
          "width_mm":   { "type": "number", "minimum": 1.5, "maximum": 6.0 },
          "depth_mm":   { "type": "number", "minimum": 0.3, "maximum": 2.0,
                          "description": "DFM-009: 잔류 살두께 ≥ 0.4 mm 보장." },
          "taper_deg":  { "type": "number", "minimum": 0, "maximum": 5, "default": 0 }
        }
      }
    },
    "speaker_grille": {
      "type": "object",
      "description": "스텝 7: 스피커 그릴 (선택).",
      "properties": {
        "angle_deg":    { "type": "number", "minimum": 0, "maximum": 360 },
        "height_mm":    { "type": "number", "minimum": 0, "maximum": 20 },
        "pin_count":    { "type": "integer", "minimum": 1, "maximum": 50 },
        "pin_dia_mm":   { "type": "number", "minimum": 0.8, "maximum": 3.0,
                          "description": "DFM-008: ≥ 0.8 mm." },
        "depth_mm":     { "type": "number", "minimum": 0.3, "maximum": 3.0 }
      }
    },
    "rear_sensor_holes": {
      "type": "array",
      "description": "스텝 8: 후면 센서 홀 배열.",
      "items": {
        "type": "object",
        "required": ["diameter_mm", "depth_mm", "through"],
        "properties": {
          "offset_x_mm": { "type": "number" },
          "offset_y_mm": { "type": "number" },
          "diameter_mm":  { "type": "number", "minimum": 0.8, "maximum": 8.0,
                            "description": "DFM-008: ≥ 0.8 mm." },
          "depth_mm":     { "type": "number", "minimum": 0.5, "maximum": 20.0 },
          "through":      { "type": "boolean" }
        }
      }
    },
    "lug_pair": {
      "type": "array",
      "description": "스텝 9: 상단/하단 러그 쌍.",
      "minItems": 2, "maxItems": 2,
      "items": {
        "type": "object",
        "required": ["width_mm", "projection_mm", "thickness_mm", "pin_hole_dia_mm"],
        "properties": {
          "width_mm":        { "type": "number", "minimum": 18, "maximum": 26 },
          "projection_mm":   { "type": "number", "minimum": 3.0, "maximum": 8.0 },
          "thickness_mm":    { "type": "number", "minimum": 1.5, "maximum": 4.0 },
          "pin_hole_dia_mm": { "type": "number", "minimum": 1.5, "maximum": 2.0 },
          "fillet_r_mm":     { "type": "number", "minimum": 0.3, "maximum": 1.5,
                               "default": 0.5 }
        }
      }
    },
    "secondary_edge_r_mm": {
      "type": "number", "minimum": 0.1, "maximum": 1.0, "default": 0.2,
      "description": "스텝 10: 2차 엣지 라운드오버 반경 (mm)."
    }
  },
  "if": { "properties": { "form_factor": { "const": "round" } } },
  "then": { "properties": { "base": { "required": ["diameter_mm"] } } },
  "else": { "properties": { "base": { "required": ["width_mm", "height_mm"] } } }
}
```

---

## 워크드 예제: 44 mm 알루미늄 시계 전면

### 입력 스펙 (JSON 발췌)

```json
{
  "schema_version": "1.0.0",
  "product_name": "KooWatch-44-Al",
  "form_factor": "round",
  "noise_seed": 42,
  "base": { "diameter_mm": 44.0, "thickness_mm": 10.5 },
  "corner_radius": { "r_top_mm": 2.5, "r_side_mm": 1.0 },
  "bezel": { "width_mm": 3.5, "depth_mm": 0.8, "taper_deg": 2.0 },
  "display_pocket": {
    "d_pocket_mm": 35.5, "depth_pocket_mm": 0.6, "glass_offset_mm": -0.1
  },
  "crown_cavity": {
    "side_angle_deg": 90, "height_pos_mm": 5.0,
    "depth_mm": 2.5, "diameter_mm": 5.0, "shaft_dia_mm": 1.5
  },
  "side_buttons": [
    { "angle_deg": 270, "height_mm": 7.5, "length_mm": 6.0,
      "width_mm": 2.5, "depth_mm": 0.8, "taper_deg": 1.5 }
  ],
  "lug_pair": [
    { "width_mm": 22.0, "projection_mm": 5.5, "thickness_mm": 2.5,
      "pin_hole_dia_mm": 1.8, "fillet_r_mm": 0.5 },
    { "width_mm": 22.0, "projection_mm": 5.5, "thickness_mm": 2.5,
      "pin_hole_dia_mm": 1.8, "fillet_r_mm": 0.5 }
  ],
  "secondary_edge_r_mm": 0.2
}
```

### 예상 STEP 출력 치수 검증 기준

| 측정 항목 | 기대값 | 허용 오차 |
|-----------|--------|-----------|
| 전체 직경 (러그 제외) | 44.0 mm | ±0.01 mm |
| 전체 두께 | 10.5 mm | ±0.01 mm |
| 베젤 내경 | 37.0 mm (=44 − 2×3.5) | ±0.02 mm |
| 디스플레이 포켓 직경 | 35.5 mm | ±0.01 mm |
| 러그 포함 총 높이 | 44 + 2×5.5 = 55.0 mm | ±0.05 mm |
| 용두 샤프트 홀 직경 | 1.5 mm | ±0.005 mm |

검증 방법: `BRepGProp::VolumeProperties` + `Bnd_Box` 기반 바운딩박스 측정.
SHA-256 결정론 검증은 CI에서 수행 ([[process/test-strategy]]).

---

## DFM 게이트 참조 (WatchFrontModel 전용)

| 규칙 ID | 적용 스텝 | 기준 |
|---------|-----------|------|
| [[engine/dfm-rules#DFM-001]] | 스텝 2 after | 외곽 R ≥ 0.2 mm |
| [[engine/dfm-rules#DFM-008]] | 스텝 7, 8 after | 홀 직경 ≥ 0.8 mm |
| [[engine/dfm-rules#DFM-009]] | 스텝 6 after | 최소 살두께 ≥ 0.4 mm |

---

## 참고 링크

- [[engine/parametric-templates]] — ProductFrontModel 공통 기반, FeatureGraph 계약
- [[engine/dfm-rules]] — DFM 규칙 전체 카탈로그
- [[engine/feature-phone]] — 스마트폰 포팅 스펙
- [[engine/reverse-route]] — RE 파이프라인 → FeatureGraph 복원
- [[architecture/multi-document]] — 멀티 문서 및 incremental rebuild
- [[scope/milestones-and-krs#M1]] — 1차 납품 마일스톤 (시계 전면 케이스)
- [[process/test-strategy]] — SHA-256 결정론 CI 검증
- [[process/occt8-migration-cookbook]] — OCCT 8.0 API 마이그레이션 가이드
