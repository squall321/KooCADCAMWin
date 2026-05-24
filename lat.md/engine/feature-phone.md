# PhoneFrontModel

스마트폰 전면 메탈 케이스의 파라메트릭 모델. Master Spec §6.3 구조를 보존하되
`ProductFrontModel` CRTP 기반([[engine/parametric-templates]])으로 재구성한다.

> **계획 등급 스펙** — 이 문서는 M2/M7+ 착수 전 설계 의도 보존을 목적으로 한다.
> 빌드-나우 수준의 구현 세부는 시계 구현([[engine/feature-watch]])이 안정화된 후 확정한다.
> 1차 납품 대상은 시계([[scope/milestones-and-krs#M1]]). 스마트폰은 M2 이후다.

---

## 시계와의 핵심 차이점

| 항목 | WatchFrontModel | PhoneFrontModel |
|------|-----------------|-----------------|
| 기본 바디 형상 | 원형 / 라운드 사각 퍽 | 세로형 라운드 직사각형 블록 |
| 디스플레이 포켓 | 소형 원형/사각, 단일 | 대형 직사각형, 화면 대부분 차지 |
| 카메라 | 측면 작은 구멍 (없을 수도) | 후면 멀티 카메라 모듈 패턴 (1–4개) |
| 버튼 | 소수 (용두 + 1–2개) | 전원·볼륨 (최대 4개, 종축 분포) |
| 포트 | 없음 또는 시계 특수 단자 | 하단 USB-C/Lightning — 종방향 타원형 |
| 치수 범위 | 직경 30–55 mm | 폭 67–78 mm, 높이 140–165 mm |
| 살두께 임계 | 0.4 mm | 0.4 mm (동일, DFM-009) |
| 카메라 데코 링 | 해당 없음 | DFM-007: 링 비율 규칙 적용 |
| 디스플레이 평탄도 | 단일 면 | DFM-013: 대형 포켓 평탄도 기준 |
| 카메라 평행도 | 해당 없음 | DFM-018: 멀티 카메라 간 평행도 |

---

## 클래스 API (계획 등급)

> **예약 경로** — 아래 헤더 블록의 실제 소스 위치는 `[[src/engine/PhoneFrontModel.hpp]]`입니다.

```cpp
// [[src/engine/PhoneFrontModel.hpp]]
// C++17 / OCCT 8.0.0  —  계획 등급 (M2/M7+ 착수 전 확정)
#pragma once

#include "ProductFrontModel.hpp"
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>
#include <TopoDS_Shape.hxx>
#include <TopExp_Explorer.hxx>
#include <STEPControl_Writer.hxx>
#include <nlohmann/json.hpp>
#include <vector>
#include <array>
#include <string>

namespace koo {

// ── 스마트폰 전용 스펙 구조체 ─────────────────────────────────

/// 카메라 홀 하나의 스펙 (후면 멀티 카메라 모듈)
struct CameraSpec {
    double offset_x_mm;     ///< 모듈 중심 X 오프셋 (케이스 중심 기준, mm)
    double offset_y_mm;     ///< 모듈 중심 Y 오프셋 (케이스 중심 기준, mm)
    double hole_dia_mm;     ///< 카메라 렌즈 홀 직경 (mm)
    double deco_ring_dia_mm;///< 데코 링 외경 (mm). DFM-007: ring/hole 비율 규칙
    double depth_mm;        ///< 홀 깊이 (mm), 보통 관통
    bool   through;         ///< true = 관통홀
};

/// 측면 버튼 스펙 (스마트폰 전원/볼륨 버튼)
struct PhoneButton {
    std::string side;       ///< "left" | "right"
    double height_pct;      ///< 케이스 총 높이 대비 위치 비율 [0.0, 1.0]
    double length_mm;       ///< 버튼 길이 (mm)
    double width_mm;        ///< 버튼 폭 (mm)
    double depth_mm;        ///< 포켓 깊이 (mm), DFM-009: 최소 살두께 ≥ 0.4
    double taper_deg;       ///< 포켓 측벽 기울기 (°)
};

/// 하단 포트 스펙 (USB-C / Lightning)
struct PortSpec {
    std::string type;       ///< "usb_c" | "lightning" | "custom"
    double width_mm;        ///< 포트 구멍 폭 (mm)
    double height_mm;       ///< 포트 구멍 높이 (mm)
    double offset_x_mm;     ///< 중심 X 오프셋 (케이스 중심 기준, mm)
    double depth_mm;        ///< 포트 깊이 (mm) — 관통 또는 맹홀
    double corner_r_mm;     ///< 포트 홀 모서리 반경 (mm)
};

/// 디스플레이 포켓 스펙 (대형 직사각형)
struct DisplayPocketSpec {
    double width_mm;        ///< 포켓 폭 (mm)
    double height_mm;       ///< 포켓 높이 (mm)
    double depth_mm;        ///< 포켓 깊이 (mm)
    double corner_r_mm;     ///< 포켓 모서리 반경 (mm)
    double glass_offset_mm; ///< 유리 단차 (음수 = 함몰)
    /// DFM-013: 포켓 바닥 평탄도 기준 (flatness tolerance, mm)
    double flatness_tol_mm;
};

// ── PhoneFrontModel ────────────────────────────────────────────

class PhoneFrontModel : public ProductFrontModel<PhoneFrontModel> {
public:
    explicit PhoneFrontModel();

    // ── 빌드 스텝 (Master Spec §6.3 구조 보존) ───────────────

    /// 스텝 1: 기본 바디 — 세로형 라운드 직사각형 블록
    /// @param width_mm       케이스 폭 (mm), 범위 [67, 78]
    /// @param height_mm      케이스 높이 (mm), 범위 [140, 165]
    /// @param thickness_mm   케이스 두께 (mm), 범위 [6.5, 9.5]
    /// OCCT: BRepPrimAPI_MakeBox → BRepFilletAPI_MakeFillet (4개 수직 엣지)
    StepResult buildBase(const nlohmann::json& spec);

    /// 스텝 2: 외곽 코너 필렛
    /// @param r_top_mm    상면/하면 테두리 필렛 (mm)
    /// @param r_side_mm   측면-하단 접합 필렛 (mm)
    /// 시계 스텝 2와 동일 로직 — [[engine/feature-watch#스텝 2]] 재사용 가능
    /// OCCT: BRepFilletAPI_MakeFillet
    StepResult applyCornerRadius(const TopoDS_Shape& in,
                                 const nlohmann::json& spec);

    /// 스텝 3: 디스플레이 포켓 — 대형 직사각형 안착 홈
    /// 시계 대비 차이: 직사각형 단면, 포켓 면적이 케이스 면적의 70–85%
    /// DFM-013: 포켓 바닥 평탄도 검증 필수
    /// OCCT: BRepBuilderAPI_MakeFace (직사각형 와이어) →
    ///       BRepPrimAPI_MakePrism 또는 BRepOffsetAPI_ThruSections →
    ///       BRepAlgoAPI_Cut
    StepResult buildDisplayPocket(const TopoDS_Shape& in,
                                  const DisplayPocketSpec& dpSpec,
                                  const nlohmann::json& spec);

    /// 스텝 4: 카메라 홀 배열 (멀티 카메라 모듈 패턴)
    /// DFM-007: 데코 링 비율 (deco_ring_dia / hole_dia) 범위 검증
    /// DFM-018: 멀티 카메라 간 평행도 (법선 벡터 편차 ≤ 기준치)
    /// OCCT: BRepPrimAPI_MakeCylinder × n → BRepAlgoAPI_Cut
    ///       데코 링: BRepPrimAPI_MakeCylinder 환형 → BRepAlgoAPI_Cut (함몰) 또는 Fuse (돌출)
    ///       평행도 검증: BRepAdaptor_Surface → 법선 벡터 비교
    StepResult addCameraHoles(const TopoDS_Shape& in,
                              const std::vector<CameraSpec>& cameras,
                              const nlohmann::json& spec);

    /// 스텝 5: 버튼 포켓 (전원·볼륨, 최대 4개)
    /// 시계와 구조 동일, 좌우 종축 배분이 다름
    /// OCCT: BRepPrimAPI_MakeBox 또는 BRepOffsetAPI_ThruSections →
    ///       BRepAlgoAPI_Cut
    StepResult addButtonFeatures(const TopoDS_Shape& in,
                                 const std::vector<PhoneButton>& buttons,
                                 const nlohmann::json& spec);

    /// 스텝 6: 포트 홀 (USB-C / Lightning — 하단 종방향 타원형)
    /// 시계와의 핵심 차이: 측면 단순 원형이 아닌 하단 종방향 타원/라운드렉트
    /// OCCT: BRepBuilderAPI_MakeWire (라운드렉트 단면) →
    ///       BRepBuilderAPI_MakeFace → BRepPrimAPI_MakePrism →
    ///       BRepAlgoAPI_Cut
    StepResult addPortFeatures(const TopoDS_Shape& in,
                               const PortSpec& port,
                               const nlohmann::json& spec);

    /// 스텝 7: DFM 전체 검증 (validateDesign — Master Spec §6.3 명칭 보존)
    /// DFM-007, DFM-009, DFM-013, DFM-018 포함
    /// OCCT: BRepExtrema_DistShapeShape, BRepAdaptor_Surface,
    ///       BRepCheck_Analyzer, BRepGProp::SurfaceProperties
    DFMReport validateDesign(const TopoDS_Shape& shape,
                             const nlohmann::json& fullSpec);
};

} // namespace koo
```

---

## 빌드 시퀀스 개요

### 스텝 1 — `buildBase` (라운드 직사각형)

```cpp
// 기본 박스 생성
BRepPrimAPI_MakeBox box(width_mm, height_mm, thickness_mm);
// 4개 수직 엣지 필렛 (초기 외곽 R)
BRepFilletAPI_MakeFillet fillet(box.Shape());
// ... TopExp_Explorer로 수직 엣지 선택 후 AddEdge
```

시계 `buildBase`와 달리 높이/폭 비율이 약 2:1이며 세로 방향이 지배적이다.
원점은 `gp::Origin()`, 높이 방향 Y축 (`gp::DY()`)으로 고정하여 결정론 보장.

### 스텝 3 — `buildDisplayPocket` (시계와의 핵심 차이)

스마트폰 디스플레이 포켓은 케이스 폭의 90% 이상을 차지하는 대형 직사각형이다.
`BRepBuilderAPI_MakeWire`로 라운드렉트 단면 와이어를 구성하고
`BRepPrimAPI_MakePrism`으로 깊이 방향 압출 후 `BRepAlgoAPI_Cut`.

평탄도 검증(DFM-013):
```cpp
// 포켓 바닥 면의 평탄도 측정
BRepAdaptor_Surface adaptor(bottom_face);
// Geom_Plane 여부 및 편차 확인
// 편차 > flatness_tol_mm 이면 DFM-013 실패
```

### 스텝 4 — `addCameraHoles` (멀티 카메라)

카메라 홀 간격 패턴은 제조사 모듈 레이아웃을 반영한다.
평행도 검증(DFM-018): 각 카메라 홀 축 벡터(`gp_Dir`) 간 각도 편차가 기준치 이내인지 확인.

```cpp
// 평행도 검증 스케치
for (size_t i = 1; i < cameras.size(); ++i) {
    gp_Dir axis0 = computeHoleAxis(cameras[0]);
    gp_Dir axisI = computeHoleAxis(cameras[i]);
    double angDeg = axis0.Angle(axisI) * 180.0 / M_PI;
    if (angDeg > DFM018_MAX_ANGLE_DEG) {
        report.failures.push_back("DFM-018: camera " + std::to_string(i)
            + " axis deviation " + std::to_string(angDeg) + "°");
    }
}
```

### 스텝 6 — `addPortFeatures` (하단 종방향 포트)

USB-C 구멍은 단순 원형이 아닌 **라운드렉트** 단면이다.
`BRepBuilderAPI_MakeWire`로 직선 2개 + 반원 2개를 조합한 단면 와이어 생성:

```cpp
// 라운드렉트 단면 와이어 생성 (포트 홀)
double halfW = port.width_mm / 2.0;
double halfH = port.height_mm / 2.0;
double r = port.corner_r_mm;
// 4개 선분 + 4개 호(arc)를 BRepBuilderAPI_MakeEdge로 생성
// BRepBuilderAPI_MakeWire로 조합
// BRepPrimAPI_MakePrism(wire_face, depth_vec) → 절삭 툴
```

---

## JSON Schema 스켈레톤 (Draft 2020-12)

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://koo-cadcam.local/schemas/phone-spec/v0.1.0",
  "title": "PhoneFrontSpec",
  "description": "스마트폰 전면 메탈 케이스 파라메트릭 빌드 스펙 (계획 등급 v0.1.0). 단위: mm, degrees.",
  "type": "object",
  "required": [
    "schema_version", "product_name",
    "base", "corner_radius", "display_pocket"
  ],
  "properties": {
    "schema_version": {
      "type": "string",
      "const": "0.1.0",
      "description": "계획 등급. M2 착수 시 1.0.0으로 승격 예정."
    },
    "product_name": {
      "type": "string", "maxLength": 64
    },
    "noise_seed": {
      "type": "integer", "minimum": 0, "default": 0,
      "description": "결정론 보장용 PRNG 시드."
    },
    "base": {
      "type": "object",
      "required": ["width_mm", "height_mm", "thickness_mm"],
      "properties": {
        "width_mm": {
          "type": "number", "minimum": 67.0, "maximum": 78.0,
          "description": "케이스 폭 (mm). 현세대 스마트폰 범위."
        },
        "height_mm": {
          "type": "number", "minimum": 140.0, "maximum": 165.0,
          "description": "케이스 높이 (mm)."
        },
        "thickness_mm": {
          "type": "number", "minimum": 6.5, "maximum": 9.5,
          "description": "케이스 두께 (mm)."
        },
        "initial_corner_r_mm": {
          "type": "number", "minimum": 1.0, "maximum": 5.0, "default": 2.5,
          "description": "buildBase 내 1차 수직 코너 필렛 반경 (mm)."
        }
      }
    },
    "corner_radius": {
      "type": "object",
      "required": ["r_top_mm", "r_side_mm"],
      "properties": {
        "r_top_mm":  { "type": "number", "minimum": 0.2, "maximum": 5.0 },
        "r_side_mm": { "type": "number", "minimum": 0.2, "maximum": 3.0 }
      }
    },
    "display_pocket": {
      "type": "object",
      "description": "대형 직사각형 디스플레이 안착 포켓.",
      "required": ["width_mm", "height_mm", "depth_mm", "corner_r_mm"],
      "properties": {
        "width_mm": {
          "type": "number",
          "description": "포켓 폭 (mm). base.width_mm의 85–95% 권장."
        },
        "height_mm": {
          "type": "number",
          "description": "포켓 높이 (mm). base.height_mm의 75–88% 권장."
        },
        "depth_mm": {
          "type": "number", "minimum": 0.3, "maximum": 2.0
        },
        "corner_r_mm": {
          "type": "number", "minimum": 0.5, "maximum": 6.0,
          "description": "포켓 모서리 반경 (mm). 외곽 corner_radius와 연동."
        },
        "glass_offset_mm": {
          "type": "number", "minimum": -1.0, "maximum": 0.5, "default": -0.1
        },
        "flatness_tol_mm": {
          "type": "number", "minimum": 0.005, "maximum": 0.1, "default": 0.02,
          "description": "DFM-013: 포켓 바닥 평탄도 허용 오차 (mm)."
        }
      }
    },
    "cameras": {
      "type": "array",
      "description": "후면 멀티 카메라 홀 배열.",
      "minItems": 1, "maxItems": 4,
      "items": {
        "type": "object",
        "required": ["offset_x_mm", "offset_y_mm", "hole_dia_mm", "depth_mm"],
        "properties": {
          "offset_x_mm":     { "type": "number" },
          "offset_y_mm":     { "type": "number" },
          "hole_dia_mm": {
            "type": "number", "minimum": 5.0, "maximum": 20.0,
            "description": "카메라 렌즈 홀 직경 (mm)."
          },
          "deco_ring_dia_mm": {
            "type": "number",
            "description": "데코 링 외경 (mm). DFM-007: deco/hole 비율 [1.2, 2.0]."
          },
          "depth_mm":  { "type": "number", "minimum": 0.5, "maximum": 10.0 },
          "through":   { "type": "boolean", "default": true }
        }
      }
    },
    "buttons": {
      "type": "array",
      "description": "측면 버튼 포켓 배열 (전원, 볼륨 등).",
      "maxItems": 4,
      "items": {
        "type": "object",
        "required": ["side", "height_pct", "length_mm", "width_mm", "depth_mm"],
        "properties": {
          "side":        { "type": "string", "enum": ["left", "right"] },
          "height_pct":  { "type": "number", "minimum": 0.0, "maximum": 1.0 },
          "length_mm":   { "type": "number", "minimum": 3.0, "maximum": 25.0 },
          "width_mm":    { "type": "number", "minimum": 1.5, "maximum": 6.0 },
          "depth_mm":    { "type": "number", "minimum": 0.3, "maximum": 2.0,
                           "description": "DFM-009: 잔류 살두께 ≥ 0.4 mm." },
          "taper_deg":   { "type": "number", "minimum": 0, "maximum": 5, "default": 0 }
        }
      }
    },
    "port": {
      "type": "object",
      "description": "하단 포트 홀 (USB-C / Lightning). 시계에 없는 스마트폰 전용 피처.",
      "required": ["type", "width_mm", "height_mm", "depth_mm"],
      "properties": {
        "type": {
          "type": "string", "enum": ["usb_c", "lightning", "custom"]
        },
        "width_mm":     { "type": "number", "minimum": 6.0, "maximum": 12.0 },
        "height_mm":    { "type": "number", "minimum": 2.0, "maximum": 5.0 },
        "offset_x_mm":  { "type": "number", "default": 0.0,
                          "description": "하단 중심 기준 X 오프셋. 0 = 대칭 중앙." },
        "depth_mm":     { "type": "number", "minimum": 1.0, "maximum": 10.0 },
        "corner_r_mm":  { "type": "number", "minimum": 0.3, "maximum": 2.0, "default": 0.5 }
      }
    },
    "secondary_edge_r_mm": {
      "type": "number", "minimum": 0.1, "maximum": 1.0, "default": 0.2,
      "description": "2차 엣지 라운드오버 반경 (mm)."
    }
  }
}
```

---

## 시계 공통 기반 재사용 전략

`PhoneFrontModel`은 `ProductFrontModel` CRTP를 통해 다음을 시계와 공유한다.

| 공유 항목 | 위치 |
|-----------|------|
| `FeatureGraph` 실행 엔진 | [[engine/parametric-templates#공통 기반 클래스]] |
| DFM 훅 등록/실행 메커니즘 | [[engine/parametric-templates#DFM 게이트 삽입]] |
| `exportSTEP` (결정론적 헤더) | [[engine/parametric-templates#결정론적 STEP 출력]] |
| `applyCornerRadius` 구현 | [[engine/feature-watch#스텝 2]] — 로직 동일, 파라미터만 다름 |
| `addButtonFeatures` 구조 | [[engine/feature-watch#스텝 6]] — SideButton → PhoneButton 타입 변환 |

`buildBase`, `buildDisplayPocket`, `addCameraHoles`, `addPortFeatures`는 스마트폰 전용 구현이다.

---

## DFM 게이트 참조 (PhoneFrontModel 전용)

| 규칙 ID | 적용 스텝 | 기준 요약 |
|---------|-----------|-----------|
| [[engine/dfm-rules#DFM-007]] | 스텝 4 after | 카메라 데코 링 비율 (deco/hole) 범위 검증 |
| [[engine/dfm-rules#DFM-009]] | 스텝 5 after | 버튼 포켓 최소 살두께 ≥ 0.4 mm |
| [[engine/dfm-rules#DFM-013]] | 스텝 3 after | 디스플레이 포켓 바닥 평탄도 기준 |
| [[engine/dfm-rules#DFM-018]] | 스텝 4 after | 멀티 카메라 홀 간 축 평행도 편차 |

---

## 마이그레이션 노트 (Master Spec §6.3 보존)

Master Spec §6.3 `FrontMetalModel` 메서드 명칭은 `PhoneFrontModel`에서 다음과 같이 대응된다.

| Master Spec §6.3 | PhoneFrontModel | 비고 |
|------------------|-----------------|------|
| `buildBase(w, h, t)` | `buildBase(spec)` | JSON spec으로 파라미터 통합 |
| `applyCornerRadius(r)` | `applyCornerRadius(in, spec)` | 상면/측면 분리 |
| `buildDisplayPocket(DisplaySpec)` | `buildDisplayPocket(in, dpSpec, spec)` | 평탄도 DFM 추가 |
| `addCameraHoles(vector<CameraSpec>)` | `addCameraHoles(in, cameras, spec)` | DFM-007/018 추가 |
| `addButtonFeatures(vector<ButtonSpec>)` | `addButtonFeatures(in, buttons, spec)` | 좌우 side 구분 |
| `addPortFeatures(PortSpec)` | `addPortFeatures(in, port, spec)` | 라운드렉트 단면 추가 |
| `validateDesign()` | `validateDesign(shape, fullSpec)` | DFM 게이트와 통합 |

Master Spec 원본: `d:\KooCADCAM\smartphone_metal_cad_project.md` (편집 금지).

---

## M2 착수 전 결정 사항 (플래그)

다음 항목은 M2 착수 전 확정이 필요하다. 의존 노드 담당자와 협의할 것.

1. **카메라 모듈 레이아웃 표준** — 세로 배열 vs L자 배열; JSON spec `camera_layout` 필드 추가 여부
2. **포트 홀 단면 라이브러리** — USB-C IEC 62680 치수 표준화 여부
3. **디스플레이 포켓 모서리 R 연동** — 외곽 `corner_radius.r_top_mm`과 `display_pocket.corner_r_mm` 의존 관계
4. **DFM-013 평탄도 기준값** — [[engine/dfm-rules]] 담당자와 기준값 확정
5. **RE 파이프라인 스마트폰 특화** — [[engine/reverse-route]]가 직사각형 포켓 인식을 지원하는지 확인

---

## 참고 링크

- [[engine/parametric-templates]] — ProductFrontModel 공통 기반, CRTP 설계
- [[engine/feature-watch]] — WatchFrontModel (공유 기반 구현체, 참조 우선)
- [[engine/dfm-rules#DFM-007]] — 카메라 데코 링 비율 규칙
- [[engine/dfm-rules#DFM-013]] — 디스플레이 포켓 평탄도 기준
- [[engine/dfm-rules#DFM-018]] — 카메라 홀 간 평행도 규칙
- [[engine/reverse-route]] — RE 파이프라인, FeatureGraph 복원
- [[architecture/multi-document]] — 멀티 문서 incremental rebuild
- [[scope/milestones-and-krs#M1]] — 1차 납품 (시계) — 스마트폰 선행 완료 조건
- [[process/occt8-migration-cookbook]] — OCCT 8.0 API 마이그레이션
