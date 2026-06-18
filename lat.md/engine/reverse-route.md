# Reverse Route (STL → STEP)

> **파이프라인 중심 문서**. 자동 인식률: **60–75%** (스마트폰급 금속 케이스 기준).
> 나머지는 인간 루프(human-in-the-loop) 확인이 필수입니다.
> 복원 결과물은 [[architecture/multi-document]] 에 새 Doc으로 로드됩니다.

---

## 파이프라인 다이어그램

```
STL 입력 (스캔 메시)
    │
    ▼
[Stage 1] Mesh Repair          ── libIGL (선택) / OCCT BRepMesh 아님
    │
    ▼
[Stage 2] Region Segmentation  ── CGAL Shape Detection (RANSAC)
    │  planes / cylinders / spheres / tori
    ▼
[Stage 3] Fillet Recognition   ── 곡률 대역 휴리스틱
    │
    ▼
[Stage 4] Free-form NURBS Fit  ── GeomAPI_PointsToBSplineSurface
    │
    ▼
[Stage 5] Trim + Sew           ── BRepBuilderAPI_MakeFace / Sewing / ShapeFix
    │
    ▼
[Stage 6] FeatureGraph 귀환    ── [[engine/parametric-templates]] (사용자 확인)
    │
    ▼
STEP 출력 / [[engine/dfm-rules]] 게이트
```

---

## Stage 1 — Mesh Repair (선택)

**입력**: 원시 스캔 STL (구멍, 중복 정점, 비다양체 엣지 가능)
**출력**: 워터타이트(watertight) 삼각형 메시

| 도구 | 용도 | 의존성 |
|------|------|--------|
| `libIGL` (igl::) | 구멍 채우기, 중복 제거, 법선 통일 | 헤더 전용, MIT |
| `igl::remove_duplicate_vertices` | 허용 오차 ε = 0.01 mm | |
| `igl::resolve_duplicated_faces` | 중복 삼각형 제거 | |
| `igl::copyleft::cgal::remesh_self_intersections` | 자기 교차 해소 | CGAL 필요 |

> OCCT `BRepMesh_IncrementalMesh`는 **입력 수리 도구가 아닙니다**. B-rep → 메시 방향으로만 사용합니다.

**실패 모드**: 스캔 밀도 부족 시 구멍 채우기 후 평탄화 오류 → 사용자에게 "재스캔 권장" 경고 표시.

---

## Stage 2 — Region Segmentation (CGAL Shape Detection)

**입력**: 워터타이트 삼각형 메시 + 법선 벡터
**출력**: 면 클러스터 리스트 (각 클러스터에 기하 타입 + 파라미터)

### CGAL API

```cpp
// RESERVED PATH: src/reverse/cgal_segment.cpp
CGAL::Shape_detection::Region_growing<
    TriangleMesh,
    CGAL::Shape_detection::Plane_map<TriangleMesh>,
    CGAL::Shape_detection::Cylinder_map<TriangleMesh>>
    shape_detection(mesh);
shape_detection.detect();
// 검출 대상: Plane, Cylinder, Sphere, Torus
```

### 라이선스 주의 사항

- CGAL 5.x: 헤더 전용 코어는 LGPL 3.0, Shape Detection 컴포넌트 일부는 **GPL 3.0**.
- **필수 조치**: GPL 컴포넌트는 별도 프로세스(dynamic-link sandbox)로 격리.
  - 빌드: vcpkg 포트 `cgal` + `cgal[shape_detection]` 활성화.
  - 라이선스 매트릭스: [[process/data-assets#License Matrix]] 참조.

### 파라미터 기본값

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| 최소 클러스터 면 수 | 50 삼각형 | 노이즈 제거 |
| 법선 허용 각도 | 15° | 평면 분류 기준 |
| 거리 허용 오차 | 0.1 mm | 포인트-평면 거리 |
| 반복 횟수 (RANSAC) | 1,000 | 정밀도/속도 균형 |

**실패 모드**: 복잡한 자유곡면 → 미분류 잔여 삼각형 → Stage 4로 이관.
자동 인식 실패 영역은 UI에서 **노란색** 하이라이트로 수동 확인 요청.

---

## Stage 3 — Fillet Recognition 휴리스틱

**입력**: Stage 2 분류된 면 클러스터 + 인접 그래프
**출력**: 필렛 엣지 목록 (반경 R, 인접 면 쌍)

### 알고리즘

1. 두 인접 평면/실린더 클러스터 경계를 탐색.
2. 경계 삼각형 스트립에서 단면 곡률 κ(s) 계산.
3. 일정한 κ 대역(표준편차 < 5%)이 연속되면 필렛으로 분류.
4. 필렛 반경 R = 1/κ_mean (허용 오차 ±0.05 mm).

**실패 모드**: 미세 챔퍼(< 0.3 mm) 및 장식용 보석 세팅(시계 베젤)은 인식 불가.
→ 사용자 수동 지정 → `FeatureGraph`에 `FilletFeature` 노드로 직접 추가.

---

## Stage 4 — Free-form NURBS Fit

**입력**: Stage 2 미분류 자유형 포인트 클라우드 패치
**출력**: `Geom_BSplineSurface` 객체 (패치별)

```cpp
// RESERVED PATH: src/reverse/nurbs_fit.cpp
GeomAPI_PointsToBSplineSurface fitter(
    pointArray,   // TColgp_Array2OfPnt
    3,            // degMin
    8,            // degMax
    GeomAbs_C2,   // continuity
    1e-3          // tolerance (mm)
);
Handle(Geom_BSplineSurface) surf = fitter.Surface();
```

**실패 모드**:
- 포인트 배열이 불균일할 경우 `GeomAPI_PointsToBSplineSurface` 수렴 실패.
  → `GeomPlate_BuildPlateSurface`로 폴백 (경계 조건 기반 패칭).
- 차수 8 초과 요구 시 패치 분할 후 재시도.

---

## Stage 5 — Trim + Sew

**입력**: Stage 2–4의 서피스 집합 + 경계 커브
**출력**: `TopoDS_Shell` 또는 `TopoDS_Solid`

```cpp
// RESERVED PATH: src/reverse/sew_solid.cpp
BRepBuilderAPI_MakeFace faceMaker(surface, wire, /*inside=*/true);
TopoDS_Face face = faceMaker.Face();

BRepBuilderAPI_Sewing sewer(/*tolerance=*/0.1);
sewer.Add(face);   // 모든 Face 추가 후
sewer.Perform();
TopoDS_Shape sewedShell = sewer.SewedShape();

ShapeFix_Shape fixer(sewedShell);
fixer.SetPrecision(0.1);
fixer.Perform();
TopoDS_Shape fixed = fixer.Shape();

BRepCheck_Analyzer checker(fixed);
// checker.IsValid() == false → Stage 6 이전에 사용자 개입 요청
```

**실패 모드**:
- 허용 오차 0.1 mm로도 틈새 봉합 불가 → 0.3 mm 재시도 후 경고 발행.
- 비다양체 엣지 잔류 → `ShapeFix_FixSmallFace` 보조 적용.

---

## Stage 6 — FeatureGraph 귀환 (The Bridge)

**목적**: 복원된 기하 피처를 `FeatureGraph`에 파라메트릭 노드로 연결.

### 절차

1. `ShapeAnalysis_FreeBounds`로 경계 루프 추출.
2. 피처 타입 추론: Plane → `BodyFace`, Cylinder → `Hole` 또는 `Boss`, Fillet → `FilletFeature`.
3. 각 추론 결과를 **Dock Widget "RE Feature Confirm"** 에 목록 표시.
4. 사용자: 승인 / 거부 / 타입 수정 후 "Commit" 클릭.
5. 승인된 피처 → [[engine/parametric-templates]] `FeatureGraph`에 노드 삽입.
6. 거부된 피처 → "수동 모델링 필요" 큐에 저장.

> 이 단계 없이 복원된 형상을 바로 제조에 사용하는 것은 **금지**입니다.

---

## 품질 기대치

| 형상 클래스 | 자동 인식률 | 주요 한계 |
|-------------|-------------|-----------|
| 스마트폰 케이스 (평면 + 단순 필렛) | **65–75%** | 미세 로고 각인, 안테나 슬롯 |
| 시계 케이스 (베젤 장식 포함) | **55–65%** | 인덱스 마커, 장식 조각 |
| 공통 실패 영역 | — | 날카로운 전이 엣지, 복합 필렛 체인 |

---

## ML 보조 인식 (M7+ 플레이스홀더)

- **BRepNet** (CVPR 2021) — B-rep 위상 그래프 기반 피처 분류.
- **UV-Net** — UV 파라미터화 메시 기반 세그멘테이션.
- 현재 스코프 외 (`[[scope/milestones-and-krs]]` M7 이후).
- 통합 시 Stage 2 CGAL 대체 또는 병렬 실행 후 교차 검증.

---

## compound-grammar

**B1.2 선언적 합성 인식.**  flat foreign-CAD cap 은 ~750 개 도메인 compound recognizer 를 추론 임계(0.7)
아래로 묶어 둔다 (느슨한 기하 fallback 이 아무 데서나 발화하기 때문).  generic
"wraps an atom" subsumption 으로 un-cap 하려는 시도는 측정으로 **반증**됨
([[engine/reverse-route#fpscan]]).  정통 해법은 **specific** 하다: 인식된 정밀
atom 들이 선언된 패턴으로 올바른 배치를 이룰 때만 compound 를 인식.

`re::recognizeCompounds(candidates)` 가 `re::analyze` 가 이미 복원한 정밀 atom 위에
grammar 룰을 평가(순수 함수, Workpiece 접근 없음).  현재 패턴:

| 패턴 | 조건 | 생성형 짝 |
|---|---|---|
| `bolt_circle_pattern` | 중심이 한 원 위에 있는 동일 홀 >=3 | [[engine/skills#bolt_circle_pattern]] |
| `linear_hole_array` | 공선·등피치 동일 홀 >=3 | [[engine/skills#linear_hole_array]] |
| `rectangular_hole_grid` | 채워진 cols×rows 격자의 동일 홀 >=6 | [[engine/skills#rectangular_hole_grid]] |
| `coaxial_step_bore` | 서로 다른 직경의 동심 full bore >=3 | [[engine/skills#coaxial_step_bore]] |

홀 패턴은 **equal-diameter · parallel-axis · REVOLUTION-COMPLETE** 드릴 홀에만
grounding (완전 회전 게이트가 부분 호/장식 절단을 배제).  인식된 compound 가
confidence >= 0.7 이고 그 face 집합이 구성 정밀 드릴의 face 를 strictly contain 하면,
[[engine/reverse-route#Recognizer]] 가 그 구성 드릴들을 **subsume** (kept 목록에서
제거)해 dispatchable 한 단일 편집 step 으로 만든다.  소스:
[[src/re/CompoundGrammar.hpp]] · [[src/re/CompoundGrammar.cpp]].
회귀: `tests/re/compound_grammar_test.cpp`.

---

## fpscan

**Un-cap 안전 스캔.**  B1.x 측정 도구.  ~750 개 도메인 compound recognizer 를 foreign CAD 에서 안전하게
un-cap 하려면, recognizer 별로 **해당 피처가 없는** 부품에서 높은 RAW confidence 로
오발화(spurious fire)하는지 알아야 한다 — 오발화하는 것은 un-cap 불가.

이 테스트는 foreign-equivalent 부품 패널(정밀 피처 합성 → STEP 라운드트립으로
메타데이터 제거)을 만들고, cap **OFF**(raw)와 **ON**(production)으로 `re::analyze`
를 돌려: (1) cap 이 비-정밀 후보를 모두 <= 0.5 로 강등함을 증명(load-bearing
invariant — cap 삭제 방지 가드), (2) skill 별 오발화 표를 출력 — 패널 전역에서
raw>=0.7 오발화 0건인 compound 가 grounded un-cap 후보.  이 측정이 generic
subsumption un-cap 을 반증하고 [[engine/reverse-route#compound-grammar]] 의 specific
grounding 으로 방향을 정했다.  회귀: `tests/re/fpscan_test.cpp`.

---

## Recognizer

**replay 경로의 게이트.**  `re::analyze` / `re::inferProcessPlan` 이 복원한 후보를
dispatchable 한 `process::StepInvocation` 으로 정규화하는 단계.  핵심 함수
`liftRecoveredStep(step)` 은 recovered `chamfer_edge` / `fillet_edge` step 의 datum
params(`edges_at_z_mm`, `tolerance_mm`)를 채우고 edge-op 을 곧바로 replay 가능하게
만든다.  소스: [[src/re/Recognizer.cpp]].  compound subsumption 은
[[engine/reverse-route#compound-grammar]] 참조.

### chamfer 보존 (silent-corruption 수정)

`chamfer_size_mm` 은 베벨 스트립에서 **측정된** 값이다(`chamfer_edge` 의 accept band
[0.05, 50] mm).  따라서 측정된 3 mm chamfer 는 replay 시에도 3 mm 여야 한다.

- **제거**: 측정값을 무조건 2.0 mm 위면 0.5 mm 로 끌어내리던 `2.0→0.5` clamp.  이
  clamp 가 정당하게 큰 measured chamfer(3 mm → 0.5 mm)를 **소리 없이 깎아** RE
  round-trip 을 망가뜨렸다.
- **현재**: sub-0.1 mm 값만 0.1 mm(DFM floor)로 바닥 처리.  upward rewrite 없음.

> **원칙 — clamp/gate 는 crash-class 또는 true-defect 입력만 고친다.**  유효한
> measured / structural 값을 절대 silent 하게 corrupt 하지 않는다.  measured 값을
> upward 로 덮어쓰는 clamp 는 거의 항상 버그다(같은 교훈:
> [[engine/skills#param-clamp]], [[engine/skills#auto_rib_between_two_walls]],
> [[engine/dfm-rules#DFM-018]]).  회귀: `tests/re/recognizer_test.cpp`.

---

## 참고 링크

- [[engine/parametric-templates]] — FeatureGraph 구조
- [[engine/dfm-rules]] — 복원 모델 DFM 검사 게이트
- [[architecture/multi-document]] — 소스 메시 Doc과 RE 결과 Doc 병렬 관리
- [[architecture/multi-view]] — 소스 / 결과 / 차이 3-패널 표시
- [[process/data-assets#License Matrix]] — CGAL GPL 라이선스 처리 방침
- [[engine/morphing-route]] — 복원 결과의 what-if 탐색 경로
- [[glossary]] — RANSAC, NURBS, B-rep, FeatureGraph 용어
