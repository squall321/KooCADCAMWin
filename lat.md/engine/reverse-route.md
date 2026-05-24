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

## 참고 링크

- [[engine/parametric-templates]] — FeatureGraph 구조
- [[engine/dfm-rules]] — 복원 모델 DFM 검사 게이트
- [[architecture/multi-document]] — 소스 메시 Doc과 RE 결과 Doc 병렬 관리
- [[architecture/multi-view]] — 소스 / 결과 / 차이 3-패널 표시
- [[process/data-assets#License Matrix]] — CGAL GPL 라이선스 처리 방침
- [[engine/morphing-route]] — 복원 결과의 what-if 탐색 경로
- [[glossary]] — RANSAC, NURBS, B-rep, FeatureGraph 용어
