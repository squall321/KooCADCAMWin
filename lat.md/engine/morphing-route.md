# Morphing Route (Demo)

> **경고 — DEMO 전용**: 이 루트는 1차 납품물이 아닙니다. 생산 설계에 사용하지 마십시오.
> 마일스톤 링크: [[scope/milestones-and-krs#M5]] (스킵 가능).
> 프로덕션 경로는 [[engine/parametric-templates]] 입니다.

---

## 개요

Morphing Route는 B-rep 외곽 셸의 제어점을 RBF(Radial Basis Function) 보간으로 변위시켜
"what-if" 윤곽 탐색을 지원합니다. 금속 케이스의 날카로운 엣지와 필렛이 많은 실제 제품 설계에는
적합하지 않으며, 오직 **±10% 이내의 부드러운 외곽선 변경** 탐색 시연용입니다.

---

## 적용 가능한 사용 사례

- 스마트폰 / 시계 케이스 외곽 곡률을 ±10% 범위 내에서 탐색하는 초기 스케치 단계
- 평탄하거나 완만한 NURBS 곡면이 지배적인 단순 형상의 "what-if" 시각화
- 디자이너가 파라메트릭 템플릿 수정 없이 빠른 실루엣 변화를 확인하고 싶을 때

---

## 사용 금지 상황 (Anti-uses)

- 날카로운 금속 코너, 직각 엣지, 깊은 포켓이 있는 시계/스마트폰 케이스 (RBF 보간이 엣지를 둥글게 만듦)
- 복잡한 필렛 체인이 있는 영역 (곡률 연속성 붕괴)
- 실제 제조용 출력물 생성 (치수 정밀도 보장 불가)
- 10% 초과 형상 변형 (B-rep 위상 일관성 파괴 위험)

---

## 파이프라인 스케치

```
사용자: 2D 탑뷰에서 윤곽 핸들 드래그
        │
        ▼
[핸들 좌표 수집] ── ΔP 벡터 리스트 (UI 레이어)
        │
        ▼
[RBF 보간] ── Wendland C2 커널, ε = 5 mm
  입력: 제어점 집합 {P_i}, 변위 {ΔP_i}
  출력: 연속 변위 필드 f: ℝ³ → ℝ³
        │
        ▼
[B-rep 제어점 변위 적용]
  Geom_BSplineSurface::SetPole(uIdx, vIdx, newPole)
  — 각 면(face)의 폴 격자를 변위 필드로 이동
        │
        ▼
[B-rep 재건]
  BRepBuilderAPI_MakeFace  (수정된 서피스 → 새 Face)
  BRep_Builder::Add        (Shell 재조립)
        │
        ▼
[유효성 검사]
  BRepCheck_Analyzer — IsValid() 실패 시 중단 후 경고
  ShapeFix_Shape     — 허용 오차 내 자동 복구 시도
        │
        ▼
[멀티뷰 표시] ── [[architecture/multi-view]]
  V3d_View V1: 원본 형상
  V3d_View V2: 모핑 결과
  V3d_View V3: 차이 오버레이 (편차 컬러맵, 0–5 mm 범위)
```

---

## 핵심 OCCT API

| API | 목적 | 비고 |
|-----|------|------|
| `Geom_BSplineSurface::SetPole` | 개별 제어점 이동 | u/v 인덱스 1-based |
| `Geom_BSplineSurface::SetWeight` | 유리 B-spline 가중치 조정 (선택) | 기본값 1.0 유지 권장 |
| `BRepBuilderAPI_MakeFace` | 수정 서피스로 새 Face 생성 | 허용 오차 `Precision::Confusion()` |
| `BRep_Builder::Add` | Shell/Compound 재조립 | |
| `BRepCheck_Analyzer` | 형상 유효성 전체 검사 | `IsValid()` → bool |
| `ShapeFix_Shape` | 자동 복구 (틈새, 노멀 방향) | `Perform()` 후 `Shape()` 추출 |
| `BRepGProp::SurfaceProperties` | 표면적 변화율 계산 | 변형 전후 비교용 |

---

## 실패 모드 및 한계

1. **엣지 열화**: 모서리 근방 폴이 이동하면 `G1` 연속성이 깨져 면 접합부가 벌어짐.
   - 탐지: `BRepCheck_Edge` → `InContext` 오류.
   - 대응: 해당 엣지 주변 핸들 비활성화.

2. **토폴로지 파괴**: 과도한 변위 시 Face가 자기 교차(self-intersection) 발생.
   - 탐지: `BRepCheck_Analyzer` `HasFaults()`.
   - 대응: 변위 크기를 자동으로 50% 감소 후 재시도 (최대 3회).

3. **RBF 조건수 폭발**: 핸들 간격이 `ε/10` 미만이면 선형 시스템이 발산.
   - 대응: 최소 핸들 간격을 UI에서 3 mm으로 강제.

4. **OCCT 재생성 비용**: 면 수가 많을수록 `MakeFace` 루프가 선형 증가.
   - 허용 범위: 200 Face 이하에서 ≤ 2 초 (측정 대상).

---

## 수용 기준 (Demo M5)

- 시각적으로 납득 가능한 윤곽 변화 (±10% 외형 범위 내)
- `BRepCheck_Analyzer::IsValid()` == true (변형 후)
- 원본 대비 표면적 편차 < 15%
- UI에서 핸들 배치 → 결과 표시까지 ≤ 3 초 (200 Face 기준)

---

## 권장 폴백: 프로덕션 전환

모핑 결과가 디자이너 승인을 받으면, 형상을 파라메트릭 루트로 귀환시켜야 합니다.

1. 모핑된 형상을 [[engine/reverse-route]] 파이프라인에 입력 (STL 경유).
2. 복원된 피처를 `FeatureGraph`에 수동 확인.
3. 이후 [[engine/parametric-templates]] 기반 치수 관리로 전환.

> 이 폴백 없이 모핑 결과물을 직접 제조에 사용하는 것은 **금지**입니다.

---

## 참고 링크

- [[architecture/multi-view]] — 3-패널 뷰 구성
- [[engine/parametric-templates]] — 프로덕션 파라메트릭 루트
- [[engine/reverse-route]] — 모핑 결과 → B-rep 복원 경로
- [[scope/milestones-and-krs#M5]] — 데모 마일스톤 정의
- [[glossary]] — RBF, B-rep, NURBS 용어 정의
