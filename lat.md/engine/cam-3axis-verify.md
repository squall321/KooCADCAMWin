# CAM 3-Axis Verification

> **스코프 엄격 제한**: 이 모듈은 **CAM 솔루션이 아닙니다**.
> 목적은 *가공 가능성 보고서(Machining Feasibility Report)* 생성입니다.
> 업스트림 게이트: [[engine/dfm-rules]] 통과 권장 (필수 아님).
> 마일스톤: [[scope/milestones-and-krs#M6]].

---

## 비목표 (Non-Goals)

- 실제 NC 프로그램 생성 (회전축, 드웰, 룩어헤드 없음)
- 체적 스톡 제거 시뮬레이션 (칩 생성, 절삭력 계산 없음)
- 4축 / 5축 경로 생성 또는 검증
- 공구 수명 / 열 해석
- 실제 포스트프로세서 (기계별 G-code 방언 지원 없음)
- 피드율 최적화 및 spindle 룩업

---

## 입력 명세

| 입력 | 타입 | 비고 |
|------|------|------|
| OCCT 솔리드 | `TopoDS_Solid` | [[engine/dfm-rules]] 통과 권장 |
| 스톡 OBB + 오프셋 | `Bnd_OBB` + `Standard_Real` | 균일 오프셋 δ (기본 2.0 mm) |
| 공구 목록 | JSON 배열 | 직경, 길이, 타입 (ballnose / endmill) |
| 공칭 이송/회전 | `Standard_Real` | 주기 시간 추정용, 검증 미사용 |
| XY 그리드 간격 | `Standard_Real` | 기본 0.5 mm (해상도/속도 균형) |

---

## Pipeline

### Stage 1 — OCCT 솔리드 테셀레이션

**목적**: OCL이 소비할 STL 표면 생성

```cpp
// RESERVED PATH: src/cam/tessellate.cpp
BRepMesh_IncrementalMesh mesher(
    shape,
    /*linearDeflection=*/0.05,   // mm
    /*isRelative=*/false,
    /*angularDeflection=*/0.00873, // 0.5°, rad
    /*isParallel=*/true            // 멀티코어 활성화
);
mesher.Perform();
// 이후 BRep_Tool::Polygon3D / Triangulation 으로 STL 직렬화
```

**실패 모드**: `mesher.IsDone() == false` → 형상 검사(`BRepCheck_Analyzer`) 재실행 후 사용자에게 솔리드 수정 요청.

---

### Stage 2 — OCL 공구경로 생성

**목적**: waterline finish + drop-cutter 샘플 포인트 계산

```
OCL (OpenCAMLib) — LGPL 2.1+, 파이썬 바인딩 또는 C++ 직접 링크
// RESERVED PATH: src/cam/ocl_bridge.cpp
```

| OCL 컴포넌트 | 용도 |
|--------------|------|
| `DropCutter` | XY 그리드 각 점에서 Z 최저 접촉 높이 계산 |
| `WaterlineFinish` | 등고선 기반 마무리 경로 |
| `STLSurf` | Stage 1 STL 로드 |
| `CylCutter` / `BallCutter` | 공구 형상 (엔드밀 / 볼노즈) |

**파라미터**:
- XY 그리드 간격: 기본 0.5 mm (공구 직경의 10% 권장).
- Waterline 등고선 간격: 기본 0.2 mm Z-step.
- 스톡 OBB는 `Bnd_OBB`에서 추출한 축 정렬 범위 + δ 오프셋으로 정의.

**실패 모드**: OCL이 STL 파일을 파싱 불가 → Stage 1 출력 메시 수리 후 재시도.

---

### Stage 3 — G-code 포스트프로세스 (기본)

**출력**: ISO 6983 선형 보간 전용 G-code (검증 참조용)

```
G00 X{} Y{} Z{}   ; 급속 이송
G01 X{} Y{} Z{} F{feed}  ; 절삭 이송
M02               ; 프로그램 종료
```

**제한**:
- 직선 보간(G01)만 출력. 원호(G02/G03), 캔드사이클(G81 등) 없음.
- 회전축(A/B/C) 없음.
- 기계별 방언 변환 없음 (Fanuc / Siemens / Heidenhain 미지원).

---

### Stage 4 — 간섭 검사

**목적**: 각 경로 샘플에서 공구-파트 거리 계산 → 충돌/미가공 분류

```cpp
// RESERVED PATH: src/cam/interference_check.cpp

// 공구 엔벨로프 솔리드 생성 (BallCutter 예시)
gp_Ax2 toolAxis(gp_Pnt(x, y, z), gp_Dir(0,0,1));
BRepPrimAPI_MakeSphere sphereMaker(toolAxis, toolRadius);
BRepPrimAPI_MakeCylinder cylMaker(toolAxis, toolRadius, toolLength);
// BRepAlgoAPI_Fuse 로 결합 → 볼노즈 솔리드

// 거리 계산
BRepExtrema_DistShapeShape distCalc(toolSolid, partSolid);
distCalc.Perform();
Standard_Real d = distCalc.Value();
```

**분류 기준**:

| 거리 d | 분류 | 시각화 색상 |
|--------|------|-------------|
| d < -tol (tol=0.05 mm) | **COLLISION** | 빨강 |
| \|d\| ≤ tol | **CONTACT** (정상 절삭) | 초록 |
| d > δ (스톡 오프셋) | **UNMACHINED** | 파랑 |
| tol < d ≤ δ | 정상 여유 | 없음 |

**실패 모드**:
- `distCalc.IsDone() == false` → 해당 샘플 점 건너뜀, 누락 카운트 별도 집계.
- 공구 엔벨로프 생성 실패 (반경 ≤ 0) → 공구 목록 입력 오류로 사용자에게 반환.

---

### Stage 5 — 보고서 생성

**출력 형식**: JSON (기계 처리) + HTML (사람 검토)

#### JSON 스키마 (핵심 필드)

```json
{
  "tool_id": "T01",
  "tool_type": "ballnose",
  "diameter_mm": 4.0,
  "collision_count": 3,
  "collision_locations": [
    {"x": 12.3, "y": 45.6, "z": -2.1}
  ],
  "unmachined_percent": 8.4,
  "contact_sample_count": 14820,
  "total_sample_count": 16000,
  "naive_cycle_time_sec": 342.5,
  "missing_sample_count": 0
}
```

#### 주기 시간 추정

```
naive_cycle_time = Σ(segment_length) / feed_rate_mm_per_sec
```

- 이는 **하한 추정값**입니다 (가감속, 코너 감속 미포함).
- 보고서에 "추정 오차 ±30%" 명시 필수.

#### HTML 보고서 섹션

1. 요약 테이블 (공구별 충돌/미가공/주기시간)
2. 충돌 포인트 좌표 목록 (상위 N, 기본 N=10)
3. 미가공 영역 비율 히스토그램 (면 클러스터별)
4. 비고: "DFM 재검토 권장" (충돌 > 0 또는 미가공 > 5% 시)

---

## 시각화

멀티뷰 [[architecture/multi-view]] 내 전용 `V3d_View` 사용:

| 객체 | OCCT 표현 | 색상/스타일 |
|------|-----------|-------------|
| 공구경로 폴리라인 | `AIS_Shape` (선분 시퀀스) | 회색, 선폭 1px |
| 충돌 포인트 | `AIS_Point` 글리프 | 빨강, 크기 5px |
| 미가공 면 영역 | `AIS_Shape` (Face 서브셋) | 파랑 반투명 |
| 정상 접촉 경로 | `AIS_Shape` | 초록 |

---

## 워크플로 통합

```
OCCT 솔리드 (통과 [[engine/dfm-rules]])
    │
    ▼
CAM 3-Axis Verify 모듈
    │
    ├─ 충돌 감지 → 설계 수정 요청 → [[engine/parametric-templates]]
    ├─ 미가공 과다 → 공구 재선택 또는 접근각 재검토
    └─ 통과 → 보고서 첨부 후 외부 CAM 소프트웨어로 인계
```

---

## 의존성 요약

| 라이브러리 | 버전 | 라이선스 | 용도 |
|-----------|------|----------|------|
| OpenCAMLib | 최신 stable | LGPL 2.1+ | 공구경로 생성 |
| OCCT | 8.0.0 | LGPL 2.1 | 테셀레이션, 간섭 계산, 시각화 |

---

## 참고 링크

- [[engine/dfm-rules]] — 업스트림 검사 게이트
- [[scope/milestones-and-krs#M6]] — CAM 검증 마일스톤
- [[architecture/multi-view]] — 전용 뷰 패널 구성
- [[engine/parametric-templates]] — 충돌 발견 시 설계 수정 경로
- [[glossary]] — OBB, BRepExtrema, OCL, drop-cutter 용어
