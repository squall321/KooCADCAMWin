---
require-code-mention: true
---

# Test Strategy

KooCADCAM 프로젝트의 품질 보증 전략을 정의한다. 단위 테스트부터 성능 게이트까지
단계별 테스트 피라미드를 구축하며, 모든 엔진 노드([[engine/parametric-templates]],
[[engine/morphing-route]], [[engine/reverse-route]], [[engine/dfm-rules]])와
빌드 인프라([[architecture/build-and-deps]])에 연동된다.

---

## 테스트 피라미드

```
           [ Performance Gate ]     ← 야간(nightly) CI
          [ Regression Tests   ]    ← 야간 CI
         [ Integration Tests    ]   ← PR CI
        [ Unit Tests              ] ← pre-commit + PR CI
```

의존성 관리는 vcpkg를 통해 `gtest` 포트로 GoogleTest를 공급한다([[architecture/build-and-deps]]).

---

## Unit Tests

### 범위
- 단일 함수 / 클래스 단위의 순수 로직 검증
- OCCT API를 직접 호출하는 코드는 헤드리스 환경에서 실행 가능해야 한다

### DFM 규칙 단위 테스트
[[engine/dfm-rules]]에 정의된 **모든 규칙**은 반드시 두 개의 픽스처(fixture) 쌍을 갖는다.

| 픽스처 종류 | 설명 |
|---|---|
| `violating` | 해당 규칙을 위반하는 `TopoDS_Shape` |
| `passing` | 해당 규칙을 통과하는 `TopoDS_Shape` |

```cpp
// 예시: 최소 살두께 규칙 (WallThicknessRule)
TEST(DfmWallThicknessRule, ViolatingShape) {
    TopoDS_Shape thin = MakeWallWithThickness(0.3);  // threshold: 0.4mm
    DfmResult result = WallThicknessRule{}.evaluate(thin);
    EXPECT_EQ(result.severity, DfmSeverity::Error);
}

TEST(DfmWallThicknessRule, PassingShape) {
    TopoDS_Shape ok = MakeWallWithThickness(0.5);
    DfmResult result = WallThicknessRule{}.evaluate(ok);
    EXPECT_LE(result.severity, DfmSeverity::Warning);
}
```

`require-code-mention: true` 프론트매터 규약에 따라 각 규칙 헤더 파일에
`// @lat: [[engine/dfm-rules#<RuleName>]]` 주석이 있어야 CI lint가 통과된다
(현재 `npx lat.md check` 비활성화 상태 — Node.js 22+ 도입 시 활성화,
[[process/coding-standards#lat.md annotations]] 참고).

---

## Determinism Tests

동일한 JSON 스펙 입력 → **바이트 단위 동일**한 canonical STEP 출력을 보장한다.

### 구현 방법
1. STEP 익스포터에 고정 타임스탬프 / UUID 시드 주입 옵션을 추가한다
2. 결과 STEP 바이트를 SHA-256으로 해시한다
3. 기준 해시를 `data/manifest.json`에 기록한다([[process/data-assets]])

```cpp
// STEP 익스포터 결정론적 설정 예시
STEPControl_Writer writer;
Interface_Static::SetCVal("write.step.timestamp", "1970-01-01T00:00:00");
// UUID 시드는 파라미터로 주입 (기본값 0x0)
```

### SHA-256 CI 검증
```bash
# CMake CTest 커스텀 커맨드 예시
sha256sum output/watch44_canonical.step > actual.sha256
diff expected/watch44_canonical.sha256 actual.sha256
```

---

## Integration Tests

### 황금(golden) JSON 스펙 목록

| 제품 | 스펙 파일 | 비고 |
|---|---|---|
| 워치 44mm (canonical) | `tests/golden/watch44_canonical.json` | 1차 타겟 |
| 워치 46mm (canonical) | `tests/golden/watch46_canonical.json` | 1차 타겟 |
| 스마트폰 6.1" | `tests/golden/phone61_canonical.json` | 2차 타겟 |
| 스마트폰 6.7" | `tests/golden/phone67_canonical.json` | 2차 타겟 |

### 출력 검증 기준

| 항목 | 허용 오차 | 측정 API |
|---|---|---|
| 부피(Volume) | ±0.1% vs baseline | `BRepGProp` / `GProp_GProps` |
| 표면적(Surface area) | ±0.1% vs baseline | `BRepGProp` |
| OBB 치수 (3축) | ±0.05mm vs baseline | `BRepBndLib::AddOBB` |
| DFM 규칙 최대 severity | ≤ Warning | [[engine/dfm-rules]] 전체 규칙 |

---

## Regression Tests

형상 변화 누적을 탐지하기 위해 `BRepExtrema_DistShapeShape`로 현재 형상과
베이스라인 형상 간 평균 거리를 계산한다.

```
평균 거리 ≤ 1µm (1.0e-3 mm)
```

```cpp
BRepExtrema_DistShapeShape dist(currentShape, baselineShape);
dist.Perform();
ASSERT_LE(dist.Value(), 1.0e-3);
```

베이스라인 STEP은 `data/` 트리에 SHA-256으로 관리된다([[process/data-assets]]).

---

## Performance Gate (KR1.2)

| 제품 | 생성 시간 상한 | 측정 대상 |
|---|---|---|
| 워치 (44mm canonical) | **≤ 1.5s** | JSON 파싱 완료 → STEP 출력 완료 |
| 스마트폰 6.1" | TBD (M2 목표 설정) | 동일 |

- CI 러너 사양: i7-13세대 동급 (최소 8코어, 16GB RAM)
- 측정 도구: `std::chrono::high_resolution_clock` + GoogleTest timing fixture
- 초과 시 빌드 FAIL (Warning이 아님)

---

## Multi-View Tests (M2+)

### 헤드리스 렌더링
- `QOpenGLWidget` offscreen 모드 사용 (Qt 6 `QOffscreenSurface`)
- AIS_InteractiveContext에 형상 표시 → `V3d_View::ToPixMap()` 캡처

### 픽셀 비교 기준

| 지표 | 통과 기준 |
|---|---|
| 픽셀 편차율 | ≤ 2% (전체 픽셀 기준) |
| 구조적 유사도 (SSIM) | ≥ 0.98 |

레퍼런스 스크린샷은 `tests/reference_renders/` 디렉토리에 PNG로 보관하며
해시를 `data/manifest.json`에 등록한다.

---

## CAM Verification Tests

| 입력 | 검증 대상 |
|---|---|
| 황금 공구 인벤토리 + 스톡(stock) 스펙 | 기대 간섭(interference) 개수 |
| 동일 입력 | 기대 미가공(unmachined) 면적 % |

상세 공구 경로 생성 로직은 [[engine/cam-3axis-verify]] 참고.

---

## Reverse-Engineering Tests

| 입력 | 검증 기준 |
|---|---|
| 황금 메시(golden mesh, `.ply` / `.stl`) | 기대 프리미티브 개수 |
| 동일 입력 | 인식률 ≥ 80% |

RE 파이프라인 세부 내용은 [[engine/reverse-route]] 참고.

---

## CI 구조

| 단계 | 트리거 | 실행 항목 |
|---|---|---|
| **pre-commit** | `git commit` | clang-format 검사, clang-tidy lint |
| **PR** | Pull Request 오픈/업데이트 | 단위 테스트 전체, 통합 테스트 전체 |
| **nightly** | 매일 02:00 UTC | 전체 회귀, 성능 게이트, 멀티-뷰 렌더링 비교 |

clang-format / clang-tidy 설정은 [[process/coding-standards]] 참고.
vcpkg 의존성 및 CMake 빌드 구성은 [[architecture/build-and-deps]] 참고.

---

## 크로스-링크 요약

- [[engine/parametric-templates]] — 파라메트릭 형상 생성 엔진
- [[engine/morphing-route]] — 모프 경로 엔진
- [[engine/reverse-route]] — 리버스 엔지니어링 파이프라인
- [[engine/dfm-rules]] — DFM 검증 규칙 전체 목록
- [[engine/cam-3axis-verify]] — CAM 연동 인터페이스
- [[process/data-assets]] — 황금 파일 관리 및 SHA-256 매니페스트
- [[process/coding-standards]] — clang-format, lat.md 주석 규약
- [[architecture/build-and-deps]] — vcpkg, CMake, CI 러너 스펙
