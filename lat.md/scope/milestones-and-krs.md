# Milestones and Key Results

KooCADCAM 개발 로드맵. 각 마일스톤은 독립적으로 빌드 가능한 상태(green build)를 목표로 한다.
KR(Key Result)은 측정 가능한 수치 기준으로 작성한다.

---

## M1 — Watch 파라메트릭 + 단일뷰 GUI (~6주)

**목표**: OCCT 8.0 + Qt6 스택이 Windows에서 동작하고, 시계 케이스를 JSON에서 STEP으로 빌드하며 3D 뷰어에서 확인할 수 있다.

| # | Key Result | 측정 방법 |
|---|-----------|-----------|
| KR1 | OCCT 8.0.0 + Qt6 빌드 Debug/Release 모두 통과 (Windows 11, MSVC 2022) | CI 빌드 로그 green |
| KR2 | Watch JSON spec → STEP 출력 ≤ 1.5초 (i7-13세대급 CPU, 릴리스 빌드) | 타이머 측정 10회 평균 |
| KR3 | 단일 `V3d_View` 인터랙티브 회전/줌 동작 확인 | 수동 QA 체크리스트 |
| KR4 | 베젤·크라운·러그·스프링바홀 4개 피처 정상 생성 | OCCT BRep_Builder 검사 |
| KR5 | STEP 파일 제3자 CAD 툴(FreeCAD 또는 SolidWorks)에서 임포트 성공 | 수동 검증 |

주요 연계 노드: [[engine/parametric-templates]], [[engine/feature-watch]], [[architecture/multi-view]]

---

## M2 — 멀티뷰 + 멀티도큐먼트 (~4주)

**목표**: 4-패널 2×2 그리드 뷰어, 카메라 동기화, AIS 컨텍스트 분리, 두 모델 컬러 오버레이 Diff.

| # | Key Result | 측정 방법 |
|---|-----------|-----------|
| KR1 | 2×2 뷰포트 그리드 렌더링, 각 뷰 독립 `V3d_View` 인스턴스 | 렌더 스크린샷 검증 |
| KR2 | 카메라 동기화 토글: 4뷰 동시 업데이트 레이턴시 ≤ 100 ms | QElapsedTimer 측정 |
| KR3 | 두 문서 병렬 로드 시 `AIS_InteractiveContext` 충돌 없음 | Valgrind 또는 ASan 검사 |
| KR4 | 두 모델 간 컬러 오버레이 Diff (편차 ≥ 0.1 mm 시 색상 표시) | 알려진 편차 모델로 회귀 테스트 |
| KR5 | 레퍼런스 모델 + 생성 모델 동시 표시 시 FPS ≥ 30 | Qt `QElapsedTimer` 프레임 루프 |

주요 연계 노드: [[architecture/multi-view]], [[architecture/multi-document]], [[architecture/document-model]]

---

## M3 — DFM 자동 검증 (~4주)

**목표**: 20개 이상 DFM 규칙 자동 실행, JSON + HTML 리포트 출력.

| # | Key Result | 측정 방법 |
|---|-----------|-----------|
| KR1 | DFM 규칙 ≥ 20개 구현 및 시계 케이스 모델에 전수 적용 | 규칙 목록 코드 리뷰 |
| KR2 | 의도적 위반 모델 10종 → 위반 100% 감지 (false negative 0) | 회귀 테스트 스위트 |
| KR3 | 위양성(false positive) ≤ 5% (정상 모델 100개 테스트) | 자동 테스트 결과 |
| KR4 | JSON 리포트 + HTML 리포트 각각 정상 생성 | 파일 출력 검증 |
| KR5 | 리포트 생성 시간 ≤ 30초 (시계 케이스 단품 기준) | 타이머 측정 |

주요 연계 노드: [[engine/dfm-rules]], [[architecture/multi-view]]

---

## M4 — 역설계 1차 패스 (~5주)

**목표**: 시계 백 커버 STL → 정렬 → RANSAC 원시 형상 인식 ≥ 80% → 수동 피니싱 UI.

| # | Key Result | 측정 방법 |
|---|-----------|-----------|
| KR1 | STL 로드 + 기준면 자동 정렬 성공 (10개 테스트 파일 기준) | 정렬 오차 ≤ 0.5° |
| KR2 | RANSAC 원시 형상(평면·원통·구·토러스) 인식률 ≥ 80% | 라벨링된 테스트셋 F1-score |
| KR3 | 인식 결과 GUI에서 수동 수정·삭제·추가 가능 | 수동 QA |
| KR4 | 역설계 완료 모델 STEP 익스포트 후 재임포트 시 형상 손실 없음 | BRep diff 체크 |
| KR5 | 전체 공정(로드→인식→수정→익스포트) 숙련자 기준 ≤ 30분 | 사용자 테스트 |

주요 연계 노드: [[engine/reverse-route]], [[architecture/multi-document]]

---

## M5 — 모핑 데모 (skippable, opt-in) (~3주)

**목표**: 외곽 쉘 B-rep 모핑 기능을 데모 전용으로 제공. 실제 금속 케이스 설계 작업에는 권장하지 않음.

> **주의**: B-rep 모핑은 금속 샤프 에지에서 형상 열화가 발생한다.
> 이 마일스톤은 건너뛸 수 있으며(opt-in), 다른 마일스톤에 영향을 주지 않는다.

| # | Key Result | 측정 방법 |
|---|-----------|-----------|
| KR1 | 원통형 쉘 → 타원형 쉘 모핑 데모 동작 | 스크린샷 |
| KR2 | 모핑 결과 STEP 출력 가능 (형상 열화 경고 포함) | 파일 출력 + 경고 메시지 확인 |
| KR3 | 데모 모드 활성화/비활성화 플래그 동작 | 설정 파일 토글 테스트 |

주요 연계 노드: [[engine/morphing-route]]

---

## M6 — 3축 CAM 검증 (~4주)

**목표**: OpenCAMLib 툴패스 + OCCT BRepExtrema 간섭 리포트.

| # | Key Result | 측정 방법 |
|---|-----------|-----------|
| KR1 | OpenCAMLib 연동, 볼·평·불 엔드밀 3종 툴패스 생성 | 툴패스 G-code 출력 |
| KR2 | BRepExtrema 기반 공구-모델 간섭 감지, 양성 케이스 100% 보고 | 알려진 간섭 모델 테스트 |
| KR3 | 간섭 위치 멀티뷰 하이라이트 표시 | 수동 QA |
| KR4 | 전체 검증 실행 시간 ≤ 60초 (시계 케이스, 거친 패스 기준) | 타이머 측정 |
| KR5 | 완전 재고 제거 시뮬레이션(stock removal sim)은 범위 외 — 리포트에 명시 | 리포트 문구 확인 |

주요 연계 노드: [[engine/cam-3axis-verify]], [[engine/dfm-rules]]

---

## M7+ — 스마트폰 전체, ML 보조 역설계, ERP 연계

| 마일스톤 | 내용 | 의존 선행 마일스톤 |
|---------|------|------------------|
| M7a | Smartphone 피처 빌드 전체 구현 | M1, M3 |
| M7b | ML 보조 RANSAC (딥러닝 분류기 앙상블) | M4 |
| M7c | ERP/PLM 데이터 연동 (BOM 자동 생성) | M1 |
| M7d | 태블릿·이어버드 케이스 피처 추가 | M7a |

주요 연계 노드: [[engine/feature-phone]], [[scope/product-catalog]]

---

## Risk Register

상위 5개 리스크를 심각도(S) × 발생 가능성(P) 기준으로 정렬.

### R1 — OCCT 8.0 NCollection 사일런트 브레이킹 체인지 (S: 높음 / P: 중간)

OCCT 7.x → 8.x 마이그레이션 과정에서 `NCollection_DataMap`, `BRep_Builder` 등
내부 API가 조용히 변경될 수 있다. 컴파일은 되지만 런타임 형상 오류가 발생하는 패턴.

**대응**: [[process/occt8-migration-cookbook]] 작성 및 단위 테스트로 OCCT 출력 형상 검증.

### R2 — OpenCAMLib 라이선스 충돌 (S: 높음 / P: 낮음)

OpenCAMLib는 LGPL 2.1+이며 동적 링크 사용 시 상업적 배포 제약이 낮지만,
정적 링크 또는 수정 배포 시 소스 공개 의무가 발생할 수 있다.

**대응**: 동적 링크 유지, 법무 검토 완료 후 M6 진입. [[architecture/build-and-deps]] 참조.

### R3 — CGAL 라이선스 (S: 중간 / P: 중간)

RANSAC 구현에 CGAL을 사용할 경우 GPLv3 조항이 적용될 수 있다.
CGAL의 Shape Detection 모듈은 별도 라이선스 조건이 있다.

**대응**: RANSAC을 CGAL 없이 직접 구현하거나, MIT 라이선스의 대안 라이브러리(FLANN + 자체 RANSAC) 채택.

### R4 — 멀티 AIS 컨텍스트 성능 저하 (S: 중간 / P: 중간)

M2에서 문서당 독립 `AIS_InteractiveContext`를 사용할 때,
동시 4뷰 + 2문서 = 8개 렌더 패스가 발생하여 저사양 GPU에서 프레임 드롭 우려.

**대응**: `V3d_Viewer` 공유 + Context 분리 아키텍처 검토. [[architecture/multi-view]], [[architecture/multi-document]] 설계 단계에서 확정.

### R5 — STL 입력 품질 다양성 (S: 중간 / P: 높음)

경쟁사 제품 스캔 STL은 노이즈·홀·비매니폴드 면 등 품질 문제가 빈번하다.
RANSAC 인식률 ≥ 80% 목표는 고품질 STL 기준이며, 저품질 입력에서는 하락 가능.

**대응**: M4 KR1에 전처리 파이프라인(메시 클리닝) 포함. 테스트셋에 저품질 STL 20% 이상 포함. [[engine/reverse-route]] 전처리 섹션 명시.

---

## 마일스톤 요약 타임라인

```
M1 (6w) ──► M2 (4w) ──► M3 (4w) ──► M4 (5w) ──► M6 (4w) ──► M7+
                                          │
                                     M5 (3w, opt-in)
```

총 핵심 경로: 6 + 4 + 4 + 5 + 4 = **23주** (M5 제외)
M5 포함 시: 최대 26주 (M4와 병렬 가능)

페르소나-마일스톤 연계: [[scope/personas-and-jobs]]
제품군 범위: [[scope/product-catalog]]
전체 아키텍처: [[architecture/overview]]
테스트 전략: [[process/test-strategy]]
