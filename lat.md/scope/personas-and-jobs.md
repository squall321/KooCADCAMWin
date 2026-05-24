# Personas and Jobs

KooCADCAM의 실제 사용자 유형과 그들이 도구에서 기대하는 작업(Job-to-be-Done)을 정의한다.
GUI 설계, 기능 우선순위, 테스트 시나리오 작성의 기준점으로 활용한다.

---

## Persona 1 — Mechanical Design Engineer (기구 설계 엔지니어)

### 프로필

- 경력 3–10년, CAD 툴(CATIA·SolidWorks·Fusion360) 사용 경험 있음
- 새로운 시계 또는 스마트폰 케이스 모델을 **처음부터(from scratch)** 설계
- 하루 작업의 60–70%를 치수 파라미터 조정 + 형상 검증에 사용

### Job-to-be-Done

> "JSON 스펙 한 장으로 베젤·크라운·러그 형상을 즉시 생성하고,
> 치수 변경 시 모델 전체가 자동 재생성되어야 한다.
> 3D 뷰에서 직접 회전·줌하며 형상 의도를 확인하고 싶다."

### 사용하는 기능

| 기능 | 참조 노드 |
|------|-----------|
| JSON → OCCT B-rep 파라메트릭 빌드 | [[engine/parametric-templates]] |
| Watch 피처 빌드 (베젤, 크라운, 러그 등) | [[engine/feature-watch]] |
| 단일 V3d_View 인터랙티브 뷰 | [[architecture/multi-view]] |
| STEP 익스포트 | [[engine/parametric-templates]] |

### 성공 지표

- Watch JSON spec → STEP 출력 ≤ 1.5초 (i7-13세대급 CPU 기준)
- 파라미터 20개 변경 후 재생성 시 크래시 없음
- V3d_View 회전 프레임레이트 ≥ 30 FPS (시계 케이스 단품 기준)

---

## Persona 2 — Production Engineer (생산 기술 엔지니어)

### 프로필

- 경력 5–15년, CNC 머시닝·DFM 검토·공정 설계 담당
- 설계 엔지니어가 넘긴 모델을 받아 **제조 가능성 검증** 수행
- 문제 발견 시 설계 엔지니어에게 수정 요청 (반복 루프)

### Job-to-be-Done

> "최소 코너 R, 박벽 두께, 공구 접근성 등 DFM 위반 항목을
> 자동 체크하고 JSON·HTML 리포트로 받고 싶다.
> 3축 CAM 경로가 실제로 모델과 간섭하지 않는지도 확인해야 한다."

### 사용하는 기능

| 기능 | 참조 노드 |
|------|-----------|
| DFM 자동 검증 (20+ 규칙) | [[engine/dfm-rules]] |
| JSON / HTML DFM 리포트 출력 | [[engine/dfm-rules]] |
| OpenCAMLib 툴패스 생성 | [[engine/cam-3axis-verify]] |
| OCCT BRepExtrema 간섭 체크 | [[engine/cam-3axis-verify]] |
| 멀티뷰에서 위반 부위 하이라이트 | [[architecture/multi-view]] |

### 성공 지표

- DFM 규칙 20개 이상 자동 실행, 위반 항목 전수 리포트
- CAM 간섭 감지 위양성(false positive) ≤ 5%
- 리포트 생성 시간 ≤ 30초 (시계 케이스 단품 기준)

---

## Persona 3 — Reverse-Engineering Specialist (역설계 전문가)

### 프로필

- 경력 3–8년, 3D 스캔 데이터·STL 처리 경험 있음
- 경쟁사 제품 또는 레거시 부품의 STL을 입력으로 받아
  **파라메트릭 STEP 모델로 변환**하는 것이 목표
- 완전 자동화 불가능하다는 현실을 알고 있으며, 도구에게 **합리적인 반자동화**를 기대

### Job-to-be-Done

> "STL을 불러오면 RANSAC으로 평면·원통·구 등 원시 형상을 자동 인식하고,
> 인식 결과를 내가 확인·수정한 뒤 파라메트릭 피처로 변환하고 싶다.
> 전체 공정의 60–75%는 자동화되고, 나머지는 내가 직접 수정한다."

### 사용하는 기능

| 기능 | 참조 노드 |
|------|-----------|
| STL 로드 + 정렬 | [[engine/reverse-route]] |
| RANSAC 원시 형상 인식 (≥80% 목표) | [[engine/reverse-route]] |
| 수동 피처 수정 UI | [[engine/reverse-route]] |
| 멀티도큐먼트: STL + STEP 병렬 표시 | [[architecture/multi-document]] |
| 컬러 오버레이 차이 시각화 | [[architecture/multi-document]] |

### 성공 지표

- RANSAC 원시 형상 인식률 ≥ 80% (시계 백 커버 STL 테스트셋 기준)
- 역설계 완료 모델의 STEP 익스포트 후 재임포트 시 형상 손실 없음
- 수동 수정 단계 수 ≤ 전체 피처의 30%

---

## Persona 4 — Project Lead (프로젝트 리드)

### 프로필

- 경력 10년+, 기술·비기술 양쪽 커뮤니케이션 담당
- 설계 리비전 간 변경 내용을 빠르게 파악해야 함
- CAD 툴 직접 조작보다 **결과 비교·검토**가 주 업무

### Job-to-be-Done

> "Rev A와 Rev B 모델을 나란히 띄우고, 어디가 얼마나 바뀌었는지
> 컬러맵으로 한눈에 보고 싶다.
> DFM 리포트도 두 버전 차이를 비교해서 볼 수 있어야 한다."

### 사용하는 기능

| 기능 | 참조 노드 |
|------|-----------|
| 2×2 멀티뷰 그리드 | [[architecture/multi-view]] |
| 멀티도큐먼트 (Rev A + Rev B 병렬 로드) | [[architecture/multi-document]] |
| 카메라 동기화 토글 | [[architecture/multi-view]] |
| 컬러 오버레이 Diff | [[architecture/multi-document]] |
| 도큐먼트 모델 구조 | [[architecture/document-model]] |

### 성공 지표

- 두 모델 로드 후 Diff 오버레이 표시까지 ≤ 5초
- 카메라 동기화 토글 시 4개 뷰포트 동시 업데이트 레이턴시 ≤ 100 ms
- 비기술직 리뷰어가 컬러맵 해석에 별도 교육 불필요 (직관적 색상 범례)

---

## Human-in-the-Loop Philosophy

**KooCADCAM은 100% 자동화를 목표로 하지 않는다.**

자동화가 실용적인 범위와 그 한계를 명시한다:

### 자동화 한계 선언

| 영역 | 자동화 범위 | 나머지 처리 |
|------|-------------|-------------|
| 파라메트릭 생성 | ~95% (JSON → STEP) | 비정형 요구사항 → 수동 피처 추가 |
| DFM 위반 감지 | 규칙 기반 100% 실행 | **위반 심각도 판단은 인간 책임** — 맥락에 따라 허용 가능한 위반 존재 |
| 역설계 원시 인식 (RANSAC) | 60–75% 실질적 자동화 | 나머지 25–40%는 전문가 수동 수정 필수 |
| CAM 간섭 감지 | BRepExtrema 기반 자동 | **가공 전략 결정은 생산 엔지니어 책임** |
| 형상 모핑 | DEMO 수준 (opt-in) | B-rep 모핑은 금속 샤프 에지에서 품질 열화 → 실제 작업에 부적합 |

### GUI = 검증·수정의 인터페이스

자동화 결과는 항상 GUI를 통해 인간에게 노출된다:

- DFM 위반 목록 → 사용자가 각 항목을 **Accept / Suppress / Flag** 처리
- RANSAC 인식 결과 → 원시 형상 패널에서 **확인 / 수정 / 삭제**
- CAM 간섭 리포트 → 공구·공정 파라미터를 **수동 조정** 후 재실행
- 멀티뷰 Diff → 리드가 **변경 승인 여부** 판단

이 철학은 [[engine/reverse-route#RANSAC 원시 인식]], [[engine/dfm-rules]], [[engine/cam-3axis-verify]] 전반에 걸쳐 구현 원칙으로 적용된다.

### 역설계의 현실적 기대치

역설계 전문가(Persona 3)는 RANSAC 결과의 20–40%를 수동 수정해야 한다.
이는 결함이 아니라 **설계 의도**다: 완전 자동화보다 빠른 반자동화가
경쟁사 제품 분석 실무에서 실질적으로 더 가치 있다.
STL→STEP 완전 자동화를 주장하는 경쟁 도구들은 복잡한 금속 케이스에서
형상 손실 또는 허위 피처를 생성하는 경우가 빈번하다.

참조: [[scope/product-catalog]], [[scope/milestones-and-krs]]
