# DFM Rule Catalog

## 정책 서문 (Policy Preamble)

이 파일은 KooCADCAM 프로젝트의 **DFM(Design for Manufacturability) 규칙 카탈로그 v1.0**이다. 각 규칙은 파라메트릭/RE/모프 경로가 생성한 `TopoDS_Shape`에 대해 자동 평가 가능한 수치 기준을 정의한다.

### 버전 관리

카탈로그는 Semantic Versioning을 따른다. 현재 버전: **v1.0**. 규칙 추가 시 마이너 버전 증가, 기준값 변경 시 메이저 버전 증가. 각 엔진 노드는 카탈로그 버전을 소비 시점에 명시해야 한다.

### 게이팅 정책

각 엔진 스텝([[engine/parametric-templates#DFM Gate Insertion]])은 생성 직후 DFM 검사를 실행한다. 검사는 세 가지 심각도 레벨로 분류된다.

| Severity | 동작 |
|---|---|
| **Error** | STEP 내보내기를 기본적으로 차단. 사용자가 명시적 override 플래그를 설정하지 않으면 다운스트림 노드가 실행되지 않음. |
| **Warning** | 리포트에 주석 추가. 파이프라인은 계속 진행되지만 CAM 검증 단계([[engine/cam-3axis-verify]])에서 재확인함. |
| **Info** | 로그 파일에만 기록. UI에 표시하지 않음. |

### 단위 테스트

모든 규칙은 양성(pass) 피처 픽스처와 음성(fail) 피처 픽스처 각 1개 이상을 가진다. 테스트 전략은 [[process/test-strategy#DFM Rule Tests]]를 참조한다.

---

## 요약 테이블 (Summary Table)

| ID | 이름 | 스코프 | 심각도 | 기준값 |
|---|---|---|---|---|
| DFM-001 | Minimum wall thickness | Both | Error | ≥ 0.4 mm (watch: ≥ 0.35 mm) |
| DFM-002 | Minimum camera/sensor hole diameter | Both | Error | ≥ φ 0.8 mm |
| DFM-003 | Minimum hole-to-hole distance | Both | Error | ≥ 1.5 mm |
| DFM-004 | Minimum corner radius | Both | Error | ≥ R 0.2 mm |
| DFM-005 | No undercuts (3-axis only) | Both (optional) | Warning | 드래그 방향 Z+ 기준 |
| DFM-006 | Button pocket depth limit | Both | Error | ≤ 0.8 mm OR ≤ thickness − 0.3 mm |
| DFM-007 | Camera deco-ring outer-diameter ratio | Phone | Warning | 0.05 ≤ od/width ≤ 0.22 |
| DFM-008 | Crown cavity concentricity | Watch | Error | ≤ 0.05 mm |
| DFM-009 | Bezel minimum width | Watch | Error | ≥ 0.6 mm |
| DFM-010 | Side-button chamfer R consistency | Both | Warning | within ±20 % of exterior R |
| DFM-011 | Anti-knife edge | Both | Error | adjacent-face angle ≥ 5° |
| DFM-012 | Forging/extrusion direction consistency | Both | Warning | aligned with Z axis |
| DFM-013 | Display pocket flatness | Phone | Error | ≤ 0.02 mm |
| DFM-014 | Speaker grille minimum pin thickness | Both | Error | ≥ 0.25 mm |
| DFM-015 | Tap-hole minimum surrounding material | Both | Error | ≥ tap diameter × 1.5 |
| DFM-016 | Anodizing mask region closed | Both | Error | polygon must close |
| DFM-017 | Outer-edge curvature continuity | Both | Warning | G1 minimum (G2 preferred) |
| DFM-018 | Camera deco-ring parallelism to top face | Phone | Warning | ≤ 0.1° |
| DFM-019 | Engraved text/logo minimum stroke width | Both | Error | ≥ 0.15 mm |
| DFM-020 | All shells closed (no open shells) | Both | Error | `TopoDS_Shell::IsClosed()` true |
| DFM-021 | Zero self-intersection | Both | Error | `BRepAlgoAPI` self-cut empty |
| DFM-022 | OBB Z aligned with thickness axis | Both | Warning | thickness < width AND thickness < height |
| DFM-023 | Watch lug span symmetry | Watch | Warning | lug-to-lug span asymmetry ≤ 0.05 mm |
| DFM-024 | Speaker mesh hole roundness | Both | Warning | hole circularity ≥ 0.85 |
| DFM-025 | Anodizing color-uniformity edge buffer | Both | Info | ≥ 0.3 mm from outer edge to color-change boundary |

---

## 규칙 상세 (Rules Detail)

---

### DFM-001: Minimum Wall Thickness

- **Scope**: Both
- **Severity**: Error
- **Criterion**: 스마트폰 케이스 벽 두께 ≥ 0.4 mm; 시계 케이스 ≥ 0.35 mm. 알루미늄 합금(6061/7075) CNC 가공 시 0.4 mm 미만은 가공 중 진동으로 인한 채터링 및 파손 위험이 높고, 아노다이징 후 강도 불균일 발생. 시계 케이스는 고밀도 재료(Ti, SUS316)로 허용 최소값을 0.35 mm까지 낮춤.
- **OCCT APIs**: `BRepExtrema_DistShapeShape`, `TopExp_Explorer`, `BRepAdaptor_Surface`, `GProp_GProps` + `BRepGProp`
- **Algorithm sketch**:
  - `TopExp_Explorer`로 모든 `TopoDS_Face` 열거.
  - 각 face 쌍에 대해 `BRepExtrema_DistShapeShape`를 실행하여 최소 거리 d 획득.
  - 두 face가 서로 마주 보는(opposing normal 내적 < −0.8) 경우만 벽 두께 후보로 분류.
  - d가 scope별 임계값 미만이면 해당 face 쌍을 위반 리스트에 추가.
  - 결과를 `BRep_Builder`를 통해 마커 `TopoDS_Vertex`로 시각화 준비.
- **Failure example**: 시계 측면 crown 슬롯 주변 벽이 0.28 mm로 생성된 경우.
- **Pass example**: 스마트폰 카메라 섬 주변 벽 0.45 mm — Error 없이 통과.
- **Linked from**: [[engine/feature-watch#Crown Slot]], [[engine/feature-phone#Camera Island]]

---

### DFM-002: Minimum Camera/Sensor Hole Diameter

- **Scope**: Both
- **Severity**: Error
- **Criterion**: φ ≥ 0.8 mm. 표준 엔드밀 최소 지름 φ0.8 mm (ISO 표준 소경 공구). 그 미만은 특수 공구 필요, 공구 파손 위험 급증. 렌즈 센서 홀의 경우 광학 사양상 φ0.8 미만은 의미 없음.
- **OCCT APIs**: `BRepAdaptor_Surface`, `BRepGProp`, `TopExp_Explorer`, `Geom_Circle` (from `BRep_Tool::Surface`)
- **Algorithm sketch**:
  - `TopExp_Explorer`로 모든 원통/원형 면(cylindrical face) 탐지.
  - `BRepAdaptor_Surface`로 `GeomAbs_Cylinder` 타입 확인.
  - `GeomAdaptor_Surface::Cylinder()`에서 반지름 r 추출, diameter = 2r.
  - 관통홀 여부: 같은 축을 공유하는 두 원형 엣지가 반대 면에 존재하는지 확인.
  - diameter < 0.8 mm인 관통홀을 위반 마킹.
- **Failure example**: 스마트폰 전면 스피커 망 홀을 φ0.6 mm으로 설계한 경우.
- **Pass example**: 카메라 렌즈홀 φ1.2 mm — 기준 초과로 통과.
- **Linked from**: [[engine/feature-phone#Speaker Grille]], [[engine/feature-phone#Camera Island]]

---

### DFM-003: Minimum Hole-to-Hole Distance

- **Scope**: Both
- **Severity**: Error
- **Criterion**: 인접 홀 간 최소 거리 ≥ 1.5 mm (홀 엣지 간 거리). 알루미늄에서 홀 간격이 1.5 mm 미만이면 가공 시 버(burr) 및 재료 파열 위험. 아노다이징 시 전류 집중으로 표면 불량 발생.
- **OCCT APIs**: `BRepExtrema_DistShapeShape`, `TopExp_Explorer`, `BRep_Tool`
- **Algorithm sketch**:
  - 모든 원형/원통 엣지(hole edge)를 `TopExp_Explorer`로 수집하여 리스트 H 생성.
  - H의 모든 쌍(i, j)에 대해 `BRepExtrema_DistShapeShape` 실행.
  - 반환된 최소 거리 d가 1.5 mm 미만이면 쌍(i, j)을 위반 리스트에 추가.
  - O(n²) 비용을 줄이기 위해 OBB 사전 필터링 적용 (`Bnd_OBB`).
  - 위반 쌍에 경고 선분(`BRep_Builder` edge)을 생성하여 시각화.
- **Failure example**: 스피커 그릴 홀 패턴에서 홀 간격 0.9 mm로 설계.
- **Pass example**: 홀 피치 2.0 mm (edge-to-edge 1.2 → diameter 조정 후 1.5 mm 이상).
- **Linked from**: [[engine/feature-phone#Speaker Grille]], [[engine/feature-watch#Crown Slot]]

---

### DFM-004: Minimum Corner Radius

- **Scope**: Both
- **Severity**: Error
- **Criterion**: 내부 코너 반경 ≥ R 0.2 mm. 표준 볼 엔드밀 최소 R 0.2 mm. R 0.2 미만은 모서리 응력 집중 및 공구 파손. 포켓 바닥 코너 기준.
- **OCCT APIs**: `BRepAdaptor_Curve`, `TopExp_Explorer`, `GCPnts_AbscissaPoint`, `GeomLProp_CLProps`
- **Algorithm sketch**:
  - `TopExp_Explorer`로 concave 엣지(내부 코너 엣지) 탐색: 인접 face 법선의 외적 방향으로 내/외부 판별.
  - 각 엣지를 `BRepAdaptor_Curve`로 래핑.
  - `GeomLProp_CLProps`로 곡률 κ 샘플링(20 points), R = 1/κ_max.
  - R < 0.2 mm인 엣지를 위반 목록 등록.
  - 직선 엣지(R → ∞)는 skip.
- **Failure example**: 시계 케이스 러그 루트 코너가 R 0.1 mm 샤프 코너로 생성.
- **Pass example**: 포켓 코너 R 0.3 mm — 기준 통과.
- **Linked from**: [[engine/feature-watch#Lug Root]], [[engine/parametric-templates#Corner Blend]]

---

### DFM-005: No Undercuts (3-Axis Only)

- **Scope**: Both (optional — 5축 캠 경로 사용 시 비활성화 가능)
- **Severity**: Warning
- **Criterion**: 드래그 방향 Z+ 기준, 언더컷 면적 = 0. 3축 CNC 가공 시 Z+ 방향으로 공구가 접근할 수 없는 면은 언더컷으로 분류. [[engine/cam-3axis-verify]]에서 최종 확인.
- **OCCT APIs**: `BRepAdaptor_Surface`, `TopExp_Explorer`, `gp_Dir`, `BRep_Tool`
- **Algorithm sketch**:
  - Z+ 드래그 방향 벡터 d = gp_Dir(0,0,1) 정의.
  - `TopExp_Explorer`로 모든 face 순회.
  - `BRepAdaptor_Surface`의 샘플 포인트에서 법선 n 계산.
  - n · d < −0.01 (즉, 법선이 Z− 방향 성분 우세)이고 면이 상부에서 가려진 경우 언더컷.
  - 가려짐 판별: ray casting (Z+ 방향 ray, `BRepIntCurveSurface_Inter` 활용).
  - 언더컷 면 면적 합산, 0 이상이면 Warning.
- **Failure example**: 시계 케이스 측면에 역방향 테이퍼 형상(−2° 드래프트) 적용.
- **Pass example**: 모든 측면 드래프트 +1° 이상 — Warning 없음.
- **Linked from**: [[engine/cam-3axis-verify#Undercut Detection]]

---

### DFM-006: Button Pocket Depth Limit

- **Scope**: Both
- **Severity**: Error
- **Criterion**: 버튼 포켓 깊이 ≤ 0.8 mm OR ≤ (총 두께 − 0.3 mm) 중 작은 값. 너무 깊은 포켓은 측벽 두께가 DFM-001을 위반하거나, 버튼 클리어런스 과다로 방진방수 등급 저하.
- **OCCT APIs**: `BRepExtrema_DistShapeShape`, `BRepBndLib`, `Bnd_Box`, `TopExp_Explorer`
- **Algorithm sketch**:
  - 버튼 포켓 face를 식별: face 법선이 측면 방향(|n·Z| < 0.3)이고 면적이 1–50 mm² 범위.
  - 해당 face에서 `Bnd_Box`로 Z 범위 추출 → pocket depth.
  - 총 두께는 전체 shape의 `BRepBndLib::Add()` 결과 Z span.
  - depth > min(0.8, thickness − 0.3) 이면 Error.
  - 복수 버튼 포켓 모두 독립적으로 검사.
- **Failure example**: 전원 버튼 포켓 깊이 1.1 mm, 총 두께 1.0 mm.
- **Pass example**: 볼륨 버튼 포켓 깊이 0.6 mm, 총 두께 1.2 mm → limit = min(0.8, 0.9) = 0.8 → 통과.
- **Linked from**: [[engine/feature-phone#Side Button]], [[engine/feature-watch#Crown Slot]]

---

### DFM-007: Camera Deco-Ring Outer-Diameter Ratio

- **Scope**: Phone
- **Severity**: Warning
- **Criterion**: 0.05 ≤ od / width ≤ 0.22 (od = 데코링 외경, width = 케이스 폭). 비율이 너무 작으면 데코링이 가공 불가 세부 요소가 되고, 너무 크면 케이스 구조 강성을 해침.
- **OCCT APIs**: `BRepAdaptor_Surface`, `BRepGProp`, `GProp_GProps`, `BRepBndLib`
- **Algorithm sketch**:
  - 카메라 데코링 face 식별: 원통 face이며 법선 방향이 Z+에 근접(|n·Z| > 0.9).
  - `BRepAdaptor_Surface::Cylinder().Radius()` 로 반지름 r 획득, od = 2r.
  - 케이스 전체 `Bnd_Box`에서 width (X 또는 Y 중 큰 값) 획득.
  - ratio = od / width, 범위 벗어나면 Warning.
  - 다중 카메라 배치 시 각 링 독립 검사.
- **Failure example**: 울트라 와이드 카메라 데코링 od = 35 mm, width = 75 mm → ratio = 0.47 → 초과.
- **Pass example**: 메인 카메라 데코링 od = 12 mm, width = 75 mm → ratio = 0.16 → 통과.
- **Linked from**: [[engine/feature-phone#Camera Island]]

---

### DFM-008: Crown Cavity Concentricity

- **Scope**: Watch
- **Severity**: Error
- **Criterion**: 크라운 구멍(crown cavity) 축과 케이스 측면 원통 축의 거리 ≤ 0.05 mm. 크라운 편심은 조립 불량 및 방수 O-링 밀봉 실패.
- **OCCT APIs**: `BRepAdaptor_Surface`, `gp_Cylinder`, `gp_Ax1`, `TopExp_Explorer`
- **Algorithm sketch**:
  - 크라운 캐비티: 측면 방향(|n·X| > 0.8 또는 |n·Y| > 0.8) 원통 face 중 깊이 > 1 mm.
  - 각 face의 `BRepAdaptor_Surface::Cylinder().Axis()` 로 축선 L₁ 추출.
  - 케이스 외부 원통 축 L₂ 추출(동일 방법).
  - 두 무한 직선(Ax1) 간 최단 거리 계산: `gp_Lin::Distance(gp_Lin)`.
  - 거리 > 0.05 mm 이면 Error.
- **Failure example**: 크라운 홀이 0.08 mm 편심으로 가공 → O-링 밀봉 불가.
- **Pass example**: 크라운 홀 편심 0.03 mm — 기준 통과.
- **Linked from**: [[engine/feature-watch#Crown Slot]]

---

### DFM-009: Bezel Minimum Width

- **Scope**: Watch
- **Severity**: Error
- **Criterion**: 베젤 최소 폭 ≥ 0.6 mm. 회전 베젤/고정 베젤 모두 해당. 0.6 mm 미만은 베젤 클릭 기구 수용 불가, 충격 시 파손.
- **OCCT APIs**: `BRepExtrema_DistShapeShape`, `TopExp_Explorer`, `BRepAdaptor_Surface`
- **Algorithm sketch**:
  - 베젤 상면(top ring face, 법선 Z+, 환형 형상)을 식별.
  - 내경 엣지와 외경 엣지를 `TopExp_Explorer`로 분리.
  - 두 엣지 간 `BRepExtrema_DistShapeShape`로 최소 거리 d 계산.
  - d < 0.6 mm 이면 Error.
  - 360° 전체에 걸쳐 최소값 탐색(균일하지 않은 베젤 대응).
- **Failure example**: 스켈레톤 다이얼 베젤 폭 0.4 mm 설계.
- **Pass example**: 스포츠 워치 베젤 폭 1.2 mm — 통과.
- **Linked from**: [[engine/feature-watch#Bezel Profile]]

---

### DFM-010: Side-Button Chamfer R Consistency

- **Scope**: Both
- **Severity**: Warning
- **Criterion**: 사이드 버튼 챔퍼/라운드 R이 케이스 외부 R과 ±20% 이내. 불일치 시 반사 및 아노다이징 줄무늬 불균일.
- **OCCT APIs**: `BRepAdaptor_Curve`, `GeomLProp_CLProps`, `TopExp_Explorer`
- **Algorithm sketch**:
  - 외부 케이스 엣지 중 convex 라운드를 `TopExp_Explorer`로 수집, 평균 R_ext 계산.
  - 버튼 포켓 엣지(버튼 면 경계)의 챔퍼 R_btn 각각 계산.
  - |R_btn − R_ext| / R_ext > 0.20 이면 Warning.
  - 챔퍼 직선(R=0)은 별도 DFM-011에서 처리, 여기서는 제외.
- **Failure example**: 케이스 외부 R 0.5 mm, 버튼 챔퍼 R 0.1 mm → 80% 차이.
- **Pass example**: 케이스 외부 R 0.5 mm, 버튼 챔퍼 R 0.45 mm → 10% 차이 → 통과.
- **Linked from**: [[engine/feature-phone#Side Button]], [[engine/feature-watch#Crown Slot]]

---

### DFM-011: Anti-Knife Edge (No Sharp Blade Edges)

- **Scope**: Both
- **Severity**: Error
- **Criterion**: 인접 두 face가 이루는 각도 ≥ 5°. 날카로운 칼날 형상(< 5°)은 가공 후 버 발생 및 상해 위험, 아노다이징 시 색상 집중 결함.
- **OCCT APIs**: `BRepAdaptor_Surface`, `TopExp_Explorer`, `gp_Dir`, `BRep_Tool`
- **Algorithm sketch**:
  - 모든 공유 엣지(shared edge)를 `TopExp_Explorer`로 순회.
  - 엣지 중점에서 양측 face의 법선 n₁, n₂ 계산 (`BRepAdaptor_Surface` UV 파라미터 사용).
  - 다면각(dihedral angle) θ = acos(n₁ · n₂) (또는 180° − θ 로 외각).
  - θ < 5° (외각 기준 175°< 통합 각도) 이면 Error.
  - 탄젠트 연속 엣지(G1)는 θ ≈ 0°이므로 별도 처리(DFM-017 참조).
- **Failure example**: 케이스 상단 엣지를 0° 드래프트로 설계 → 칼날 형상.
- **Pass example**: 상단 엣지에 0.15 mm 챔퍼 추가 → 내각 45° → 통과.
- **Linked from**: [[engine/parametric-templates#Edge Blend]], [[engine/feature-watch#Bezel Profile]]

---

### DFM-012: Forging/Extrusion Direction Consistency

- **Scope**: Both
- **Severity**: Warning
- **Criterion**: 단조/압출 방향이 Z축(두께 축)과 정렬됨. 단조 유선 방향이 비틀린 경우 내부 결함 및 아노다이징 줄무늬 불균일.
- **OCCT APIs**: `BRepGProp`, `GProp_GProps`, `BRepBndLib`, `Bnd_OBB`
- **Algorithm sketch**:
  - `Bnd_OBB`로 형상의 주 관성 축(Principal axis) 계산.
  - OBB의 Z축(최소 반경 방향 = 두께)과 글로벌 Z 벡터 간 각도 α = acos(Z_obb · Z_global).
  - α > 5° 이면 압출 방향 불일치 Warning.
  - 단조 공정 메타데이터(작업 지시서)와 교차 참조 필요 → 자동 검사는 형상 기반.
- **Failure example**: 케이스가 X축으로 15° 회전된 상태로 모델링 → 압출 방향 불일치.
- **Pass example**: 케이스 두께 방향 = Z → OBB Z축 정렬 → 통과.
- **Linked from**: [[engine/parametric-templates#Blank Orientation]]

---

### DFM-013: Display Pocket Flatness

- **Scope**: Phone
- **Severity**: Error
- **Criterion**: 디스플레이 포켓 바닥 면의 평탄도(flatness) ≤ 0.02 mm. OLED/LCD 디스플레이 본딩 요구사항. 0.02 mm 초과 시 접착제 두께 불균일 → 뉴턴링 또는 들뜸 발생.
- **OCCT APIs**: `BRepAdaptor_Surface`, `BRepGProp`, `gp_Pln`, `GeomAPI_ProjectPointOnSurf`
- **Algorithm sketch**:
  - 디스플레이 포켓 바닥 face 식별: 법선 Z+, 사각 형상, 면적 > 1000 mm².
  - face 위에 그리드 샘플 포인트(n × n, n ≥ 10) 생성.
  - 최소 자승 평면(best-fit plane) `gp_Pln` 피팅.
  - 각 샘플 포인트에서 평면까지의 거리 d_i 계산, max(|d_i|)가 flatness.
  - flatness > 0.02 mm 이면 Error.
- **Failure example**: 포켓 바닥 중앙이 0.05 mm 볼록하게 모델링.
- **Pass example**: 포켓 바닥 평탄도 0.008 mm — 통과.
- **Linked from**: [[engine/feature-phone#Display Pocket]]

---

### DFM-014: Speaker Grille Minimum Pin Thickness

- **Scope**: Both
- **Severity**: Error
- **Criterion**: 스피커 그릴 핀(홀 간 기둥) 두께 ≥ 0.25 mm. 그 미만은 가공 중 핀 파손 위험. 알루미늄 최소 0.25 mm, 스틸 0.2 mm(별도 파라미터).
- **OCCT APIs**: `BRepExtrema_DistShapeShape`, `TopExp_Explorer`, `Bnd_OBB`
- **Algorithm sketch**:
  - 스피커 그릴 영역의 홀 엣지 리스트 수집 (DFM-002와 동일 방법).
  - 인접 홀 엣지 쌍 간 `BRepExtrema_DistShapeShape`로 핀 두께 d 계산.
  - d < 0.25 mm 인 핀을 위반 마킹.
  - 그릴 패턴 전체에 걸쳐 최솟값 보고.
- **Failure example**: 스피커 그릴 홀 직경 0.5 mm, 피치 0.65 mm → 핀 두께 0.15 mm.
- **Pass example**: 홀 직경 0.4 mm, 피치 0.7 mm → 핀 두께 0.3 mm → 통과.
- **Linked from**: [[engine/feature-phone#Speaker Grille]], [[engine/feature-watch#Speaker Grille]]

---

### DFM-015: Tap-Hole Minimum Surrounding Material

- **Scope**: Both
- **Severity**: Error
- **Criterion**: 탭 홀 주변 재료 두께 ≥ tap diameter × 1.5. 탭 홀 너무 가까운 엣지/홀 → 나사 체결 시 갈라짐. 예: M1.0 탭 홀 → 주변 재료 ≥ 1.5 mm.
- **OCCT APIs**: `BRepExtrema_DistShapeShape`, `BRepAdaptor_Surface`, `TopExp_Explorer`
- **Algorithm sketch**:
  - 탭 홀 식별: 나사산 메타데이터(속성 맵) 또는 직경 범위(0.8–3 mm) + 깊이/직경 비 > 0.8.
  - 각 탭 홀 축으로부터 인접 face/edge까지 `BRepExtrema_DistShapeShape` 최소 거리 d_surround.
  - d_surround < tap_diameter × 1.5 이면 Error.
  - 외벽까지의 거리도 포함(홀 벽 = 재료 두께).
- **Failure example**: M1.2 탭 홀 중심에서 외벽까지 1.0 mm → 요구 1.8 mm 미달.
- **Pass example**: M1.0 탭 홀 중심에서 외벽 2.0 mm → 요구 1.5 mm 충족.
- **Linked from**: [[engine/feature-watch#Lug Root]], [[engine/feature-phone#Bracket Mount]]

---

### DFM-016: Anodizing Mask Region Closed

- **Scope**: Both
- **Severity**: Error
- **Criterion**: 아노다이징 마스킹 영역 폴리곤이 완전히 닫혀야 함(no open wire). 열린 폴리곤은 아노다이징 액 침투로 색상 경계 번짐 발생.
- **OCCT APIs**: `BRep_Builder`, `TopExp_Explorer`, `TopoDS_Wire`, `BRepCheck_Analyzer`
- **Algorithm sketch**:
  - 마스킹 영역은 어트리뷰트/레이어 메타데이터로 태그된 `TopoDS_Wire` 또는 `TopoDS_Face`.
  - `BRepCheck_Analyzer`로 각 wire의 `CheckShape` 실행.
  - `BRepCheck_Wire` 결과에서 `BRepCheck_NotClosed` 상태 탐지.
  - 또는: `TopExp_Explorer`로 wire의 vertex gap 검사(첫 vertex ≠ 마지막 vertex).
  - 열린 wire 개수 > 0 이면 Error.
- **Failure example**: 아노다이징 경계 스케치 마지막 세그먼트가 0.1 mm gap으로 미완성.
- **Pass example**: 전면 파트 마스킹 폴리곤 완전 폐쇄 → 통과.
- **Linked from**: [[engine/parametric-templates#Anodizing Setup]]

---

### DFM-017: Outer-Edge Curvature Continuity

- **Scope**: Both
- **Severity**: Warning
- **Criterion**: 외부 실루엣 엣지 연속성 최소 G1(탄젠트 연속); G2(곡률 연속) 권장. G0 연속(위치만 연속, 탄젠트 불연속)은 가공 흔적 및 아노다이징 줄무늬.
- **OCCT APIs**: `BRepAdaptor_Curve`, `GeomLProp_CLProps`, `TopExp_Explorer`, `ShapeAnalysis_Edge`
- **Algorithm sketch**:
  - 외곽 실루엣 엣지(silhouette edge): 법선이 수평(|n·Z| < 0.1)인 face의 경계 엣지.
  - 연결 vertex에서 두 인접 curve의 탄젠트 벡터 t₁, t₂ 계산 (`GeomLProp_CLProps::D1`).
  - cos θ = t₁ · t₂; θ > 0.5° 이면 G0(탄젠트 불연속) → Warning.
  - G2 검사: 같은 점에서 곡률 κ₁, κ₂ 비교, |κ₁ − κ₂| > 0.1 mm⁻¹ 이면 G2 미달 Info.
- **Failure example**: 케이스 측면 두 face가 G0 접합으로 날카로운 능선 생성.
- **Pass example**: NURBS 혼합으로 G2 연속 외곽 프로파일 → Warning/Info 없음.
- **Linked from**: [[engine/morphing-route#Surface Blend]], [[engine/parametric-templates#Corner Blend]]

---

### DFM-018: Camera Deco-Ring Parallelism to Top Face

- **Scope**: Phone
- **Severity**: Warning
- **Criterion**: 카메라 데코링 상면과 케이스 전체 상면(top face) 간 기울기 ≤ 0.1°. 기울기 초과 시 카메라 모듈 조립 간섭 및 렌즈 광축 편차.
- **OCCT APIs**: `BRepAdaptor_Surface`, `gp_Pln`, `TopExp_Explorer`, `gp_Vec`
- **Algorithm sketch**:
  - 케이스 상면 법선 n_top (Z+ 방향 face, 최대 면적).
  - 각 데코링 상면 법선 n_ring 계산.
  - angle = acos(|n_top · n_ring|) (절댓값으로 방향 무관).
  - angle > 0.1° 이면 Warning.
  - 다중 카메라 배치 시 각 링 독립 검사.
- **Failure example**: 카메라 아일랜드가 0.3° 기울어진 상태로 생성.
- **Pass example**: 데코링 상면과 케이스 상면 각도 0.04° → 통과.
- **Linked from**: [[engine/feature-phone#Camera Island]]

#### 구현 노트: 데코링 유효성 (surround check)

`dfm::runProductDFM` 의 DFM-018 은 `spec["camera_deco_rings"]` 를 받아 찢어질 수 있는
**두 벽**을 검사한다 (phone 프로파일 `decoRingRule` 일 때만, watch 는 N/A):

1. **groove 채널 폭** = `(outer_dia − inner_dia)/2` — 컷터가 들어가야 하는 벽.
2. **lens surround** = `(inner_dia − matching_camera_hole_dia)/2` — 카메라 홀과 링
   내벽 사이의 lip, **찢어지기 쉬운 벽**.

링↔카메라 매칭은 `cameraDiaAt(x,y)` 람다가 같은 offset(±0.5 mm)의 카메라를 찾아
`hole_dia_mm` 을 돌려준다.  두 값 중 하나라도 `profile.minWallMm` 미만이면 Warning.

> **수정(silent-corruption)**: 이전 규칙은 groove 폭**만** 측정해, 카메라 홀에
> 위험하게 가까운 링(얇은 lens surround)을 **놓쳤다**.  이제 measured 카메라
> 직경으로 surround 도 검사한다.  원칙은 동일 — 검사/게이트는 측정 가능한 실제
> 결함을 빠짐없이 잡되, 유효한 값을 임의로 덮어쓰지 않는다 (같은 교훈:
> [[engine/reverse-route#Recognizer]], [[engine/skills#param-clamp]],
> [[engine/skills#auto_rib_between_two_walls]]).  소스:
> [[src/engine/dfm/ProductDFM.cpp]].

---

### DFM-019: Engraved Text/Logo Minimum Stroke Width

- **Scope**: Both
- **Severity**: Error
- **Criterion**: 음각 텍스트/로고 획 폭 ≥ 0.15 mm. 표준 V-커터 최소 폭 0.15 mm. 그 미만은 가공 불가 또는 텍스트 판독 불가.
- **OCCT APIs**: `BRepExtrema_DistShapeShape`, `TopExp_Explorer`, `BRepBndLib`
- **Algorithm sketch**:
  - 음각 영역 식별: Z− 방향으로 들어간 얕은 face(깊이 0.01–0.5 mm), 면적 < 100 mm².
  - 각 음각 face의 내접원 직경(inscribed circle diameter) ≈ 2 × min distance to boundary.
  - `BRepExtrema_DistShapeShape`로 face 내부 포인트에서 외곽 wire까지 최소 거리 d_min.
  - stroke width ≈ 2 × d_min; < 0.15 mm 이면 Error.
  - 또는 face 폭을 `Bnd_Box` local X/Y span으로 근사.
- **Failure example**: 로고 세리프 획 폭 0.08 mm 설계.
- **Pass example**: 로고 최소 획 폭 0.20 mm — 통과.
- **Linked from**: [[engine/feature-phone#Logo Engrave]], [[engine/feature-watch#Case Back Engrave]]

---

### DFM-020: All Shells Closed (No Open Shells)

- **Scope**: Both
- **Severity**: Error
- **Criterion**: 모든 `TopoDS_Shell`이 닫혀 있어야 함(`IsClosed() == true`). 열린 쉘은 솔리드 연산 실패 및 STEP 내보내기 오류.
- **OCCT APIs**: `TopoDS_Shell`, `BRepCheck_Analyzer`, `TopExp_Explorer`
- **Algorithm sketch**:
  - `TopExp_Explorer`로 모든 `TopoDS_Shell` 열거.
  - 각 shell에 `BRepCheck_Analyzer` 적용.
  - `BRepCheck_Shell` 결과에서 `BRepCheck_NotClosed` 감지.
  - 또는 shell의 모든 edge가 정확히 두 face에 공유되는지 확인(자유 엣지 개수 = 0).
  - 자유 엣지(free edge) 개수 > 0 이면 Error.
- **Failure example**: 파라메트릭 블렌드 실패로 face 사이 0.001 mm gap 발생.
- **Pass example**: `TopoDS_Solid` 정상 생성, 모든 shell closed — 통과.
- **Linked from**: [[engine/parametric-templates#Solid Validation]], [[engine/morphing-route#Shell Repair]]

---

### DFM-021: Zero Self-Intersection

- **Scope**: Both
- **Severity**: Error
- **Criterion**: 솔리드 자기 교차 없음. `BRepAlgoAPI_Cut(shape, shape)` 결과가 빈 shape. 자기 교차 솔리드는 CAM 경로 생성 불가 및 물리적으로 불가능한 형상.
- **OCCT APIs**: `BRepAlgoAPI_Cut`, `BRepCheck_Analyzer`, `BOPAlgo_Checker`
- **Algorithm sketch**:
  - `BOPAlgo_Checker`로 shape의 자기 교차 검사 (권장 방법).
  - `BOPAlgo_Checker::Perform()` 후 `HasErrors()` 확인.
  - 대안: `BRepAlgoAPI_Cut(S, S)` 실행 후 결과 shape의 volume > epsilon 이면 자기 교차.
  - `BRepCheck_Analyzer::IsValid()` 로 추가 확인.
  - 오류 발생 시 `BOPAlgo_Checker::DumpErrors()` 로 위치 정보 추출.
- **Failure example**: 모프 경로에서 두 face가 내부적으로 관통하여 자기 교차 생성.
- **Pass example**: 정상 솔리드 → `BOPAlgo_Checker::HasErrors() == false` → 통과.
- **Linked from**: [[engine/morphing-route#Topology Repair]], [[engine/reverse-route#Mesh-to-Solid]]

---

### DFM-022: OBB Z Aligned with Thickness Axis

- **Scope**: Both
- **Severity**: Warning
- **Criterion**: OBB(Oriented Bounding Box)의 최소 반경 방향(두께 축)이 글로벌 Z와 정렬됨. thickness < width AND thickness < height. 잘못 정렬된 케이스는 가공 셋업 오류.
- **OCCT APIs**: `Bnd_OBB`, `BRepBndLib`, `gp_Dir`
- **Algorithm sketch**:
  - `BRepBndLib::AddOBB(shape, obb)` 로 OBB 계산.
  - OBB의 세 반축(half-extents) XSize, YSize, ZSize 및 방향 XDir, YDir, ZDir 추출.
  - ZSize = min(XSize, YSize, ZSize) 인지 확인 (두께 = 최소 치수).
  - OBB의 두께 방향 축과 gp::DZ() 간 각도 θ = acos(|thickness_axis · Z|).
  - θ > 5° 이면 Warning.
- **Failure example**: 케이스가 X축 기준 90° 회전된 상태로 저장, 두께 방향이 X.
- **Pass example**: 케이스 두께 방향 = Z, OBB Z축 = gp::DZ() → 통과.
- **Linked from**: [[engine/parametric-templates#Blank Orientation]], [[engine/cam-3axis-verify#Setup Orientation]]

---

### DFM-023: Watch Lug Span Symmetry

- **Scope**: Watch
- **Severity**: Warning
- **Criterion**: 시계 러그(lug) 좌우 스팬 비대칭 ≤ 0.05 mm. 스트랩 장착 시 좌우 러그 간격이 다르면 스트랩 비틀림 및 핀 파손. 표준 스트랩 핀(스프링 바) 규격은 ±0.05 mm 허용.
- **OCCT APIs**: `BRepBndLib`, `Bnd_Box`, `TopExp_Explorer`, `BRepAdaptor_Surface`
- **Algorithm sketch**:
  - 러그 face 식별: 케이스 상단에서 Y방향으로 돌출된 face, 법선 ±Y 성분 우세.
  - 좌측 러그 쌍과 우측 러그 쌍의 `Bnd_Box` Y 범위 추출.
  - span_left = Y_max_left − Y_min_left, span_right 동일.
  - |span_left − span_right| > 0.05 mm 이면 Warning.
  - 12시/6시 방향 러그도 X축 기준 동일 검사.
- **Failure example**: 러그 미러링 오류로 좌측 스팬 18.05 mm, 우측 17.95 mm → 0.10 mm 차이.
- **Pass example**: 좌/우 러그 스팬 모두 18.00 mm → 통과.
- **Linked from**: [[engine/feature-watch#Lug Root]]

---

### DFM-024: Speaker Mesh Hole Roundness

- **Scope**: Both
- **Severity**: Warning
- **Criterion**: 스피커 메시 홀의 진원도(circularity) ≥ 0.85. 진원도 = 4π × Area / Perimeter². 타원형/변형 홀은 막힘 및 음향 성능 저하. CNC 가공 정밀도 지표로도 활용.
- **OCCT APIs**: `BRepGProp`, `GProp_GProps`, `BRepAdaptor_Curve`, `TopExp_Explorer`
- **Algorithm sketch**:
  - 스피커 메시 영역 홀 face(관통홀, 직경 0.3–2 mm) 수집.
  - 각 홀 face의 경계 wire에서 `BRepGProp::LinearProperties`로 둘레 P 계산.
  - 동일 face의 `BRepGProp::SurfaceProperties`로 면적 A 계산.
  - circularity = 4π × A / P².
  - circularity < 0.85 이면 Warning.
- **Failure example**: 가공 오류 시뮬레이션으로 홀이 타원(0.5 × 0.8 mm) → circularity 0.78.
- **Pass example**: 원형 홀 φ0.6 mm → circularity ≈ 1.0 → 통과.
- **Linked from**: [[engine/feature-phone#Speaker Grille]], [[engine/feature-watch#Speaker Grille]]

---

### DFM-025: Anodizing Color-Uniformity Edge Buffer

- **Scope**: Both
- **Severity**: Info
- **Criterion**: 외곽 엣지에서 아노다이징 색상 경계(color-change boundary)까지 거리 ≥ 0.3 mm. 엣지 근처에서 아노다이징 전류가 집중되어 색상이 달라짐(edge burn). 0.3 mm 이상 이격 시 균일한 색상 보장.
- **OCCT APIs**: `BRepExtrema_DistShapeShape`, `TopExp_Explorer`, `TopoDS_Wire`
- **Algorithm sketch**:
  - 아노다이징 색상 경계 wire를 메타데이터에서 로드(레이어 태그 또는 속성 맵).
  - 외곽 실루엣 엣지 리스트 수집 (DFM-017과 동일 방법).
  - 각 색상 경계 wire 포인트에서 가장 가까운 외곽 엣지까지 `BRepExtrema_DistShapeShape`.
  - 최소 거리 < 0.3 mm 이면 Info 로그.
  - 색상 경계 메타데이터 없으면 검사 skip.
- **Failure example**: 케이스 상단의 그라데이션 경계가 외곽 엣지에서 0.15 mm 이격.
- **Pass example**: 그라데이션 경계가 외곽에서 0.5 mm 안쪽에 배치 → Info 없음.
- **Linked from**: [[engine/parametric-templates#Anodizing Setup]], [[engine/feature-phone#Color Gradient]]

---

## 부록: OCCT API 참조 (Appendix: OCCT API Reference)

아래는 이 카탈로그에서 반복적으로 사용되는 OCCT 8.0.0 클래스 목록이다.

| 클래스 | 용도 |
|---|---|
| `BRepExtrema_DistShapeShape` | 두 shape 간 최소 거리 계산 |
| `BRepCheck_Analyzer` | 토폴로지 유효성 검사 |
| `BRepAdaptor_Surface` | face를 표면 어댑터로 래핑 |
| `BRepAdaptor_Curve` | edge를 곡선 어댑터로 래핑 |
| `BRepAlgoAPI_Cut` | 솔리드 차집합 연산 |
| `BOPAlgo_Checker` | 자기 교차 및 BOP 오류 검사 |
| `GProp_GProps` + `BRepGProp` | 관성/면적/체적 계산 |
| `TopExp_Explorer` | 토폴로지 요소 순회 |
| `Bnd_OBB` + `BRepBndLib` | Oriented Bounding Box 계산 |
| `GeomLProp_CLProps` | 곡선 미분 속성(탄젠트, 곡률) |
| `gp_Pln`, `gp_Dir`, `gp_Ax1` | 기하 프리미티브 |
| `ShapeAnalysis_Edge` | 엣지 연속성 분석 |

---

## 변경 이력 (Changelog)

| 버전 | 날짜 | 변경 내용 |
|---|---|---|
| v1.0 | 2026-05-25 | 초기 25개 규칙 정의. Watch + Smartphone 케이스 금속 CNC 가공 기준. |
| v1.0.1 | 2026-06-18 | DFM-018 데코링 검사에 lens surround `(inner−matching_camera_dia)/2` 추가 (이전엔 groove 폭만 측정). 2차 adversarial 리뷰 silent-corruption 수정. |

---

*이 파일은 [[engine/parametric-templates]], [[engine/feature-watch]], [[engine/feature-phone]], [[engine/morphing-route]], [[engine/reverse-route]], [[engine/cam-3axis-verify]]가 공통으로 참조하는 DFM 계약 문서이다. 규칙 추가/변경은 이 파일만 수정하고 버전을 증가시킨다.*
