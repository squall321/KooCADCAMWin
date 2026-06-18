# Modeling Advancement Roadmap

8-차원 적대적 멀티에이전트 탐색(2026-06-18, survey→코드선별→verify→synthesize, 17 에이전트) +
**메인 컨텍스트 직접 grep 재검증**으로 확정한 KooCADCAM 기하/CAD 모델링 고도화 로드맵.

## 성숙도 (8 차원)
| 차원 | 성숙도 |
|---|---|
| robustness (Boolean 후 검증/healing) | **crude** |
| parametric (구동 치수/구속) | **crude** |
| assembly (멀티바디/메이트) | **crude** |
| freeform (NURBS/loft/sweep) | partial |
| product-fidelity (제품 피처 충실도) | partial |
| tolerance-gdt | partial |
| tessellation (메시/측정) | partial |
| sketch-2d (프로파일 기반) | partial |

## 핵심 진단
모델링이 두 개의 분리된 서브시스템 — **엔진 경로**(WatchFrontModel/PhoneFrontModel)는
모든 step이 `prim::runStep`의 BRepCheck 게이트를 통과하고 FeatureEditor가
heal→volume-sanity→BRepCheck→reject를 구현하는 등 견고함. 반면 **250+ 스킬 합성 경로**(모델링의
대부분)는 raw Boolean 출력에서 곧장 Workpiece를 만들고 검증/healing이 전무. 가장 큰 레버리지는
이 합성 경로를 엔진 경로만큼 견고하게 만드는 것.

## 검증된 작업 (7갭 확정 / 1갭 기각, 우선순위순)

### #1 — 스킬 합성 출력 경로에 heal+validate 게이트 (START FIRST) — HIGH / M
**메인 컨텍스트 grep 재검증 완료**: `finalizeOutput` 채택자 **0**, 인라인 `make_shared<Workpiece>` **757개소**,
Workpiece ctor([Workpiece.cpp:26-30](../src/skills/Workpiece.cpp#L26))는 `enumerate()`만 — BRepCheck/null/empty/ShapeFix 전무.
무효·빈 BRep이 무음으로 "유효한" Workpiece가 되어 피처체인+STEP export로 전파.
- `Workpiece::makeValidated(shape, material)` 팩토리: BRepCheck → 무효면 ShapeFix 1회 → 재검사 → 여전히 무효/null/empty면 `SkillError` throw, 아니면 검증된 `shared_ptr` (FeatureEditor.cpp:157-166 패턴 미러, TKShHealing 이미 링크 CMakeLists:946)
- `finalizeOutput`([Workpiece.hpp:108-116](../src/skills/Workpiece.hpp#L108))을 makeValidated 경유로 변경 → sanctioned 경로가 construction-by-validation
- 757개 인라인 사이트를 finalizeOutput로 마이그레이션(기계적, check_feature_chain.py의 delegating-compound 예외 수기 처리). **757개라 batch 워크플로우로 per-batch 빌드 검증하며 진행 권장**
- heal-성공 분기에 spdlog::warn (telemetry)
- 유닛테스트: 무효/빈 BRep → SkillError 또는 healed-valid (절대 무음 "valid" 아님) + 정상 출력 round-trip 불변
- **코어 lib 변경 → full rebuild 필수**(incremental ctest는 false-green, project memory)

### #2 — CAM 정확 BRep 충돌 검증 (BRepExtrema_DistShapeShape + SolidClassifier) — HIGH / M
CollisionCheck가 workpiece를 단일 AABB로 축약([CollisionCheck.cpp:98-99](../src/cam/CollisionCheck.cpp#L98)) → 모든 오목 피처(포켓/베젤 캐비티/카운터보어)가 기하학적으로 소거됨(포켓 진입=false negative, 개방 포켓 위 클리어런스=false positive). AABB broad-phase 유지 + 정확 테스트 추가.

### #3 — 구동 치수 pre-pass (JSON spec 위 선택적 per-field expr) — MED / M
치수 간 관계가 독립 JSON 값 + 하드코딩 커플링 (구동/구속 없음). 빌더 실행 전 spec 위 expr 해석 pre-pass.

### #4 — XCAF 어셈블리 import (named instance + placement) — MED / L
멀티솔리드를 named instance + per-instance placement로 (현재 단일 익명 compound 반환). 폰=다부품.

### #5 — 워치 돌출 크라운(널링)+후면 센서 돔 (첫 제품 충실도 패스) — MED / M
크라운이 순수 차감([WatchFrontModel.cpp:194-196](../src/engine/WatchFrontModel.cpp#L194)) — 돌출 노브/널링 없음. coneFrustum/fuseMany 기존 활용. fuse-before-cut, 순차 cut(OCCT compound-cut rule).

### #6 — freeform 스킬 1개를 제품에 watertight solid로 연결: 돔 사파이어 글래스(solid loft) — MED / L
9개 freeform 스킬(real OCCT apply: ThruSections/PointsToBSpline/MakePipe/MakeOffsetShape)이 두 제품 어디에도 미사용(grep ZERO). 평면 글래스→돔. `solid=true` ThruSections(loft_surface는 face를 compound에 fuse→non-manifold 위험, 이 슬라이스는 watertight solid 필수). `glass_profile: flat|domed` opt-in.

## 별개 프론티어 (이 로드맵 밖)
- watch→phone 제품 간 전이 ([[project_multiproduct_transfer_gap]]) — product-identity 다리. 모델링 깊이가 아니라 제품 간 일반화.
