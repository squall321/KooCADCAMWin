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

### #1 — 스킬 합성 출력 경로에 heal+validate 게이트 ✅ **완료 (d7624f0 + b374e24)** — HIGH / M
**Phase A (d7624f0)**: `validateOrHeal`(BRepCheck→ShapeFix heal→throw) + `finalizeOutput` 경유 + ctor null 가드 + drill_hole 증명 + workpiece_validate_test(5케이스).
**Phase B (b374e24)**: 756사이트 대량 diff 대신 **Workpiece ctor가 validateOrHeal 호출** → 전 757사이트+RE+stock이 construction-by-validation. non-fatal probe로 측정한 blast radius = 정확히 2개 스킬(778 중)이 ShapeFix-unhealable invalid 출력. 적대적 진단 워크플로우(standalone probe가 1차 가설을 반증)로 근본 수정: **internal_gear**(bore+gullet 침투 단일 cutMany→순차 cut), **top_face_recess_with_walls**(over-broad edgesAtZ fillet 자기교차→외곽 4모서리 술어+validity-guard). authoritative 812 ctest 100% pass, 무회귀. 이제 어떤 스킬도 heal/throw 경로 미진입 — 게이트는 순수 회귀 backstop.

<details><summary>원래 진단(보존)</summary>

**메인 컨텍스트 grep 재검증 완료**: `finalizeOutput` 채택자 **0**, 인라인 `make_shared<Workpiece>` **757개소**,
Workpiece ctor([Workpiece.cpp:26-30](../src/skills/Workpiece.cpp#L26))는 `enumerate()`만 — BRepCheck/null/empty/ShapeFix 전무.
무효·빈 BRep이 무음으로 "유효한" Workpiece가 되어 피처체인+STEP export로 전파.
- `Workpiece::makeValidated(shape, material)` 팩토리: BRepCheck → 무효면 ShapeFix 1회 → 재검사 → 여전히 무효/null/empty면 `SkillError` throw, 아니면 검증된 `shared_ptr` (FeatureEditor.cpp:157-166 패턴 미러, TKShHealing 이미 링크 CMakeLists:946)
- `finalizeOutput`([Workpiece.hpp:108-116](../src/skills/Workpiece.hpp#L108))을 makeValidated 경유로 변경 → sanctioned 경로가 construction-by-validation
- 757개 인라인 사이트를 finalizeOutput로 마이그레이션(기계적, check_feature_chain.py의 delegating-compound 예외 수기 처리). **757개라 batch 워크플로우로 per-batch 빌드 검증하며 진행 권장**
- heal-성공 분기에 spdlog::warn (telemetry)
- 유닛테스트: 무효/빈 BRep → SkillError 또는 healed-valid (절대 무음 "valid" 아님) + 정상 출력 round-trip 불변
- **코어 lib 변경 → full rebuild 필수**(incremental ctest는 false-green, project memory)

</details>

### #2 — CAM 정확 BRep 충돌 검증 (BRepExtrema_DistShapeShape + SolidClassifier) ✅ **완료 (11b6276)** — HIGH / M
broad(단일 AABB reject 유지) + narrow(tip-IN SolidClassifier / 툴기둥 DistShapeShape) 2단계. 개방포켓 위 클리어런스 false-positive + 벽 진입 false-negative 제거. `exact_narrow_phase=true` 기본, 포켓 회귀 테스트 2건. **CAVEAT: koo_cam은 production 호출처 0(dead code) — checkPath를 Executor/GUI에 연결하기 전엔 dark ship(추적 follow-up).**

### #3 — 구동 치수 pre-pass (JSON spec 위 선택적 per-field expr) ✅ **완료 (68a2813, 3a)** — MED / M
`io/SpecExpr.hpp`: `=expr` 문자열 leaf를 숫자로 해석(dotted-path 참조, `+ - * /`, 괄호, precedence, 단항, chained-ref fixpoint). plain string/숫자 불변. Watch/Phone buildAll의 transparent pre-pass(header-only). watch→phone 멀티제품 파라메트릭 직결. (3b XCAF는 #4-doc 참조 — 소비자 없어 보류)

### #4-doc — XCAF 어셈블리 import (named instance + placement) ⏸ **보류 (소비자 없음)** — MED / L
스코핑 결과 named instance에 현재 소비자 없음(PartsLayout가 flatten). 폰 다부품 어셈블리가 실제 필요해질 때 진행.

### #5 — 워치 돌출 크라운(널링)+후면 센서 돔 (첫 제품 충실도 패스) ✅ **완료 (a993001)** — MED / M
크라운 순수 차감→돌출 knob(coneFrustum, fuse-before-cut)+널링(순차 cut)+샤프트. 후면 센서 평면홀→융기 돔. 전부 opt-in. 재료 추가+돌출 검증 테스트.

### #6 — freeform 스킬 1개를 제품에 watertight solid로 연결: 돔 사파이어 글래스(solid loft) ✅ **완료 (a993001)** — MED / L
`pr::domeSolid`(ThruSections solid=true watertight) 신규 엔진 primitive. `display_pocket.glass_profile="domed"`로 돔 글래스 fuse. **freeform 엔진을 처음으로 제품에 연결**(곡면 BSpline face 검증). opt-in, 기본 flat 불변.

### 입력공간 감사 (사용자 옵션 #4) ✅ **완료 (영구 가드)**
게이트(validateOrHeal)가 어떤 입력이든 무효 출력을 런타임 영구 차단. 측정: 전 778 스킬@test입력=2 invalid(수정됨), precise-tier@극단스케일(0.1×~20×)=0 invalid. `corpus_scale_small/large` ctest로 극값 geometric 유효성 영구 회귀 가드 추가. (전 778 bespoke-input fuzz는 per-skill 생성기 필요 — 별도 follow-up)

## 별개 프론티어 / Follow-ups (이 로드맵 밖)
- **CAM wiring**: koo_cam(#2)을 Executor/GUI 파이프라인에 연결 — 그래야 충돌 검증이 실제로 실림
- **전 778 입력-fuzz**: per-skill 입력 생성기 → 전수 입력공간 감사
- **XCAF 어셈블리**(#4-doc): 폰 다부품 소비자 생길 때
- watch→phone 제품 간 전이 ([[project_multiproduct_transfer_gap]]) — product-identity 다리. 모델링 깊이가 아니라 제품 간 일반화.
