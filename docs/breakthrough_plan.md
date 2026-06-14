# KooCADCAM 자동화 한계 돌파 계획 (Breakthrough Plan)

> 작성: 2026-06-10. 방법론: 9개 서브시스템 병렬 감사(읽기 전용) → 제안별 적대적 검증(증거 실존·OCCT 헤더 실존·제약 충돌·effort 현실성) → 완전성 비평 2렌즈. 총 62 에이전트.
> 원칙: **검증 통과한 설계만 채택**, refuted 항목은 사유와 함께 "하지 말 것"에 보존 (재발 방지).
> @lat: [[process/breakthrough-plan]]

---

## 0. 진단 요약 — 무엇이 자동화를 막고 있나

| # | 한계 (검증된 증거) | 막는 것 |
|---|---|---|
| 1 | **0.5 신뢰도 캡** — 비-precise 인식기는 외부 CAD에서 절대 0.7 추론 임계 통과 불가 (Recognizer.cpp:474-480) | 외부 STEP 이해가 15개 atomic 스킬로 영구 제한 |
| 2 | **스킬 계약 드리프트** — 777 스킬 중 ~332개가 feature-chain 미전파 (실측: 269 loop + 170 setFeatures + 6 cloneWithSignature = 445 전파) | 직접 apply 체인의 메타데이터 복원·재실행 신뢰성 |
| 3 | **복원 플랜 재실행 실패** — 중복 drill 원자 누적으로 빈 shape (bearing_housing DISABLED 테스트, vC=0) | "외부 부품 → 인식 → 재실행" 비전 |
| 4 | **material-add 편집 취약** — annulus fuse + far_center 휴리스틱 (1046mm 실패 사례), move 기능 부재 | 치수/위치 편집의 제품화 |
| 5 | **DFM 측정 한계 + 중복 임박** — midpoint-normal은 평면 전용, PhoneFrontModel에 runDFM 없음 ("its own copy" TODO) | 멀티 제품 DFM 일반화 |
| 6 | **GUI 손작업 위젯** — 워치 step 1-6만 편집 가능(7-10 보존만), 폰 위젯 0개, currentSpec/setSpec 제품별 중복 | 멀티 제품 GUI 스케일 |
| 7 | **라벨 데이터 천장** — 인식 품질이 binary pass/fail 단위테스트뿐, recall/precision 측정 불가, AS1급 스케일 갭 감지 불가 | 인식 품질의 체계적 개선 |
| 8 | **검증 인프라 구멍** — verify 게이트 스크립트 부재, ctest LABELS 0개, false-green 재발 가능 | "all green" 주장의 신뢰성 |
| 9 | **reframe 치수 미스케일** — applyPartDeltaToParams가 위치만 변환, diameter/depth 미스케일; AABB 매칭은 대칭/중첩 부품에서 모호 | 부품 리사이즈 시 가공 자동 재생성의 정확성 |

감사가 **정정한 사실** (이후 모든 계획의 기준):
- 스킬 수 777 (765 아님), 전파 445/777 (266/765 아님) → 미전파 **332**
- `BRepAlgoAPI_Defeaturing` OCCT 8.0 헤더 **실존 확인** (BOPAlgo_RemoveFeatures 래핑, 자동 힐링) — 편집 돌파의 핵심 enabler
- CI(.github/workflows/ci.yml)는 **이미 존재** (OCCT 캐시 포함) — "CI 만들기"가 아니라 "CI 신뢰성 보강"이 과제
- `engine::DFMReport` 에는 add()가 없음 (skill::DFMReport엔 있음) — 레지스트리 전 통일 필요
- geomFingerprint는 position_z 포함 0.05mm 그리드 — c35f72c가 DISABLED 테스트 비활성화 **이후** 랜딩 → 재활성화 실측이 첫걸음

---

## 1. 돌파 트랙 (검증 통과 설계만)

### B1. 인식 일반화 — 캡을 증거 합성으로 대체
**목표**: 외부 CAD에서 compound 인식이 가능하면서 false-positive 홍수는 차단.

1. **B1.1 곱셈형 증거 부스트** (G-RECOG-2 수정판, S)
   `conf_adjusted = base × boost_geom(1.2 if fingerprint 안정) × boost_metadata(1.3 if replay)`, 1.0 캡.
   - false-positive(0.5×1.0)=0.5 → 임계 미달 유지 / 기하 일치 compound 0.6 → 접근 / 메타+기하 0.78 → 통과.
   - analyze()의 캡 적용 직후·정렬 직전에 1개 함수 삽입. (원안의 가중 합산식은 계수합 1.15로 수학 오류 — 검증자가 잡음)
2. **B1.2 의존성 인식 오버로드** (G-RECOG-1 수정판 Phase 1a, M)
   `recognize(wp, prior_atoms)` 오버로드 신설, compound 5종(gauge_block_step, jig_plate_with_drill_bushings, auto_gusset_corner_brace, hollow_cavity, auto_boss_under_hole)부터 opt-in. atomic 증거 없으면 compound 후보 미발행.
3. **B1.3 선언적 컴파운드 문법(DSL)** (G-RECOG-1 Phase 1b-2, L)
   compound = { atomic_prerequisites, geometric_constraints, confidence_formula(min) } YAML/JSON 룰 + 경량 인터프리터. 5종 검증 후 상위 20-30 compound 이식. **777개 손코딩 휴리스틱의 유지보수 천장을 데이터로 전환.**
4. **B1.4 compound dedupe 보강** (G-RECOG-4 수정판, S, ~15줄)
   dedupe()에 `is_compound` 후보용 (skill_id + subfeature_count) 2차 키 추가.
5. **B1.5 ✅ 완료 (2026-06-11)** — strict-superset 교체 규칙 구현, post-dedupe recall 6/15→12/15=1.000, corpus `--gate 0.99` 활성화. 추가 수확: 특이성 규칙이 노출한 팬텀 4종을 물리 게이트로 봉쇄(rect_pocket 오목성+각도스팬, slot 오목성, bws void-axis+순차 cut[compound-wipe 실발화 수정], counterbore entry-air+숄더 claim) — bearing 재실행이 6-스킬 리치 플랜으로 PASS. 잔여: bolt_hole entry claim(게이트 면제 중), countersink 역방향 케이스. (원안: M) — 코퍼스 첫 실측(2026-06-11)이 정량화: recall_pre=1.000 인 9개 precise 스킬(counterbore/countersink/bore_*/mill_*pocket/mill_slot/bolt_hole/drill_through_hole)이 **dedupe 이후 0** — drill_hole(0.95)이 공유 실린더 면을 선점하면 dedupe 가 raw confidence 만으로 더 구체적인 후보를 버린다 (AS1 counterbore 2→0 과 동일 메커니즘). 수정 방향: face-ID 겹침 시 **더 많은 기하를 설명하는 후보 우선**(matched face-set 크기 → 동률이면 confidence). 수정 후 corpus_basic 의 `--gate`(post-dedupe)를 켠다.

수용 기준: 자기-라벨 코퍼스(B7)에서 compound recall 측정 가능 + bare-stock false-positive 0 유지 + 798 테스트 green.

**B1.x 측정 결과 (2026-06-14, fpscan_test):** re::analyze(applyCap=false) raw 모드 + 6-파트 foreign 패널. 발견: raw≥0.7 spurious 발화 12개 domain-compound 중 **8개는 이미 dedupe(B1.5 특이성)가 subsume → un-cap 안전**(iso_h7/press_fit/tube_swage/dowel_pin 등 promiscuous bore-spec 포함), **단 4개만 dedupe 생존 → B1.2 타깃 그라운딩 필요**(gauge_block_step 3/6, gas_strut_hinge/hollow_cavity/pin_and_diamond 각 1/6). 캡 load-bearing 단언 + 측정 표가 fpscan_test 에 상시 가동. **B1.2 범위가 ~750 → 4개로 좁혀짐.** **B1.2 #1 완료(2026-06-14):** gauge_block_step 외부-실루엣 게이트(+Z step 면이 파트 XY 경계에 닿을 때만 카운트 — 내부 hole/pocket floor 거부) → fpscan 생존자 4→3. **B1.2 #2-3 완료:** gas_strut_hinge·pin_and_diamond 에 U-span 게이트(quarter 코너 필렛 제외) → 생존자 3→1. **남은 1개 = hollow_cavity(centered rect pocket 에 발화 — cavity↔pocket 의미 overlap, 명백한 FP 아니므로 보류; dedupe confidence 중재).** 측정된 un-cap-blocker 4→1로 정리. 다음: 캡 완화 전략(dedupe-subsume 다수 un-cap 안전성을 더 큰 패널로 확증) 또는 B1.3 DSL.

**정공법 진행 (2026-06-14c):** fpscan 패널 6→13 확장 → 생존자 1→**5**(hollow_cavity 4 등; 작은 패널이 과소평가했음). **grounding-by-subsumption un-cap 가설을 측정으로 반증**: compound 가 atom 을 strict-subsume 해도 "감쌈≠그 피처"(countersink→tapped_hole 0.70/countersunk_bolt_seat 0.65, pocket→cam_actuated_slider) + 0.7 grounded 는 dedupe 특이성으로 올바른 atom 대체 → revert(in-code NOTE 보존). **결론: 안전한 un-cap 은 generic superset 이 아니라 B1.2 SPECIFIC atomic-prerequisite 매칭 필요.** flat 캡은 load-bearing 으로 유지. 진짜 다음 = B1.2 per-compound 의존성 명세(각 compound 의 정확한 sub-feature 패턴) — 큰 작업, 라벨/명세 데이터 의존.

### B2. 스킬 계약 기계화 — 드리프트의 구조적 차단
1. **B2.1 `finalizeOutput()` 헬퍼** (src/skills/OutputUnifier.hpp, S) — 전파+서명+SkillOutput 생성을 한 함수로. (대안으로 검증자가 제시한 "Executor-side 일원화"는 직접 apply 체인을 깨뜨리므로 채택 안 함 — recognizer_test가 바로 그 경로)
2. **B2.2 기계화 codemod** (M, 실측 10-14h) — 332 미전파 + 269 loop + 170 setFeatures를 finalizeOutput으로 통일. 100개 배치마다 full ctest. /WX 클린 빌드 필수.
3. **B2.3 계약 회귀 테스트** (G-UNIT-TEST-CONTRACT-1, **viable**) — Phase 1: 10개 dispatched 스킬 파라미터라이즈드 테스트(2-3h), Phase 2: 50개로 확장.
4. **B2.4 계약 문서화** (G-DOCUMENTATION-ENFORCE-1, **viable**, S) — Skill.hpp REQUIRES 주석 + validateFeatureChain() 테스트 헬퍼.
   ※ clang-query AST 가드는 **refuted** (현 코드 패턴과 불일치) — grep 기반 CI 체크는 codemod 완료 후에만 의미 있음.

수용 기준: `grep -L finalizeOutput src/skills/*.cpp` = 0 (예외 화이트리스트 제외) + 계약 테스트 green.

### B3. 복원 플랜 재실행 견고화
1. **B3.1 ✅ 완료 (2026-06-10 실측)** — 재활성 측정 결과 빈-shape(vC=0)는 이미 해소돼 있었다(fingerprint dedupe c35f72c). 잔여 결함은 테스트 자체가 0.5로 추론해 캡된 material-adding false-positive(gusset/rib/post)가 플랜에 유입, bbox Z 50→71.9mm. 표준 임계 0.7로 수정 후 **PASS, 볼륨 폐합 |dV|/V_A=0.078** (30% 단언으로 강화, DISABLED 제거). slice-11 TODO 종결.
2. **B3.2 ⏸ 보류 (증거 대기)** — B3.1 실측이 현 dedupe로 충분함을 보임. 검증 가능한 이득 없는 변경 금지 원칙에 따라 코퍼스(B7)에서 실패 사례가 나올 때만 적용.
3. **B3.3 공간 정준화(canonicalization)** (G-SPATIAL-1 수정판, M) — dedupe 후 skill_id 그룹 내 (x,y,dia,depth) 클러스터링, z 제외(관통홀 양끝 수렴). 최고 신뢰도 대표 1개만 플랜에.
4. **B3.4 복원 파라미터 검증·클램프 레이어** (G-RECOVERED-PARAMS-1, **viable**, M) — Executor::execute 디스패치 직전 ValidatorFn 레지스트리: stock bbox 대비 클램프, 실패 시 사유 기록+skip. hollow_cavity wall_thickness 크래시類 차단.
5. **B3.5 liftRecoveredStep 프로덕션 승격** (수정판, S) — chamfer/fillet의 nested edge_selector 리프팅만 re:: 로 이동 (drill의 position_z는 이미 무해 — 검증자 확인).
   ※ per-step 볼륨 트랜잭션 게이트는 **refuted** (근본 원인은 실행 전 중복이지 실행 중이 아님).

수용 기준: bearing_housing 재실행 vC가 원본 대비 ±25% 이내, DISABLED 제거.

### B4. Defeaturing 편집 레이어 — material-add 취약성의 구조적 우회
**핵심 enabler 검증됨**: `BRepAlgoAPI_Defeaturing`(헤더 실존, 자동 토폴로지 힐링, 멀티솔리드 보존).
1. **B4.1 ✅ 완료+초과 달성 (2026-06-12)** — koo_edit(FeatureEditor): defeature(기하 기반 면 수집 — face-ID 비의존이라 검증자의 stale-ID 함정 자체를 우회) + recut 으로 **enlarge/shrink/MOVE 가 단일 재료-제거 연산**. 직접 기하 recut 채택으로 skill::apply 재적용이 불필요해져 B4.2 의 entry_face 폴백 전략 없이 단일-실린더 패밀리 3종 편집이 모두 가동. koo_modify --move 추가, fuse 경로 삭제. 4 케이스 테스트 잠금(shrink 볼륨 복원 실측, move 후 구멍 치유 확인 포함).
2. **B4.2 ✅ 완료 (2026-06-14)** — defeatureCoaxialRegion(동축 영역 전체 면 제거+힐) + cutCounterbore(seat→pilot 순차 cut) → editCounterbore 로 counterbore 리사이즈/이동. 직접 기하 recut 채택으로 skill 재적용·entry_face 폴백 불필요. modify_in_place +1(seat Ø12→16, 4 dim 보존). bore_with_shelf 등 추가 stepped-bore 류는 동일 패턴 확장 가능.
3. **B4.3 GUI/CLI 공용화** — FeatureSelector(재사용: koo_modify 근접 탐색) + SkillReapplier. 모든 인식 가능 스킬이 결국 편집 가능해지는 일반 구조.
   ※ 단순 "shrink를 defeature로 교체"(G-SHRINK-ASSEMBLY-FIX)는 entry_face 미복원 문제로 **refuted** — B4.2 폴백 전략이 선행 조건.

수용 기준: modify_in_place_test에 shrink/move 케이스 추가 green + 멀티솔리드 STEP에서 enlarge/shrink 무손상.

### B5. DFM 일반화 — 제품 불가지론 측정·룰 엔진
1. **B5.1 [선행] DFMReport 통일** (S) — engine::DFMReport에 add() 부여 (skill::DFMReport와 정합).
2. **B5.2 GeometryProbe 모듈** (G-PROBE-1 수정판, M) — Phase 1: 현 평면 dihedral을 probe API로 추출(동작 불변), Phase 2: BRepLProp_SLProps 엣지-UV 노멀로 곡면 knife-edge 검출(검증된 OCCT API).
3. **B5.3 홀-외벽 클리어런스 프로브** (G-PROBE-2 수정판, **S로 강등**) — 레이캐스팅 불필요(검증자 정정): BRepExtrema로 홀 실린더↔외곽면 최소거리.
4. **B5.4 🟨 1단계 완료 (2026-06-11)** — 룰 엔진을 product-agnostic `dfm::runProductDFM(shape, spec, DFMProfile)` 로 추출(src/engine/dfm), 제품 차이는 프로파일 **데이터**(minWall 0.35/0.40, phone-scoped 룰 스위치). 풀 JSON 외부화는 룰셋 안정화 후 2단계.
5. **B5.5 ✅ 완료 (2026-06-11)** — PhoneFrontModel::runDFM = 1줄 래퍼, 기본 폰이 첫 실측에서 자기 게이트 PASS + 테스트 잠금. "its own copy" TODO 소멸.
6. **B5.6 곡면 포켓 곡률 룰 DFM-019** (G-PROBE-6 수정판, S, opt-in spec 플래그).
   ※ 777 스킬 validate() 임계값 전면 레지스트리화(G-PROBE-5)는 risk 대비 가치 낮음 — DFM-002류 공유 상수만 1차 이관.

수용 기준: watch 결과 798 테스트 불변 + phone runDFM 가동 + 룰 추가가 JSON 1레코드.

### B6. 스키마 주도 제품/GUI — 위젯을 데이터로
1. **B6.1 JSON Schema 파일** (G-SCHEMA-3, **viable**, S) — data/schemas/watch.json, phone.json (Draft 2020-12). 로드 검증 + 범위의 단일 진실원.
2. **B6.2 ✅ 1단계 완료 (2026-06-11)** — FieldSchema 팩토리(src/gui/FieldSchema) + bezel 그룹 PoC 마이그레이션 + 시그널 debounce 회귀 테스트(연속 3편집→정확히 1발화). 잔여: display/crown/buttons 그룹 이관.
3. **B6.3 🟨 절반 완료** — speaker_grille(step 7)·secondary_fillets(step 10) 편집 패널 가동(crown 식 enable-erase 의미론). 잔여: rear_sensors/lugs 용 제네릭 테이블 위젯(step 8-9).
4. **B6.4 ProductRegistry** (G-SCHEMA-2 수정판, M) — Meyer 싱글톤 + 제품/스텝 메타데이터. AppMenus 바이패스 제거, 폰 패널이 데이터로 떨어짐.
   ※ 풀 디스크립터 시스템 일괄 전환(G-SCHEMA-1 원안 XL)은 빅뱅 금지 — 위 증분 경로가 공식.

수용 기준: 워치 10스텝 전부 GUI 편집 가능 + 폰 4스텝 패널 가동 + parameter_panel 테스트(시그널 포함) green.

### B7. 자기-라벨 코퍼스 + 품질 게이트 — 라벨 천장 돌파
**통찰**: 777개 forward 생성기를 보유 → 라벨은 공짜다.
1. **B7.1 koo_corpus Phase 1** (G-CORPUS-1, **viable**, M) — 15 precise 스킬 × 5샘플, bounds는 tests/data/corpus_metadata.json 손작성(15개뿐), 스톡 100×100×50, STEP 라운드트립, {step, label} JSONL.
2. **B7.2 koo_metrics** (G-CORPUS-3, **viable**, M) — recall/precision/치수RMSE/혼동쌍(bore↔drill)/신뢰도 분포. precise 15종 recall≥85% 회귀 게이트.
3. **B7.3 티어 확장** (G-CORPUS-2 수정판) — Basic(커밋 게이트 15스킬), Nightly(스케일 스윕 1×/2.5×/10× — **AS1급 254mm 갭을 합성으로 재현**), Monthly(회전 포함 종합).
4. **B7.4 precise-tier 승격 기준 제도화** — 코퍼스 recall/precision 충족 시 compound를 캡 면제 티어로 승격하는 측정 기반 경로 (B1과 결합).
   ※ @corpus 주석+libclang 자동 추출(G-CORPUS-5)은 Phase 1엔 과설계 — 15개 손작성 후 가치 재평가.

수용 기준: 매 커밋 corpus-basic 게이트 가동, nightly 대시보드에 스킬별 recall 추세.

### B8. 검증 인프라 — false-green의 구조적 차단
1. **B8.1 scripts/verify.ps1 + verify.sh** (G-VERIFY-GATE-SCRIPT, **viable**, M) — clean → configure → full build → full ctest. "all green" 주장의 유일한 공인 출처. README/CONTRIBUTING 문서화.
2. **B8.2 ctest LABELS** (수정판, S) — 매크로 2곳 수정 + 직접 테스트 ~20개 주석: skill/integration/re/gui/io/process/watch/cam/primitive. 로컬 빠른 피드백 (단, scoped green ≠ full green 명시).
3. **B8.3 CI 점검** (S) — Linux 프리셋 OpenCASCADE_DIR 경로(lib/cmake/opencascade vs cmake)는 **Linux 러너 실측으로 판정** (Windows/Linux 레이아웃이 다른 게 정상일 수 있음 — 검증자 주장 그대로 믿지 말 것). OCCT 캐시 키에 빌드 플래그 변경 시 버전 범프 규칙 주석.
4. **B8.4 repo 위생** (S) — 루트 build_*.log/cmd 잔재 정리 + Testing/ gitignore + 루트 청결 CI 체크.
   ※ CMakePresets v6→v5 다운그레이드 **refuted** (v5도 CMake 3.24+ 필요 — 버전 매핑 오류). 로컬 CMake 업그레이드 안내가 정답. 단일 타겟 빌드 금지 **refuted** (근본 원인 오진 — 게이트 스크립트가 정답).

### B9. Datum/Adapt — 리사이즈의 완전한 일반화
1. **B9.1 ✅ 완료 (2026-06-13)** — applyPartDeltaToParams 에 축-인식 치수 스케일링: 피처 axis_dir(기본 −Z) 기준 axial(depth=|S⊙â|), radial(diameter/radius/dia/r = 평면 평균 스트레치, 원→타원 방지 등방근사), planar(length=sx/width=sy/height=sz 로컬 주축). Z축 피처는 axial=sz·radial=(sx+sy)/2 로 환원. sx=sy=sz=1 이면 no-op(회전·이동 불변). [원래: 치수 스케일링** (G-PARAM-1 수정판, M) — applyPartDeltaToParams 2차 패스: diameter/radius → (sx+sy)/2, depth → sz, length/width → 해당 축. 386개 스킬이 해당 필드 보유(실측). 네이밍 컨벤션 문서화.]
2. **B9.2 ✅ 완료** — 결합 affine 수용 테스트(rotate+scale+translate 동시) + 축별 depth/length/width 단언 추가 (parts_layout 13 TEST). [원래: 결합 affine 수용 테스트** (G-ACCEPT-1 수정판, M, B9.1 선행) — 회전 30°+스케일 1.5×+이동 동시, 6개 코어 스킬, 위치 ±0.1mm/치수 ±0.05mm.]
3. **B9.3 소유권 신뢰도** (G-AMBIG-1 수정판, M) — anchorPart가 단일 Part* 대신 confidence 랭킹 후보 리스트(1.0 유일 내포 / 0.85 동률 / 0.6 경계) 반환, JSON v2 직렬화 + v1 폴백. LLM/사용자 해소 경로의 토대.
   ※ feature-frame 기하 시그니처 매칭(G-ANCHOR-1)은 현 단계 **refuted** — AABB+신뢰도로 충분, 랜드마크 인프라 부재.

---

## 2. 하지 말 것 (refuted — 사유 보존)

| 제안 | refute 사유 (요지) |
|---|---|
| G-RECOG-3 스테이지드 파이프라인 | compound 수 오집계(142≠250), effort 10× 과소평가 |
| G-RECOG-5 공간 배제 존 | dedupe()가 이미 동일 기능 수행 중 |
| G-RECOG-6 replay fast-path | analyze() 경로에서 wp.features()는 항상 비어 있음 — 발동 불가 |
| G-STATIC-GUARD(clang-query) | 탐지 패턴이 코드베이스에 미존재 (codemod 후 grep이면 충분) |
| G-TRANSACTIONAL-1 볼륨 게이트 | 근본 원인은 실행 전(dedupe) — 게이트 발동 시점엔 이미 빈 shape |
| G-ACCEPTANCE-TEST-1 단독 재활성+가정 | 선행 픽스 미존재 가정 (단, **순수 재측정**은 B3.1로 채택) |
| G-SHRINK defeature 직행 | entry_face 미복원 — B4.2 폴백 전략 없인 재적용 불가 |
| G-PRECISE-TIER-EXPAND | 문제는 티어가 아니라 **레지스트리 미등록** (micro_drill 등 recognize 있는데 등록 안 됨 — 별도 1줄 픽스 후보) |
| G-CMAKE-PRESET-V5 | v5도 3.24+ 필요 — 문제 해결 안 됨 |
| G-SINGLE-TARGET 금지 | false-green 근본 원인 오진; 게이트 스크립트로 해결 |
| G-INTEGRATION-TEST-DEDUPE 신규 구현 | fingerprint dedupe 이미 랜딩(c35f72c) — 재활성 측정이 먼저 |
| G-ANCHOR-1 feature-frame 매칭 | 랜드마크 인프라 부재, AABB+confidence로 현 단계 충분 |

---

## 3. 실행 로드맵 (의존성 순)

**Phase 0 — 즉효 (1주) ✅ 완료 (2026-06-10)**
B3.1 재측정→PASS·재활성 ✅ → B3.2 보류(증거 대기) ⏸ → B8.1 verify 스크립트 ✅ → B8.2 LABELS ✅ → B2.3 계약 테스트 Phase1 ✅ (+precise-tier 7스킬 전파 수정) → B5.1 DFMReport 통일 ✅ → B8.4 repo 위생 ✅ → 보너스: micro_drill/gun_drill/multi_step_bore 미등록 확인·등록 ✅

**Phase 1 — 계약·측정 기반 ✅ 완료 (2026-06-11)**
B2.1 finalizeOutput 헬퍼+계약 문서화 ✅ → B2.2 codemod 307건(자동 304+수동 3: lapping_op/rest_milling/pocket_with_corner_relief; 위임형 11 검증·화이트리스트) ✅ → B2 가드 check_feature_chain.py(ctest: feature_chain_guard) ✅ → B7.1-7.2 koo_corpus+corpus_basic 게이트(`--gate-pre 0.99 --exempt ream,spot_face`) ✅ → B5.2-5.3 GeometryProbe(+holeToOuterMinDistance, geometry_probe ctest) ✅ → B6.1 watch/phone JSON Schema+spec_schema ctest ✅ → 보너스: precise-tier 미등록 인식기 4종(drill_through_hole/ream/spot_drill/spot_face) 등록 ✅

**Phase 1 실측 베이스라인 (corpus 3샘플/스킬, scale 1.0)**: recall_pre 1.000 = 13/15 (ream/spot_face 는 설계상 conf<0.7); post-dedupe recall 0 = 9스킬 — B1.5 dedupe 특이성 결함으로 등록. dim_err 최대 0.014mm.

**Phase 2 — 일반화 코어 (3-5주)**
B1.1 증거 부스트 → B1.2 의존성 인식(5종) → B3.3-B3.5 재실행 견고화 → B4.1 defeature enlarge → B5.4-B5.5 룰 레지스트리+폰 DFM → B6.2-B6.3 GUI 팩토리+step7-10 → B9.1-B9.2 치수 스케일링+수용 테스트

**Phase 3 — 돌파 완성 (5주+)**
B1.3 컴파운드 DSL(20-30종) → B4.2-B4.3 shrink/move+GUI 편집 → B6.4 ProductRegistry+폰 패널 → B7.3-B7.4 nightly 스케일 스윕+승격 제도 → B9.3 소유권 신뢰도

**게이트**: 각 Phase 종료 시 `scripts/verify.ps1` full green + 코퍼스 게이트(가동 후) 충족 없이는 다음 Phase 진입 금지.

---

## 4. 성공 기준 (측정 가능)

1. **인식**: 코퍼스 Basic에서 precise 15종 recall ≥85%·precision ≥90% 게이트 상시 가동; compound 5종이 합성 코퍼스에서 의존성 경로로 recall 측정치 보유; bare-stock false-positive 0 유지.
2. **계약**: 777 스킬 전부 finalizeOutput 단일 경로; 계약 테스트가 회귀를 커밋 시점에 차단.
3. **재실행**: bearing_housing 재실행 볼륨 오차 ≤25%, DISABLED 0개.
4. **편집**: 멀티솔리드 실 STEP에서 enlarge/shrink/move 3종 데모 + 회귀 테스트.
5. **DFM**: 신규 룰 추가 = JSON 1레코드; watch/phone 동일 엔진; 곡면 knife-edge 검출 가동.
6. **GUI**: 워치 10스텝 + 폰 4스텝 전부 패널 편집; 신규 제품 추가 시 GUI 코드 증가량 ≈ 0 (스키마만).
7. **인프라**: "all green" = verify 스크립트 출력만 인정; AS1급 스케일 갭이 nightly에서 합성 재현·감지.

---

## 5. 비평가 추가 발견 (백로그 — 트랙 외)

- **LLM 브리지 실연결**: LlmBridge는 HTTP 미구현 스텁 — WinHTTP(Win)/libcurl(Linux) 전송층 + env 플래그 게이트.
- **멀티 문서/멀티 뷰**: 단일 V3d_View+AIS 컨텍스트뿐 — DocumentManager/뷰 동기화는 Master Spec 스코프인데 미착수.
- **플랜 편집 undo/redo + 감사 추적**: PlanEditor에 트랜잭션/스냅샷 부재.
- **관측성**: 스킬/Boolean 타이밍 메트릭 수집 없음 — Timer RAII + 스킬별 실행 통계.
- **인식기 레지스트리 자동화**: Recognizer.cpp 수동 #include 캐스케이드 — 등록 매크로 or CMake 생성 + "전 스킬 등록" ctest.
- **lat.md 현행화**: PhoneFrontModel 상태, step7-10 GUI 상태 등 문서-코드 불일치 항목에 Last-Verified 날짜.
- **API 버저닝/CHANGELOG**: version.hpp + deprecated 마킹 규약.

---

## 6. 리스크

| 리스크 | 완화 |
|---|---|
| codemod 777파일 /WX 빌드 파손 | 100개 배치 + 배치마다 full ctest + 클린 빌드 |
| 증거 부스트가 false-positive 재유입 | 코퍼스 게이트 선행(B7이 B1.3보다 먼저 가동), bare-stock 0 단언 유지 |
| Defeaturing이 특정 토폴로지에서 실패 | per-call BRepCheck + 실패 시 원본 무손상 보장(비파괴 설계), 지원 스킬 화이트리스트 |
| GUI 마이그레이션 중 시그널 누락 | 그룹별 단일 커밋 + 시그널 발화 회귀 테스트 선행 |
| 코퍼스 런타임 폭주 | 티어 분리(커밋 게이트는 15스킬×5샘플만), nightly/monthly 분리 |
