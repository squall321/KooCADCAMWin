# Machining Skill Library

> **2026-05-27 — Architecture pivot.** v0.1 (tag `v0.1-parametric-design`)
> 은 워치 전용 파라메트릭 디자인 툴이었다. 이후의 KooCADCAM 은 **머시닝
> 스킬 라이브러리 + 합성/분석 양방향 루프** 로 진화한다. 이 문서는
> 새 아키텍처의 중심 계약을 정의한다.

## 동기

이전 framing 의 한계:
- `WatchFrontModel::buildBezel` 같은 메서드는 **워치-특정 디자인 step**.  분석 불가능 (이 형상이 어떤 step 의 산물인지 식별 X).
- 데이텀이 spec 의 수치값으로 hardcode (e.g., "베젤 width=3mm" — 케이스 외경 변화에 따라가지 못함).
- 다른 제품(폰/태블릿)으로 재사용 어려움.

새 framing 의 핵심 통찰:
- **모든 가공은 reusable skill 의 합성**으로 표현 가능.
- **양방향**: 합성(forward) + 분석(reverse).  분석 가능해야 RE / LLM 어댑테이션이 동작.
- **데이텀 기반**: skill 은 데이텀(face / edge / 추론된 기준)을 받음; 부품 배치가 바뀌면 데이텀 cascade 로 자동 따라감.

이 framing 으로 가능한 워크플로:
```
INPUT  : 기존 프레임.step + 기존 부품 layout.step + 새 부품 layout.step
PIPE   : RE(기존 프레임) → process_plan_v1
       : extract constraints(부품 ↔ skill datum 의존 그래프)
       : LLM adapt(부품_v2 입력 → process_plan_v2)
       : execute(process_plan_v2) → 새 프레임
OUTPUT : 새 프레임.step + DFM report
```

---

## 5-Layer 아키텍처

```
┌─ Layer 5  llm_adapter/   자연어 ↔ skill 시퀀스, 부품 변경 → process 재구성 ─┐
├─ Layer 4  re/            CAD → skill 시퀀스 (feature recognition + seq) ─┤
├─ Layer 3  process/       process plan 표현, 부분 재실행, datum 그래프    ─┤
├─ Layer 2  skills/    ★   packaged 머시닝 skill (synthesis + analysis)   ─┤
└─ Layer 1  engine/primitives/   OCCT 추상 (cylinder/box/cut/fillet …)    ─┘
```

- Layer 1 은 워치/폰 모델에서 이미 만들어둠 (`prim::cylinder`, `prim::cut`, …).  v0.1 era 의 자산.
- Layer 2 가 본 노드의 주제 — 모든 skill 의 공통 contract.
- Layer 3-5 는 후속 마일스톤.

---

## Skill 의 7가지 조건 (포장된 = packaged)

각 skill 은 다음 7 조건을 모두 갖춰야 한다.  단 하나라도 빠지면
RE/adaptation 루프 어디선가 깨진다.

| # | 조건 | 코드에서의 표현 |
|---|---|---|
| 1 | 명시적 타입 계약 | `Input` 구조체 + `apply`/`validate`/`recognize` 시그니처 |
| 2 | 데이텀 의존성 명시 | `FaceDatum` / `EdgeDatum` / `VertexDatum` variant 로 입력에 데이텀 references 포함 |
| 3 | 결정론 | 동일 Input → 동일 output shape + 동일 FeatureSignature |
| 4 | 인식 가능 시그니처 | `FeatureSignature.pattern` 에 토폴로지 패턴 (cyl_face_count, axis_dir, dims …) |
| 5 | 파라미터 복원 | `recognize(workpiece)` 가 패턴에서 원래 Input 역산 |
| 6 | 합성 안전 | 출력 워크피스는 다른 skill 의 valid 입력 (orphan face 없음, topology valid) |
| 7 | 내부 DFM 게이트 | `validate(in)` 이 `DFMReport` 반환, 위반이 `apply()` 차단 |

레퍼런스 구현: [[src/skills/drill_hole.hpp]] + [[src/skills/drill_hole.cpp]].  모든 신규 skill 은 이 패턴을 strict imitation.

---

## 하이브리드 데이텀

skill 의 입력은 **explicit + inferred 둘 다 받음**.

```cpp
FaceDatum d1 = FaceIdRef{ 3 };                       // explicit — 워크피스의 index 3 face
FaceDatum d2 = FaceByNormal{ gp_Dir(0,0,1), 5.0 };   // inferred — Z+ 법선 가진 face
FaceDatum d3 = FaceLargestPlanar{};                  // inferred — 가장 큰 planar
FaceDatum d4 = FaceTopAtXY{ 10.0, 5.0 };             // inferred — (10,5) XY 의 topmost Z planar
```

variant 유니온이라 같은 위치에 두 형식 자유롭게 혼용.  `Workpiece::resolve(datum)` 가 양쪽 모두 해석.

지원 inferred 데이텀 (`Datum.hpp`):

| 종류 | 의미 |
|---|---|
| `FaceByNormal{dir, tol_deg, variant}` | dir 와 가장 가까운 법선의 planar face (variant: any/largest/smallest) |
| `FaceLargestPlanar` | 면적 최대 planar |
| `FaceByRay{origin, dir}` | ray hit 결과 (M2 본격 구현) |
| `FaceCylinderByAxis{axis, tol_deg}` | axis 와 평행한 cylindrical face |
| `FaceTopAtXY{x, y}` | (x,y) 위의 가장 위에 있는 planar |
| `EdgeAtZ{z, tol}` | Z 좌표가 z 인 edge |
| `EdgeByCircle{radius, tol}` | 원형 edge (반지름 r 매칭) |
| `EdgeBetweenFaces{f1, f2}` | 두 face 의 공유 edge (M2) |
| `VertexAtPoint{p, tol}` | 좌표 p 의 vertex |

---

## Workpiece 와 stock

`Workpiece` (`src/skills/Workpiece.hpp`) 는 TopoDS_Shape + 안정적 face/edge/vertex 열거 + feature history.

- 열거: `NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher>` (OCCT 8.0 권장; deprecated `TopTools_IndexedMapOfShape` 회피)
- 데이텀 해석: `resolve(FaceDatum) -> std::optional<int>`
- 기하 도우미: `faceNormal/Center/Area`, `edgeMidZ`, `isFacePlanar/Cylinder`, `boundingBox`

Stock factory (`src/skills/Stock.hpp`):
- `createCuboidStock(w, h, d, material)` — slice 1 의 표준 시작점
- `createCylindricalStock(dia, thickness, material)` — 라운드 stock

향후 확장 (M2+):
- `createSTEPStock(path)` — 임의 STEP 입력
- `createExtrudedProfile(wire, length)` — 사용자 정의 단면

---

## Skill 카탈로그 (slice 별)

| Slice | Skills | 상태 |
|---|---|---|
| **1** (commit 1cd2f56) | drill_hole | ✅ shipped — 7 조건 모두 충족 |
| **2** (in-progress) | counterbore, countersink, mill_circular_pocket, mill_rect_pocket, mill_slot, fillet_edge, chamfer_edge, face_milling, bore_cylindrical, **bore_with_shelf**, hollow_cavity | 🟡 sub-agent 4명 병렬 작업 중 |
| **3** | spot_drill, ream, mill_open_pocket, profile_milling | 📋 다음 작업 |
| **4** | tap_thread, thread_mill (helical sweep — 새 primitive 필요) | 📋 M4 |
| **5** | engrave_text (font glyph), engrave_path | 📋 M5 |
| **compound** | drill_and_tap, bore_and_finish, pocket_with_corner_relief | 📋 P0 skill 들 안정화 후 |

총 카탈로그 목표 (Tier 1): **20+ skill**.

### 카테고리별 의미

| 카테고리 | 대표 skill | 용도 |
|---|---|---|
| Drilling | drill_hole, counterbore, countersink, spot_drill, ream | 모든 홀 가공 |
| Milling pockets | mill_*_pocket, mill_slot | 다양한 단면의 함몰 |
| Milling profile/edge | face_milling, profile_milling, chamfer_edge, fillet_edge | 외관 마감 |
| Boring | bore_cylindrical, bore_with_shelf, bore_taper | 정밀 직경 + 스텝 보어 |
| Hollowing | hollow_cavity | 솔리드 → 셸 변환 |
| Threading | tap_thread, thread_mill | 나사 |
| Engraving | engrave_text, engrave_path | 문자/로고 |
| Compound | drill_and_tap, … | 자주 묶이는 macro |

---

## 검증 시나리오 — Round-trip

slice 1 에서 확립된 검증 패턴:

1. **Stock 생성**: `createCuboidStock(...)`
2. **Synthesis**: skill.apply(stock, input) → workpiece
3. **STEP export**: `StepIO::write(workpiece.shape, path)`
4. **Reimport**: `StepIO::read(path)` — 메타데이터/시그니처는 **고의 폐기**
5. **Recognize**: `skill.recognize(reimported_workpiece)` → 후보 list
6. **Verify**:
   - **(a) 파라미터 복원**: 복원된 input 의 수치값이 원본의 tolerance 내
   - **(b) Topology hash**: synthesis(복원된 input) 의 face/edge/vertex 개수가 원본과 일치
   - **(c) 부피 일치**: volumeOf(원본) == volumeOf(재합성)  (ε 이내)

이 3가지 통과 = skill 의 "포장" 이 양방향 일관됨.  drill_hole 의 `RoundTripViaStep` 케이스가 정확히 이 패턴.

향후 SHA-256 단계 추가 — STEP write 의 결정론 (timestamp 고정, header 정규화) 이 안정되면.

---

## 생성형 grammar-compound skills (B1.2)

[[engine/reverse-route#compound-grammar]] 의 선언적 grammar 가 정밀 atom 들에서
RING/ROW/GRID/STACK 패턴을 **인식(recognize)** 한다면, 아래 4개 skill 은 그 반대
방향 — 같은 패턴을 **생성(synthesize)** 한다.  공통 구현 idiom: `drill_hole::apply`
를 hole 마다 합성(entry/through 해석과 chain 계약을 검증된 atom 으로부터 상속),
그 위에 compound `FeatureSignature` 를 stamp.  recognition 은 koo_re grammar 에만
있으므로 각 skill 의 `recognize()` 는 빈 vector 반환(레지스트리 중복 카운트 방지).
이로써 인식된 패턴이 **편집 가능한 replayable step** 이 된다 — hole_count·pitch·
diameter 를 바꿔 재실행하면 패턴 전체가 재생성.

### bolt_circle_pattern

지름 `bolt_circle_dia_mm` 의 피치원 위에 `hole_count`(>=3) 개의 동일 홀을 등각
배치해 드릴.  `validate` 는 hole_count/직경 sanity + 인접 홀 chord 간극 1.5 mm
(DFM-003 미러) 검사.  recovered_params: `{hole_count, bolt_circle_dia_mm,
hole_dia_mm, center_x/y, axis_dir, start_angle_deg}`.
소스: [[src/skills/bolt_circle_pattern.hpp]] · [[src/skills/bolt_circle_pattern.cpp]].

### linear_hole_array

방향 `(dir_x, dir_y)` 로 `pitch_mm` 등간격, `hole_count`(>=3) 개의 동일 홀 한 줄을
드릴.  recovered_params: `{hole_count, hole_dia_mm, pitch_mm, start_x/y, direction,
axis_dir}`.  소스: [[src/skills/linear_hole_array.hpp]].

### rectangular_hole_grid

origin 코너 + 두 축 방향(u/v) 으로 채워진 `cols×rows`(>=6) 동일 홀 격자를 드릴
(예: 워치 스피커 그릴).  recovered_params: `{cols, rows, hole_dia_mm, origin_x/y,
u/v basis, pitch_u/v_mm}`.  소스: [[src/skills/rectangular_hole_grid.hpp]].

### coaxial_step_bore

동심 full bore `steps`(>=3, 서로 다른 직경)를 entry face 에서 각 step 의 누적
depth 까지 보어(최심 step 은 through 가능).  step 마다 `{diameter_mm, depth_mm,
through}`.  소스: [[src/skills/coaxial_step_bore.hpp]].

---

## param-clamp

Layer 3 — recovered-parameter clamp (B3.4).  RE 로 복원한 ProcessPlan 을 새
stock 에 재실행할 때, 일부 recovered param 이 over-specify 되어 skill 의 `apply()`
를 crash 시킨다(예: hollow_cavity 의 wall_thickness 가 stock 보다 커서 잔여 cavity
가 non-positive).  `Executor` 가 매 step 의 params 를 이 clamp 에 먼저 통과시켜,
live stock extent 로 물리적으로 불가능한 값을 가장 가까운 valid 값으로 rewrite.
`skill_id` 별 순수 함수 레지스트리(`clampTable()`)이며 entry 없는 skill 은 no-op,
clamp 는 절대 throw 하지 않음.  소스: [[src/process/ParamClamp.hpp]] ·
[[src/process/ParamClamp.cpp]].  회귀: `tests/process/param_clamp_test.cpp`.

### thin-wall 보존 (silent-corruption 수정)

`hollow_cavity` clamp 가 wall 을 `0.45 * min(XY extent)` 로만 제한한다 — `apply()`
는 wall 이 XY extent 의 절반을 넘을 때만 throw 하므로 그 crash 경계 **안쪽만** 막으면
충분하다.

- **제거**: DFM-001 의 0.4 mm wall **floor**.  이 floor 는 유효한 0.35 mm
  watch-grade wall 을 silent 하게 0.4 mm 로 over-clamp 했다.
- **근거**: thin wall 은 `apply()` 를 crash 시키지 않는다(DFM warning 일 뿐, 측정
  되는 [[engine/dfm-rules#DFM-001]] watch 임계가 0.35 mm).  clamp 는 crash-class
  입력만 고쳐야 하므로 valid thin wall 을 그대로 보존한다.

> **원칙** — clamp 는 crash-class 입력(`wall > 0.45*min(XY extent)`)만 rewrite 하고,
> 유효한 measured/structural 값은 절대 silent corrupt 하지 않는다.  같은 교훈:
> [[engine/reverse-route#Recognizer]], [[engine/skills#auto_rib_between_two_walls]],
> [[engine/dfm-rules#DFM-018]].

---

## auto_rib_between_two_walls

Mechanical-structure skill (BASF 플라스틱 리브 설계 비율 기반).  두 벽 face 사이에
리브 블록을 fuse 하고 양 끝 junction 에 chamfer cut.  `recognize()` 는 5 mm 보다
좁고 anti-parallel 한 평면 쌍을 리브 후보로 잡는다.  소스:
[[src/skills/auto_rib_between_two_walls.cpp]].  회귀:
`tests/skills/auto_rib_between_two_walls_test.cpp`.

### slab gate (silent-corruption 수정)

리브 후보가 부품 자신의 두 외곽 면(= 얇은 slab)일 때 false-recognize 되지 않도록
거르는 게이트.  단일 ratio 게이트(`dist > 0.5 * normal-extent`)만 쓰면 **얇은 body
안의 얇은 리브**(예: 3 mm 벽 안의 1.6 mm 리브)를 slab 으로 오인해 false-reject 했다.

- **현재**: ratio AND absolute 둘 다 만족할 때만 slab 으로 reject —
  `dist > 0.5 * extN` **그리고** `dist > kSlabAbsMm`(`= 3.0` mm).
- 결과: 얇은 리브는 살아남고, 진짜 slab(80×80×4 plate 의 4 mm gap — fpscan 생존
  케이스)은 두 조건을 모두 넘겨 여전히 reject.

> **원칙** — gate 는 true-defect(진짜 slab)만 거르고 유효한 structural 형상(얇은
> 리브)을 silent 하게 죽이지 않는다.  ratio 단독은 thin-body 케이스에서 over-fire
> 하므로 absolute floor 와 AND 결합.  같은 교훈:
> [[engine/reverse-route#Recognizer]], [[engine/skills#param-clamp]],
> [[engine/dfm-rules#DFM-018]].

---

## 참고

- [[engine/dfm-rules]] — 25개 DFM 규칙 카탈로그 (skill validate() 가 호출)
- [[engine/parametric-templates]] — v0.1 era 의 primitives 레이어 (Layer 1)
- [[engine/feature-watch]] / [[engine/feature-phone]] — v0.1 era 의 제품-특정 모델 (Layer 3 데모로 재분류)
- [[process/occt8-migration-cookbook]] — OCCT 8.0 typedef 주의사항 (NCollection_*)
- [[process/test-strategy]] — round-trip 회귀 정책
