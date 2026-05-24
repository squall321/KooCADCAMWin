# Coding Standards

KooCADCAM C++17 코드베이스 전체에 적용되는 스타일, 명명, 관용구, 검토 기준을 정의한다.
OCCT 8.0 기반선([[process/occt8-migration-cookbook]])과 빌드 인프라([[architecture/build-and-deps]])에
직접 연동된다.

---

## C++17 필수 기능 목록

OCCT 8.0이 C++17을 기준선으로 요구하므로 프로젝트 전체가 C++17을 사용한다.
아래 기능들은 **적극 활용 권장** 목록이다.

| 기능 | 사용 목적 |
|---|---|
| `if constexpr` | 템플릿 분기 — `#ifdef` 대신 |
| `std::optional<T>` | 선택적 반환값 (nullable 포인터 대체) |
| `std::variant<Ts...>` | 타입 안전 유니온; EvalRep 디스패치 패턴 |
| `std::string_view` | 읽기 전용 문자열 인자 (복사 없음) |
| `std::shared_mutex` | 다중 읽기 / 단일 쓰기 잠금 |
| 구조적 바인딩 (structured bindings) | 가독성 향상 (`auto [key, val] = ...`) |
| fold expressions | 가변 인자 패킹 표현 단순화 |
| `[[maybe_unused]]` | 미사용 변수/파라미터 경고 억제 (Standard_UNUSED 대체) |
| `[[nodiscard]]` | 반환값 무시 금지 강제 |

---

## 스타일: .clang-format

프로젝트 루트에 `.clang-format` 파일을 두고 clang-format 18+로 강제 적용한다.
pre-commit 훅에서 포맷 위반 시 커밋을 차단한다([[process/test-strategy#CI 구조]]).

```yaml
# .clang-format  (LLVM 기반, KooCADCAM 맞춤)
BasedOnStyle: LLVM
Language: Cpp
Standard: c++17

# 컬럼 제한
ColumnLimit: 110

# 들여쓰기
IndentWidth: 4
TabWidth: 4
UseTab: Never
AccessModifierOffset: -4
NamespaceIndentation: None
IndentCaseLabels: false

# 중괄호
BreakBeforeBraces: Attach
AllowShortFunctionsOnASingleLine: Inline
AllowShortIfStatementsOnASingleLine: Never
AllowShortLoopsOnASingleLine: false

# 포인터/레퍼런스 정렬
PointerAlignment: Left

# 파라미터 정렬
AlignAfterOpenBracket: Align
BinPackArguments: false
BinPackParameters: false

# 주석
ReflowComments: true
SpacesBeforeTrailingComments: 2

# 인클루드 정렬
SortIncludes: CaseSensitive
IncludeBlocks: Regroup
IncludeCategories:
  - Regex: "^<(BRep|Topo|gp_|AIS_|V3d_|Standard_)"
    Priority: 2
  - Regex: "^<Q"
    Priority: 3
  - Regex: "^<"
    Priority: 4
  - Regex: ".*"
    Priority: 1
```

---

## 명명 규약

| 대상 | 규칙 | 예시 |
|---|---|---|
| 클래스 / 구조체 | PascalCase | `WatchFrontModel`, `DfmResult` |
| 자유 함수 | camelCase | `buildBezel`, `extractCameraHoles` |
| 멤버 변수 | `m_camelCase` | `m_cornerRadius`, `m_cameraSpecs` |
| 정적 멤버 상수 | `kPascalCase` | `kMinWallThickness` |
| 파라미터 / 지역 변수 | camelCase | `wallThickness`, `idx` |
| 네임스페이스 | lowercase, 중첩 허용 | `koocadcam::engine`, `koocadcam::ui` |
| 열거형 값 | PascalCase | `DfmSeverity::Error` |
| 매크로 (최소화) | UPPER_SNAKE | `KOOCAD_ASSERT(...)` |

### 네임스페이스 계층

```cpp
namespace koocadcam {
    namespace engine { /* 형상/DFM/CAM 로직 */ }
    namespace ui     { /* Qt 위젯, 컨트롤러 */ }
    namespace io     { /* STEP/JSON 입출력 */ }
    namespace test   { /* 테스트 헬퍼 */ }
}
```

---

## OCCT 관용구

### 스마트 포인터
```cpp
// OCCT 핸들 (intrusive reference count)
Handle(AIS_Shape) shape = new AIS_Shape(topo);

// 프로젝트 내부 heap 객체 — std::shared_ptr / std::unique_ptr 사용
std::unique_ptr<WatchFrontModel> model = std::make_unique<WatchFrontModel>(spec);
```

### 기본 타입 선택
```cpp
// OCCT 대면 API: OCCT 타입 사용
Standard_Real    radius  = 2.5;
Standard_Integer count   = cameras.size();

// 프로젝트 내부 코드: C++ 표준 타입 사용
double wallThickness = 0.45;
int    cameraCount   = 3;
```

### 수학 함수 (OCCT 8.0 기준)
```cpp
// 8.0 이후: std::* 함수 직접 사용
double angle = std::acos(dot);
double len   = std::sqrt(dx*dx + dy*dy);
double s     = std::sin(theta);

// 7.x 잔재 금지 (마이그레이션 상세: [[process/occt8-migration-cookbook]])
// ACos(x), Sqrt(x), Sin(x)  ← 컴파일 경고 후 제거 예정
```

---

## 예외 정책

```cpp
// 발생: 표준 throw (OCCT 8.0 방식)
if (radius < 0.0)
    throw Standard_Failure("WatchFrontModel: radius must be non-negative");

// 캐처: API 경계에서는 std::exception& 로 수신
//   (Standard_Failure가 std::exception 파생이므로 호환)
try {
    model.build();
} catch (const std::exception& ex) {
    spdlog::error("[Engine] build failed: {}", ex.what());
    return tl::unexpected(Error{ErrorCode::BuildFailure, ex.what()});
}

// 금지: 레거시 Raise() 및 무기명 catch(...)
// Standard_Failure::Raise("msg");  ← 금지
```

---

## 에러 모델

암묵적 실패(silent failure)를 금지한다. 함수가 실패할 수 있을 때는 항상
`tl::expected<T, Error>` 를 반환한다.

```cpp
// koocadcam/error.hpp  (RESERVED PATH — src/ 미존재)
// @lat: [[engine/parametric-templates#Error Model]]
struct Error {
    enum class Code { BuildFailure, DfmViolation, IoError, NotImplemented };
    Code        code;
    std::string message;
};

template<typename T>
using Result = tl::expected<T, Error>;

// 사용 예
Result<TopoDS_Shape> buildWatch(const WatchSpec& spec);
```

---

## 로깅 정책

```cpp
#include <spdlog/spdlog.h>

// 레벨 사용 기준
spdlog::trace("...");   // 내부 알고리즘 단계별 추적 (기본 비활성화)
spdlog::debug("...");   // 개발 중 진단 정보
spdlog::info("...");    // 정상 이벤트 (STEP 출력 완료 등)
spdlog::warn("...");    // DFM Warning, 비권장 경로
spdlog::error("...");   // 복구 가능 오류
spdlog::critical("..."); // 프로세스 종료 필요 수준

// std::cout / printf 사용 금지
// 구조화 로그: spdlog::info("[Module] key={}", value);
```

---

## lat.md 주석 규약

모든 헤더 파일에서 최상위 엔진 클래스를 선언할 때 메서드 선언 옆에
`// @lat:` 주석을 붙인다.

```cpp
// src/engine/dfm/WallThicknessRule.hpp  (RESERVED PATH)
// @lat: [[engine/dfm-rules#WallThicknessRule]]
class WallThicknessRule {
public:
    // @lat: [[engine/dfm-rules#evaluate]]
    DfmResult evaluate(const TopoDS_Shape& shape) const;

    // @lat: [[engine/dfm-rules#threshold]]
    Standard_Real threshold() const;
};
```

`require-code-mention: true` 프론트매터를 가진 파일([[process/test-strategy]])은
`npx lat.md check` 실행 시 이 주석이 없으면 lint 실패한다.

---

## Node.js (lat.md CLI) — 예약 사항

- Node.js **22+** 가 설치된 후 `npx lat.md check` 를 CI에 통합한다.
- 현재는 비활성화 상태이며 pre-commit 훅에서 건너뛴다.
- 활성화 시 [[architecture/build-and-deps]] 의 CI 런너 이미지에 Node.js 22 레이어 추가 필요.

---

## 코드 리뷰 정책

| 조건 | 필요 승인 수 | 비고 |
|---|---|---|
| 일반 PR | 1 명 + CI 그린 | 기본 |
| 대형 PR (> 500 LoC 변경) | 2 명 + CI 그린 | LoC = 추가+삭제 합계 |
| 라이선스 변경 포함 | 1 명 + `[license-ack]` 태그 | [[process/data-assets#라이선스 컴플라이언스 CI 게이트]] |

리뷰어 체크리스트:
- [ ] 명명 규약 준수
- [ ] `std::cout` / `printf` 없음
- [ ] 새 OCCT 잔재(`Raise(`, `ACos(`, `Standard_UNUSED`) 없음
- [ ] `tl::expected` 에러 전파 누락 없음
- [ ] lat.md `@lat:` 주석 추가 여부 (엔진 헤더 신규 추가 시)

---

## 크로스-링크 요약

- [[process/test-strategy]] — CI 단계, pre-commit 훅, lat.md 주석 관련 테스트
- [[process/occt8-migration-cookbook]] — Raise() → throw, ACos → std::acos 등 마이그레이션
- [[architecture/build-and-deps]] — clang-format/tidy 버전, vcpkg, CI 러너
