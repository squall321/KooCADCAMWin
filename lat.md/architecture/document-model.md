# Document Model

이 문서는 KooCADCAM의 문서 데이터 모델 선택 근거, 마이그레이션 경로,
그리고 영속성 전략을 기술한다.

---

## 결정 테이블: own Doc vs TDocStd_Document (OCAF)

| 기준 | own `class Doc` | `TDocStd_Document` (OCAF) |
|---|---|---|
| **구현 복잡도** | 낮음 — 표준 C++ 클래스 | 높음 — TDF 레이블 트리, Handle 체계 |
| **Undo/Redo** | 수동 Command 스택 구현 필요 | `TDocStd_Document::Undo()`로 자동 |
| **문서 간 참조** | feature_id 정수 (약한 참조) | XLink (`TDF_Reference`) 강한 참조 |
| **영속성** | JSON 스펙 + STEP/XCAF 수출 | XBF 바이너리, STEP/XCAF 내장 지원 |
| **Viewer 연동** | 직접 `AIS_InteractiveContext` 보유 | OCAF-AIS 브릿지 (`TPrsStd_AISViewer`) |
| **테스트 용이성** | 일반 단위 테스트로 충분 | OCAF 환경 초기화 필요 |
| **라이선스 의존** | 없음 | OCCT LGPL (이미 의존) |
| **M1 적합성** | ✅ 충분 | ❌ 오버엔지니어링 |

---

## 권고사항

**M3 완료까지는 own `class Doc`을 사용한다.**

OCAF 전환 트리거 (셋 중 하나라도 해당되면):

1. **Undo/Redo 요구 규모**: 단일 파라미터 변경이 여러 Doc에 걸쳐
   연속 트랜잭션을 요구하고, 수동 Command 스택으로 관리가 어려워질 때.
2. **XLink 크로스 문서 참조**: 참조 모델의 특정 face를 생성 모델의
   feature가 직접 참조하는 구조가 필요해질 때.
3. **XBF 영속성**: JSON + STEP 조합으로는 재현할 수 없는 복잡한
   문서 상태를 바이너리로 저장해야 할 때.

---

## own Doc → OCAF 마이그레이션 경로 (API 비호환 최소화)

### 단계 1 — 인터페이스 격리 (지금 해야 할 것)

`Doc` 클래스를 순수 가상 인터페이스로 분리한다.

```cpp
// RESERVED: src/engine/IDoc.hpp
class IDoc {
public:
    virtual ~IDoc() = default;
    virtual int                           id()         const = 0;
    virtual std::string                   name()       const = 0;
    virtual const TopoDS_Shape&           rootShape()  const = 0;
    virtual Handle(AIS_InteractiveContext) aisContext() const = 0;
    virtual void                          highlight(int featureId, bool on) = 0;
    virtual const gp_Trsf&               registration() const = 0;
    // ... 기타 공개 API
};

// M1~M3: 구현체
class Doc : public IDoc { /* ... */ };

// M4+: OCAF 래퍼
class OcafDoc : public IDoc {
    Handle(TDocStd_Document) m_tdoc;
    // IDoc 인터페이스를 TDF 레이블에 위임
};
```

### 단계 2 — DocManager 인터페이스 기반으로 교체

```cpp
// DocManager는 IDoc* 또는 std::shared_ptr<IDoc>만 다룬다.
class DocManager {
public:
    std::shared_ptr<IDoc> open(const std::string& path);
    void                  close(int docId);
    IDoc*                 active() const;
private:
    std::vector<std::shared_ptr<IDoc>> m_docs;
    int                                m_activeId = -1;
};
```

### 단계 3 — OCAF 도입 시 OcafDoc 구현

```cpp
// OcafDoc::aisContext() 구현 예시
Handle(AIS_InteractiveContext) OcafDoc::aisContext() const
{
    Handle(AIS_InteractiveContext) ctx;
    // TPrsStd_AISViewer::Find() 또는 직접 attribute에서 꺼냄
    TPrsStd_AISViewer::Find(m_tdoc->Main(), ctx);
    return ctx;
}
```

GUI 코드는 `IDoc*` 인터페이스를 통해서만 접근하므로 구현체 교체 시
UI 레이어를 수정할 필요가 없다.

---

## 영속성 전략 (마일스톤별)

### M1 — JSON 스펙 + STEP 수출

```
파라미터 입력
    │
    ▼
JSON 파일 (human-readable, Git-diff 가능)
    │
    ▼ JsonSpecLoader::load()
    │
    ▼
FeatureBuilder::build() → TopoDS_Shape
    │
    ▼
StepExporter::save() → STEP AP214
```

JSON 스키마 예시 (watch case):
```json
{
  "template": "watch_front_case_v1",
  "params": {
    "outer_width_mm": 44.0,
    "outer_height_mm": 38.0,
    "thickness_mm": 1.2,
    "corner_radius_mm": 6.0,
    "lug_width_mm": 22.0
  }
}
```

### M2 — Morphing 중간 상태 저장

JSON에 `morph_weights` 배열 추가. 별도 파일 포맷 불필요.

### M3 — RE 파이프라인 결과

STL 원본 + 인식된 feature JSON + 생성 STEP 묶음을 프로젝트 폴더로 저장.

### M4+ — OCAF XBF (조건부)

OCAF 전환 트리거 충족 시 `TDocStd_Application::SaveAs()` 사용.
기존 JSON/STEP 수출 경로는 호환성을 위해 유지.

---

## 불변 규칙

- `TopoDS_Shape`는 직렬화하지 않는다. 항상 파라미터 → 빌더 → 형상
  재생성 경로를 유지한다.
- STEP 파일은 최종 결과 교환 포맷이지 내부 영속성 포맷이 아니다.
- JSON 스펙 파일은 Git 버전 관리 대상이다 (diff 가능, 충돌 해결 가능).

---

## 관련 문서

- [[architecture/multi-document]] — AIS_InteractiveContext per Doc, feature_id 동기화
- [[architecture/build-and-deps]] — OCCT LGPL 동적 링크 요건
- [[engine/parametric-templates]] — JSON 스펙 파라미터 정의
- [[engine/reverse-route]] — STL→STEP RE 파이프라인
- [[process/coding-standards]] — IDoc 인터페이스 설계 규약
