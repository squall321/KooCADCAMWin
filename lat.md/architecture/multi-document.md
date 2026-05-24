# Multi-Document

KooCADCAM은 참조 모델, 파라메트릭 생성 모델, RE 결과물 등을 동시에
열 수 있는 다중 문서 환경을 지원한다. 각 `Doc`은 독립적인
`AIS_InteractiveContext`를 보유한다.

---

## AIS_InteractiveContext: Doc 당 1개 원칙

### 왜 공유하지 않는가

| 이유 | 설명 |
|---|---|
| **선택 범위 격리** | 공유 context에서는 문서 A의 선택이 문서 B의 AIS_Shape에 영향을 줄 수 있다. Doc마다 분리하면 `Select()`의 영향 범위가 명확하다. |
| **정리 용이성** | Doc을 닫을 때 해당 context만 `Remove()` 하면 된다. 공유 context에서는 어느 shape가 어느 Doc 소속인지 태그로 추적해야 한다. |
| **교체 가능성** | Doc을 갱신(재생성)할 때 context를 통째로 교체할 수 있어 부분 제거 버그를 방지한다. |
| **FreeCAD 패턴** | `src/Gui/Document.cpp`에서 Doc마다 별도 `SoSeparator`(Coin3D)를 사용하는 것과 동일한 철학. AIS 버전으로 대응. |

### 구현 위치

`DocManager`가 모든 Doc을 소유하며, 활성 Doc의 context를 `ViewGrid`에
전달한다 (RESERVED: [[src/engine/DocManager.hpp]]).

---

## Doc 클래스 의사(pseudo) API

```cpp
// RESERVED: src/engine/Doc.hpp

#pragma once
#include <AIS_InteractiveContext.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Trsf.hxx>
#include <optional>
#include <string>
#include <vector>
#include <memory>

enum class DocSource { Parametric, Morphed, ReverseEngineered };

struct FeatureNode {
    int                        id;
    std::string                label;
    TopoDS_Shape               shape;       // 해당 feature의 부분 형상
    std::vector<int>           children;    // feature 그래프 (DAG)
    std::optional<std::string> errorMsg;    // DFM/CAM 위반 메시지
};

class Doc {
public:
    // --- 식별 ---
    int            id()     const;   // 프로세스 내 유일 정수 ID
    std::string    name()   const;   // 사용자 표시 이름 (예: "Watch_v3")
    DocSource      source() const;

    // --- 형상 ---
    const TopoDS_Shape&              rootShape() const;
    void                             setRootShape(const TopoDS_Shape& s);

    // --- Feature 그래프 ---
    const std::vector<FeatureNode>&  features() const;
    FeatureNode*                     findFeature(int featureId);

    // --- AIS ---
    Handle(AIS_InteractiveContext)   aisContext() const;
    void                             displayShape();   // rootShape를 AIS에 추가
    void                             highlight(int featureId, bool on);

    // --- 좌표 정합 (diff 정렬용) ---
    const gp_Trsf&                   registration() const;
    void                             setRegistration(const gp_Trsf& t);

private:
    int                              m_id;
    std::string                      m_name;
    DocSource                        m_source;
    TopoDS_Shape                     m_rootShape;
    std::vector<FeatureNode>         m_features;
    Handle(AIS_InteractiveContext)   m_aisCtx;
    gp_Trsf                         m_registration;
};
```

---

## 문서 간 선택 동기화

`featureSelected(int docId, int featureId)` Qt 신호는 모든 열린 Doc에
브로드캐스트된다. 각 Doc은 자신의 `AIS_InteractiveContext`에서 해당
`feature_id`를 가진 AIS_Shape를 하이라이트한다.

```
[KooViewWidget::mousePressEvent]
    │
    └─► emit featureSelected(doc_id, feature_id)
              │
              ▼
        DocManager::onFeatureSelected(int docId, int featureId)
              │
              ├─► Doc_0->highlight(featureId, true/false)
              ├─► Doc_1->highlight(featureId, true/false)
              └─► Doc_N->highlight(featureId, true/false)
```

`feature_id`는 파라메트릭 빌더가 shape를 생성할 때 부여하는 안정적인
정수값이다. AIS_Shape에 `SetOwner(new AIS_DimensionOwner(id))` 또는
사용자 정의 owner로 태깅한다.

---

## 문서 간 기하 정렬 (Diff용)

두 Doc을 비교할 때(V2 오버레이 뷰), 한 Doc에 `registration gp_Trsf`를
저장하여 참조 Doc의 좌표계로 변환한다.

```cpp
// 참조 Doc(id=0)에 대해 생성 Doc(id=1)을 정렬
gp_Trsf t = computeRegistration(doc0->rootShape(), doc1->rootShape());
doc1->setRegistration(t);

// AIS 표시 시 변환 적용
Handle(AIS_Shape) aisShape = ...;
aisShape->SetLocalTransformation(doc1->registration());
doc1->aisContext()->RecomputePrsOnly(aisShape, Standard_True);
```

정합 알고리즘: 초기 M2에서는 바운딩 박스 중심 정렬. M3+에서는
ICP (Iterative Closest Point) 또는 CGAL `cgal-shape-detection` 활용.

---

## Diff 오버레이 전략

Bottom-Left 뷰(V2)에서 두 shell의 대칭 차집합을 색상으로 표시한다.

```cpp
// Doc0 (참조), Doc1 (생성) 의 diff 오버레이 생성
BRepAlgoAPI_Cut cutter0(doc0->rootShape(), doc1->rootShape());
BRepAlgoAPI_Cut cutter1(doc1->rootShape(), doc0->rootShape());

TopoDS_Compound diffShape;
BRep_Builder builder;
builder.MakeCompound(diffShape);
builder.Add(diffShape, cutter0.Shape());  // 참조에만 있는 영역
builder.Add(diffShape, cutter1.Shape());  // 생성에만 있는 영역

Handle(AIS_Shape) diffAIS = new AIS_Shape(diffShape);
diffAIS->SetColor(Quantity_NOC_RED);
diffAIS->SetTransparency(0.4);
diffContext->Display(diffAIS, Standard_True);
```

Diff 전용 context는 V2 뷰에서만 사용하는 별도 `AIS_InteractiveContext`로
관리한다.

---

## OCAF 도입 시점 (졸업 기준)

| 조건 | 초기 (own Doc) | OCAF 도입 후 |
|---|---|---|
| Undo/Redo | Doc 단위 수동 스택 | TDF_Transaction 자동 |
| 문서 간 참조 | feature_id 정수 | XLink (TDF_Reference) |
| 영속성 | JSON + STEP/XCAF | XBF 바이너리 |
| 복잡도 | 낮음 | 중간~높음 |

**규칙**: M3 완료 후, (a) 문서 그래프 전체 Undo/Redo가 요구되거나,
(b) XLink 참조가 필요하거나, (c) XBF 영속성이 필요할 때 OCAF로 전환한다.
결정 테이블 전문은 [[architecture/document-model]] 참조.

---

## 관련 문서

- [[architecture/overview]] — 모듈 분해 및 레이어 구조
- [[architecture/multi-view]] — V3d_View 바인딩, featureSelected 신호 발원
- [[architecture/document-model]] — own Doc vs OCAF 전환 경로
- [[engine/parametric-templates]] — feature_id 부여 방식
- [[engine/reverse-route]] — RE Doc 생성 흐름
