# Architecture Overview

KooCADCAM은 시계/스마트폰 금속 전면 케이스를 대상으로 하는 **Windows + Linux
멀티플랫폼** C++17/Qt 6/OCCT 8.0 기반 설계 자동화 도구다. Windows는 Visual
Studio 2022 + MSVC, Linux는 CMake CLI + GCC/Clang으로 네이티브 빌드한다.
공통 빌드 시스템은 CMake. 이 문서는 시스템 전체의 레이어 구조와 모듈 분해를 기술한다.

---

## 시스템 블록 다이어그램

```
┌─────────────────────────────────────────────────────────────────────┐
│                          Qt UI Layer                                │
│                                                                     │
│  ┌───────────────┐  ┌──────────────────┐  ┌──────────────────────┐ │
│  │  MainWindow   │  │  ViewGrid (2×2)  │  │  DockWidgets         │ │
│  │               │  │                  │  │  - FeatureTree       │ │
│  │  - MenuBar    │  │  ┌────┬────┐     │  │  - ParameterPanel    │ │
│  │  - ToolBar    │  │  │ V0 │ V1 │     │  │  - DFMReportPanel    │ │
│  │  - StatusBar  │  │  ├────┼────┤     │  │  - MessageLog        │ │
│  │               │  │  │ V2 │ V3 │     │  └──────────────────────┘ │
│  └───────────────┘  │  └────┴────┘     │                           │
│                     └──────────────────┘                           │
└────────────────────────────┬────────────────────────────────────────┘
                             │  Qt signals / slots
┌────────────────────────────▼────────────────────────────────────────┐
│                         Engine Layer                                │
│                                                                     │
│  ┌─────────────┐  ┌──────────────────┐  ┌───────────────────────┐  │
│  │ DocManager  │  │ FeatureBuilders   │  │ DFMVerifier           │  │
│  │             │  │ - WatchTemplate   │  │ - WallThickness       │  │
│  │ - open()    │  │ - PhoneTemplate   │  │ - DraftAngle          │  │
│  │ - close()   │  │ - MorphBuilder    │  │ - RadiusCheck         │  │
│  │ - active()  │  │ - REPipeline      │  └───────────────────────┘  │
│  └─────────────┘  └──────────────────┘                             │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │ CAMVerifier (3-axis collision / tool-reach check)           │    │
│  └─────────────────────────────────────────────────────────────┘    │
└────────────────────────────┬────────────────────────────────────────┘
                             │  Handle_* / NCollection / BRep API
┌────────────────────────────▼────────────────────────────────────────┐
│                      OCCT 8.0.0 Kernel                              │
│                                                                     │
│  BRep / BRepAlgo  │  BRepMesh  │  AIS / V3d  │  XCAF / STEP/IGES  │
│  GeomLib / Geom2d │  TopExp    │  OpenGl_*   │  TDocStd (optional) │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 모듈 분해 (계획된 소스 경로, 현재 RESERVED)

### `src/gui/` — Qt UI 레이어

| 파일 | 역할 |
|---|---|
| [[src/gui/MainWindow.hpp]] | 최상위 QMainWindow, DocManager 보유 |
| [[src/gui/ViewGrid.hpp]] | 2×2 QOpenGLWidget 컨테이너, V3d_View 바인딩 |
| [[src/gui/ParameterPanel.hpp]] | 파라메트릭 입력 폼, 실시간 preview 요청 |
| [[src/gui/DFMReportPanel.hpp]] | DFM 검사 결과 트리, 위반 항목 하이라이트 |
| [[src/gui/FeatureTreeModel.hpp]] | Qt 모델/뷰, Doc feature graph 반영 |

### `src/engine/` — 비즈니스 로직

| 파일 | 역할 |
|---|---|
| [[src/engine/Doc.hpp]] | 문서 단위 (AIS context + feature graph + shape) |
| [[src/engine/DocManager.hpp]] | 다중 Doc 생명주기, active doc 추적 |
| [[src/engine/FeatureBuilder.hpp]] | 추상 기반 클래스, build() → TopoDS_Shape |
| [[src/engine/WatchTemplate.hpp]] | 시계 전면 케이스 파라메트릭 빌더 |
| [[src/engine/MorphBuilder.hpp]] | morphing 데모 (M2) |
| [[src/engine/DFMVerifier.hpp]] | BRep 분석 → DFMResult 목록 반환 |

### `src/io/` — 입출력

| 파일 | 역할 |
|---|---|
| [[src/io/StepExporter.hpp]] | XCAF 기반 STEP AP203/AP214 저장 |
| [[src/io/StlImporter.hpp]] | RE 파이프라인용 STL 읽기 |
| [[src/io/JsonSpecLoader.hpp]] | M1 파라미터 JSON 로드 |

### `src/re/` — Reverse Engineering 파이프라인

STL → feature recognition → STEP 변환 경로. [[engine/reverse-route]] 참조.

### `src/dfm/` — DFM 규칙 엔진

제조 가능성 검사 규칙 집합. [[engine/dfm-rules]] 참조.

### `src/cam/` — CAM 검증

3축 충돌·공구 도달 검사. [[engine/cam-3axis-verify]] 참조.

---

## 레이어링 근거 ("Why this layering")

**UI ↔ Engine 분리**: Qt UI는 OCCT 타입을 직접 보유하지 않는다. 신호는
`doc_id`/`feature_id`(정수) 또는 `QVariantMap`으로 전달되어 테스트 가능성을
높인다. [[process/coding-standards]] 참조.

**Engine ↔ OCCT 분리**: `FeatureBuilder` 추상 기반 클래스를 두면 OCCT
버전 업그레이드 또는 대안 커널 도입 시 영향 범위를 `src/engine/` 내부로
국한할 수 있다.

**다중 Feature Route**: 파라메트릭(primary), Morphing(demo), STL→STEP RE
세 경로는 동일한 `FeatureBuilder` 인터페이스를 구현한다. Doc은 어느 경로로
생성된 shape도 동일하게 표현한다. [[engine/parametric-templates]] 참조.

**Human-in-the-loop**: GUI는 자동 결과를 검토·수정하는 인터페이스다.
DFM 결과·RE 중간 결과·CAM 경고는 모두 Panel에서 사용자 승인을 요구한다.

**OCAF는 선택적**: `TDocStd_Document`는 M3 이후 필요 시 도입한다.
결정 기준은 [[architecture/document-model]] 참조.

---

## 관련 문서

- [[architecture/multi-view]] — OpenGl_GraphicDriver 공유, V3d_View 바인딩
- [[architecture/multi-document]] — AIS_InteractiveContext per Doc 정책
- [[architecture/document-model]] — own Doc vs OCAF 결정 테이블
- [[architecture/build-and-deps]] — vcpkg, CMake, MSVC 설정
- [[scope/milestones-and-krs]] — M1(Watch), M2(Morph), M3(RE) 일정
