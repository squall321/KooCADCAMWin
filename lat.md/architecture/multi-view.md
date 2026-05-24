# Multi-View

KooCADCAM은 2×2 뷰 그리드를 1급 기능으로 지원한다. OCCT 8.0의 Multi-View
패턴을 따르며, 단일 `OpenGl_GraphicDriver` + 단일 `V3d_Viewer` + 다수의
`V3d_View`(각각 독립 `QOpenGLWidget`에 바인딩)로 구성된다.

참고 구현: `github.com/gkv311/occt-samples-qopenglwidget`

---

## 초기화 시퀀스

다음 순서는 `MainWindow::initViewSystem()` 또는 `ViewGrid` 생성자에서
수행된다 (경로 RESERVED: [[src/gui/ViewGrid.hpp]]).

### 1단계 — 공유 OpenGl_GraphicDriver 생성

```cpp
// Windows: buffersNoSwap=true 필수 (각 QOpenGLWidget이 자체 swap 수행)
Handle(Aspect_DisplayConnection) aDisp;  // Windows에서는 nullptr 가능
Handle(OpenGl_GraphicDriver) aDriver =
    new OpenGl_GraphicDriver(aDisp, /*theToInitialize=*/false);

OpenGl_Caps& caps = aDriver->ChangeOptions();
caps.buffersNoSwap = true;   // ★ 핵심: 드라이버가 swap하지 않음
caps.contextDebug  = false;  // release 빌드에서는 false

aDriver->InitContext();      // OpenGL 컨텍스트 초기화
```

드라이버는 애플리케이션 생명주기 동안 단 하나만 존재하며,
`DocManager` 또는 별도 `ViewSystem` 싱글턴이 소유한다.

### 2단계 — V3d_Viewer 생성

```cpp
Handle(V3d_Viewer) aViewer = new V3d_Viewer(aDriver);
aViewer->SetDefaultLights();
aViewer->SetLightOn();
// 씬 당 하나의 Viewer. 다중 씬이 필요하면 Viewer를 추가한다.
```

### 3단계 — QOpenGLWidget 당 V3d_View 생성 및 바인딩

각 `KooViewWidget : public QOpenGLWidget`에서 수행한다
(RESERVED: [[src/gui/KooViewWidget.hpp]]).

```cpp
void KooViewWidget::initializeGL()
{
    // QOpenGLWidget의 네이티브 핸들을 Aspect_Window로 래핑
    Handle(Aspect_NeutralWindow) aWin = new Aspect_NeutralWindow();
    aWin->SetSize(width(), height());
    // Windows: WId를 HWND로 캐스팅
    aWin->SetNativeHandle(reinterpret_cast<Aspect_Drawable>(winId()));

    m_view = m_viewer->CreateView();
    m_view->SetWindow(aWin, /*theContext=*/nullptr);
    m_view->MustBeResized();
    m_view->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_WHITE, 0.08);
}

void KooViewWidget::paintGL()
{
    m_view->InvalidateImmediate();
    FlushViewEvents(m_aisContext, m_view, true);
}

void KooViewWidget::resizeGL(int w, int h)
{
    m_view->MustBeResized();
    m_view->Invalidate();
}
```

---

## 카메라 동기화 정책

### 비트마스크 기반 동기화

```
View0 (top-left)  — Master  (bitmask: 0b0001)
View1 (top-right) — Follower (bitmask: 0b0010)
View2 (bot-left)  — Follower (bitmask: 0b0100)
View3 (bot-right) — Follower (bitmask: 0b1000)
```

`ViewGrid`는 `cameraChanged(int viewIndex)` Qt 신호를 수신하면
`m_syncMask`에 설정된 Follower View들에 카메라를 복사한다.

```cpp
void ViewGrid::onCameraChanged(int masterIdx)
{
    Handle(Graphic3d_Camera) masterCam =
        m_views[masterIdx]->Camera();

    for (int i = 0; i < 4; ++i) {
        if (i != masterIdx && (m_syncMask & (1 << i))) {
            m_views[i]->SetCamera(masterCam->Copy());
            m_views[i]->Invalidate();
        }
    }
}
```

### 동기화 토글 UI

`ParameterPanel` 또는 ViewGrid 헤더 툴바의 체크박스로
`m_syncMask`를 런타임에 변경 가능하다. 예: "Sync Cameras" 토글.
이를 통해 독립 탐색 모드(diff 검토)와 동기 탐색 모드(발표)를 전환한다.

---

## 피킹(Picking) 흐름

```
사용자 마우스 이벤트
        │
        ▼
KooViewWidget::mouseMoveEvent(QMouseEvent* e)
        │
        ├─► m_aisContext->MoveTo(e->x(), e->y(), m_view, Standard_True)
        │        (하이라이트 갱신)
        │
KooViewWidget::mousePressEvent(QMouseEvent* e)   [Left Click]
        │
        ├─► m_aisContext->Select(Standard_True)
        │
        ├─► m_aisContext->InitSelected()
        │        while (m_aisContext->MoreSelected())
        │            Handle(AIS_Shape) shape = ...
        │            feature_id = shape->GetOwner()->...
        │
        └─► emit featureSelected(doc_id, feature_id)   ← Qt 신호
```

`featureSelected(int docId, int featureId)` 신호는 `DocManager`가
수신하여 해당 Doc의 `AIS_InteractiveContext`에서 선택 하이라이트를
갱신한다. 자세한 내용은 [[architecture/multi-document]] 참조.

---

## 2×2 뷰 그리드 기본 용례

| 위치 | 기본 콘텐츠 | 뷰 타입 |
|---|---|---|
| Top-Left (V0) | 참조 모델 (레퍼런스 또는 고객 STL) | 표준 음영 |
| Top-Right (V1) | 파라메트릭 생성 모델 (현재 결과) | 표준 음영 |
| Bottom-Left (V2) | 컬러 오버레이 차분 (V0 vs V1) | 색상 맵 |
| Bottom-Right (V3) | 단면 / 분해도 / CAM 공구 경로 | 와이어프레임 또는 투명 |

뷰 배치는 사용자가 드래그·재배치 가능하도록 설계한다 (M2+).

---

## 성능 고려 사항

### 공유 드라이버 vs 뷰별 드라이버

| 항목 | 공유 드라이버 (채택) | 뷰별 드라이버 |
|---|---|---|
| GPU 메모리 | 셰이더/버퍼 공유 → 절약 | 각 드라이버가 독립 로드 |
| 초기화 복잡도 | `buffersNoSwap=true` 필요 | 단순하지만 중복 비용 |
| 동기화 | Viewer 레벨에서 자연스럽게 처리 | 수동 동기 필요 |
| OCCT 권고 | Multi-view 공식 패턴 | 단일 뷰 용도에 적합 |

`buffersNoSwap=true`로 설정하면 드라이버가 `SwapBuffers`를 호출하지 않아
각 `QOpenGLWidget`의 `paintGL()`이 Qt의 swap을 직접 제어한다. 이를 빠뜨리면
첫 번째 뷰 외에는 화면이 갱신되지 않는다.

---

## 관련 문서

- [[architecture/overview]] — 레이어 구조 및 모듈 분해
- [[architecture/multi-document]] — AIS_InteractiveContext per Doc 정책
- [[architecture/build-and-deps]] — Qt 6 OpenGL 링크 플래그
- [[engine/parametric-templates]] — 뷰에 표시할 형상 생성
