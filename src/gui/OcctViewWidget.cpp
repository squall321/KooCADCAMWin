// @lat: [[architecture/multi-view#3단계 — QOpenGLWidget 당 V3d_View 생성 및 바인딩]]
//
// OCCT 8.0 + QOpenGLWidget integration uses the gkv311 OcctGlTools pattern
// (MIT licensed, see licenses/OcctGlTools_MIT.txt). Core idea: per-paintGL
// FBO wrapping so OCCT renders into Qt's QOpenGLFramebufferObject instead of
// the default (0) framebuffer (which Qt does not display).

#ifdef _WIN32
  #include <windows.h>
#endif
#include <OpenGl_Context.hxx>

#include "OcctViewWidget.hpp"

#include "occt/OcctGlTools.h"

#include <AIS_Shape.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <Quantity_Color.hxx>

#include <QMouseEvent>
#include <QWheelEvent>

#include <spdlog/spdlog.h>

namespace koocadcam::gui {

OcctViewWidget::OcctViewWidget(QWidget* parent)
    : QOpenGLWidget(parent)
{
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
    setAttribute(Qt::WA_AcceptTouchEvents);
    setBackgroundRole(QPalette::NoRole);
    setUpdatesEnabled(true);
    setUpdateBehavior(QOpenGLWidget::NoPartialUpdate);

    // ── OpenGl_GraphicDriver — create up front, no native context yet ──────
    Handle(Aspect_DisplayConnection) aDisp;
    m_driver = new OpenGl_GraphicDriver(aDisp, false);
    OpenGl_Caps& caps = m_driver->ChangeOptions();
    caps.buffersNoSwap = true;       // Qt swaps buffers, not OCCT
    caps.buffersOpaqueAlpha = true;  // do not write into alpha channel
    caps.useSystemBuffer = false;    // force offscreen FBO (key for QOpenGLWidget)
    caps.contextDebug = false;

    // ── V3d_Viewer ─────────────────────────────────────────────────────────
    m_viewer = new V3d_Viewer(m_driver);
    m_viewer->SetDefaultBackgroundColor(Quantity_NOC_GRAY30);
    m_viewer->SetDefaultLights();
    m_viewer->SetLightOn();

    // ── V3d_View — window/context attached lazily in initializeGL ──────────
    m_view = m_viewer->CreateView();
    m_view->SetImmediateUpdate(false);
    m_view->ChangeRenderingParams().NbMsaaSamples = 4;

    // ── AIS context ────────────────────────────────────────────────────────
    m_aisContext = new AIS_InteractiveContext(m_viewer);
    m_aisContext->SetDisplayMode(AIS_Shaded, false);
}

OcctViewWidget::~OcctViewWidget()
{
    // Hold display connection until another GL context is made current to
    // avoid sudden crash in QOpenGLWidget destructor on X11.
    Handle(Aspect_DisplayConnection) aDisp = m_viewer->Driver()->GetDisplayConnection();
    if (!m_aisContext.IsNull())
        m_aisContext->RemoveAll(false);
    m_aisContext.Nullify();
    if (!m_view.IsNull())
        m_view->Remove();
    m_view.Nullify();
    m_viewer.Nullify();
    makeCurrent();
    aDisp.Nullify();
}

void OcctViewWidget::initializeGL()
{
    const Aspect_Drawable nativeWin = (Aspect_Drawable)effectiveWinId();
    const NCollection_Vec2<int> viewSize(rect().right() - rect().left(),
                                         rect().bottom() - rect().top());

    if (!OcctGlTools::InitializeGlWindow(m_view, nativeWin, viewSize, devicePixelRatioF()))
    {
        spdlog::error("[OcctView] OcctGlTools::InitializeGlWindow FAILED");
        return;
    }
    makeCurrent();
    m_view->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_WHITE, 0.08);
    spdlog::debug("[OcctView] initializeGL ok ({}x{} @ {})",
                  width(), height(), devicePixelRatioF());
}

void OcctViewWidget::paintGL()
{
    if (m_view.IsNull() || m_view->Window().IsNull())
        return;

    // Re-init GL window if Qt recreated the native handle (DPI / monitor change).
    if (m_view->Window()->NativeHandle()
        != OcctGlTools::GetGlNativeWindow((Aspect_Drawable)effectiveWinId()))
    {
        spdlog::warn("[OcctView] native window changed, re-initialising GL");
        initializeGL();
    }

    // Per-paint: wrap Qt's current QOpenGLFramebufferObject so OCCT renders into it.
    if (!OcctGlTools::InitializeGlFbo(m_view))
    {
        spdlog::error("[OcctView] OcctGlTools::InitializeGlFbo FAILED");
        return;
    }

    OcctGlTools::ResetGlStateBeforeOcct(m_view);

    m_view->InvalidateImmediate();
    m_view->Redraw();

    OcctGlTools::ResetGlStateAfterOcct(m_view);
}

void OcctViewWidget::resizeGL(int w, int h)
{
    if (m_view.IsNull() || m_view->Window().IsNull())
        return;
    Handle(Aspect_NeutralWindow) win =
        Handle(Aspect_NeutralWindow)::DownCast(m_view->Window());
    if (!win.IsNull())
        win->SetSize(w, h);
    m_view->MustBeResized();
    m_view->Invalidate();
}

void OcctViewWidget::setShape(const TopoDS_Shape& shape)
{
    if (m_aisContext.IsNull() || shape.IsNull())
        return;
    try
    {
        m_aisContext->RemoveAll(false);
        Handle(AIS_Shape) ais = new AIS_Shape(shape);
        m_aisContext->Display(ais, AIS_Shaded, 0, false);
        m_currentShape = shape;
        m_view->FitAll(0.05, true);
        update();
    }
    catch (const std::exception& e)
    {
        spdlog::error("OcctViewWidget::setShape — {}", e.what());
    }
}

void OcctViewWidget::clearShape()
{
    if (m_aisContext.IsNull())
        return;
    m_aisContext->RemoveAll(false);
    m_currentShape = TopoDS_Shape();
    update();
}

void OcctViewWidget::fitAll()
{
    if (!m_view.IsNull())
    {
        m_view->FitAll(0.05);
        update();
    }
}

void OcctViewWidget::mousePressEvent(QMouseEvent* e)
{
    m_lastX = e->pos().x();
    m_lastY = e->pos().y();
    if (e->button() == Qt::LeftButton)
    {
        m_drag = DragMode::Rotate;
        if (!m_view.IsNull())
            m_view->StartRotation(m_lastX, m_lastY);
    }
    else if (e->button() == Qt::RightButton)
    {
        m_drag = DragMode::Pan;
    }
}

void OcctViewWidget::mouseMoveEvent(QMouseEvent* e)
{
    if (m_view.IsNull())
        return;
    const int x = e->pos().x();
    const int y = e->pos().y();
    if (m_drag == DragMode::Rotate)
    {
        m_view->Rotation(x, y);
        update();
    }
    else if (m_drag == DragMode::Pan)
    {
        m_view->Pan(x - m_lastX, m_lastY - y);
        m_lastX = x;
        m_lastY = y;
        update();
    }
}

void OcctViewWidget::mouseReleaseEvent(QMouseEvent* /*e*/)
{
    m_drag = DragMode::None;
}

void OcctViewWidget::wheelEvent(QWheelEvent* e)
{
    if (m_view.IsNull())
        return;
    const int delta = e->angleDelta().y();
    m_view->SetZoom(delta > 0 ? 1.1 : 0.9);
    update();
}

}  // namespace koocadcam::gui
