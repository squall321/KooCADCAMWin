// @lat: [[architecture/multi-view#3단계 — QOpenGLWidget 당 V3d_View 생성 및 바인딩]]

#include "OcctViewWidget.hpp"

#include <AIS_Shape.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <OpenGl_Caps.hxx>
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
}

OcctViewWidget::~OcctViewWidget() = default;

void OcctViewWidget::initializeGL()
{
    // Stage 1: Create OpenGl_GraphicDriver with external (Qt-owned) GL context.
    // Standard_False = do NOT create its own GL context; Qt's QOpenGLWidget owns it.
    m_driver = new OpenGl_GraphicDriver(Handle(Aspect_DisplayConnection)(), Standard_False);

    // Stage 2: Tune caps — disable driver-side buffer swap (Qt handles it) and
    // suppress debug output for release builds.
    OpenGl_Caps& caps = m_driver->ChangeOptions();
    caps.buffersNoSwap = true;
    caps.contextDebug  = false;

    m_driver->InitContext();

    // Stage 2: Create V3d_Viewer and enable default lights.
    m_viewer = new V3d_Viewer(m_driver);
    m_viewer->SetDefaultLights();
    m_viewer->SetLightOn();

    // Stage 3: Wrap the native window handle in Aspect_NeutralWindow.
    m_window = new Aspect_NeutralWindow();
    m_window->SetSize(width(), height());
    m_window->SetNativeHandle(
        reinterpret_cast<Aspect_Drawable>(static_cast<intptr_t>(winId())));

    // Stage 3: Create V3d_View and bind to the neutral window.
    m_view = m_viewer->CreateView();
    m_view->SetWindow(m_window, nullptr);
    m_view->MustBeResized();
    m_view->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_WHITE, 0.08);

    // Set a neutral dark-grey background.
    m_view->SetBackgroundColor(Quantity_NOC_GRAY30);

    // Create AIS interactive context attached to this viewer.
    m_aisContext = new AIS_InteractiveContext(m_viewer);
    m_aisContext->SetDisplayMode(AIS_Shaded, Standard_False);
}

void OcctViewWidget::paintGL()
{
    if (m_view.IsNull())
        return;
    m_view->Redraw();
}

void OcctViewWidget::resizeGL(int w, int h)
{
    if (!m_window.IsNull())
        m_window->SetSize(w, h);
    if (!m_view.IsNull())
    {
        m_view->MustBeResized();
        m_view->Invalidate();
    }
}

void OcctViewWidget::setShape(const TopoDS_Shape& shape)
{
    if (m_aisContext.IsNull())
        return;
    try
    {
        m_aisContext->RemoveAll(Standard_False);
        Handle(AIS_Shape) ais = new AIS_Shape(shape);
        m_aisContext->Display(ais, Standard_False);
        m_currentShape = shape;
        m_view->FitAll(0.05, Standard_False);
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
    m_aisContext->RemoveAll(Standard_False);
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

// @lat: [[architecture/multi-view#피킹(Picking) 흐름]]
// Picking implementation is reserved for a later milestone.
// The mouse handlers below cover rotate / pan / zoom navigation only.

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
    m_view->SetZoom(static_cast<Standard_Real>(delta > 0 ? 1.1 : 0.9));
    update();
}

}  // namespace koocadcam::gui
