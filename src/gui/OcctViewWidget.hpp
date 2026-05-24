#pragma once
// @lat: [[architecture/multi-view#3단계 — QOpenGLWidget 당 V3d_View 생성 및 바인딩]]

#include <QOpenGLWidget>

#include <AIS_InteractiveContext.hxx>
#include <Aspect_NeutralWindow.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <TopoDS_Shape.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>

namespace koocadcam::gui {

class OcctViewWidget : public QOpenGLWidget
{
    Q_OBJECT
public:
    explicit OcctViewWidget(QWidget* parent = nullptr);
    ~OcctViewWidget() override;

    void setShape(const TopoDS_Shape& shape);
    void clearShape();
    [[nodiscard]] TopoDS_Shape currentShape() const { return m_currentShape; }
    void fitAll();

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;

    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;

private:
    Handle(OpenGl_GraphicDriver)   m_driver;
    Handle(V3d_Viewer)             m_viewer;
    Handle(V3d_View)               m_view;
    Handle(AIS_InteractiveContext) m_aisContext;
    Handle(Aspect_NeutralWindow)   m_window;

    TopoDS_Shape m_currentShape;
    int m_lastX{0};
    int m_lastY{0};
    enum class DragMode { None, Rotate, Pan } m_drag{DragMode::None};
};

}  // namespace koocadcam::gui
