// @lat: [[architecture/overview]]
// KooCADCAM application entry point.

#include "gui/MainWindow.hpp"

#include <QApplication>
#include <QSurfaceFormat>

#include <spdlog/spdlog.h>

#include <Standard_Version.hxx>

int main(int argc, char* argv[])
{
    // OpenGL 3.2 core profile + 24-bit depth + 8-bit stencil
    // ([[architecture/multi-view#1단계 - 공유 OpenGl_GraphicDriver 생성]])
    QSurfaceFormat fmt;
    fmt.setVersion(3, 2);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setDepthBufferSize(24);
    fmt.setStencilBufferSize(8);
    fmt.setSamples(4);  // optional AA
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication app(argc, argv);
    QCoreApplication::setApplicationName("KooCADCAM");
    QCoreApplication::setOrganizationName("KooCADCAM");
    QCoreApplication::setApplicationVersion("0.1.0-M0");

    spdlog::info("KooCADCAM starting (OCCT {}.{}.{})",
                 OCC_VERSION_MAJOR, OCC_VERSION_MINOR, OCC_VERSION_MAINTENANCE);

    koocadcam::gui::MainWindow w;
    w.show();
    return app.exec();
}
