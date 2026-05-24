// @lat: [[architecture/overview#src/gui/]]

#include "MainWindow.hpp"
#include "AppMenus.hpp"
#include "OcctViewWidget.hpp"

#include <QMenuBar>
#include <QStatusBar>

namespace koocadcam::gui {

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("KooCADCAM — M0");
    resize(1280, 800);

    m_view = new OcctViewWidget(this);
    setCentralWidget(m_view);

    m_menus = new AppMenus(this);
    m_menus->install(menuBar());

    statusBar()->showMessage("Ready", 0);
}

MainWindow::~MainWindow() = default;

}  // namespace koocadcam::gui
