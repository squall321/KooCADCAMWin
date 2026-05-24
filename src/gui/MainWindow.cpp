// @lat: [[architecture/overview#src/gui/]]

#include "MainWindow.hpp"
#include "AppMenus.hpp"
#include "OcctViewWidget.hpp"
#include "ParameterPanel.hpp"

#include <engine/WatchFrontModel.hpp>

#include <QDockWidget>
#include <QMenuBar>
#include <QStatusBar>

#include <nlohmann/json.hpp>

namespace koocadcam::gui {

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("KooCADCAM — M1.1");
    resize(1280, 800);

    m_view = new OcctViewWidget(this);
    setCentralWidget(m_view);

    m_menus = new AppMenus(this);
    m_menus->install(menuBar());

    // ── Parameter panel dock ─────────────────────────────────────────────────
    m_paramPanel = new ParameterPanel(this);
    QDockWidget* dock = new QDockWidget(tr("Parameters"), this);
    dock->setWidget(m_paramPanel);
    dock->setAllowedAreas(Qt::RightDockWidgetArea | Qt::LeftDockWidgetArea);
    addDockWidget(Qt::RightDockWidgetArea, dock);

    connect(m_paramPanel, &ParameterPanel::specChanged,
            this, &MainWindow::onSpecChanged);

    statusBar()->showMessage("Ready", 0);

    // Trigger initial build with default spec.
    onSpecChanged();
}

MainWindow::~MainWindow() = default;

void MainWindow::onSpecChanged()
{
    using koocadcam::engine::WatchFrontModel;
    using koocadcam::engine::BuildWarning;
    try {
        nlohmann::json spec = m_paramPanel->currentSpec();
        std::vector<BuildWarning> warnings;
        TopoDS_Shape shape = WatchFrontModel::buildAll(spec, warnings);
        if (shape.IsNull()) {
            QString msg = "Build failed";
            if (!warnings.empty()) {
                msg += QString::fromStdString(": " + warnings.front().message);
            }
            statusBar()->showMessage(msg, 5000);
            return;
        }
        m_view->setShape(shape);
        QString status = "Watch rebuilt";
        if (!warnings.empty()) {
            status += QString(" (%1 warnings)").arg(static_cast<int>(warnings.size()));
        }
        statusBar()->showMessage(status, 3000);
    } catch (const std::exception& e) {
        statusBar()->showMessage(
            QString("Build exception: %1").arg(e.what()), 8000);
    }
}

}  // namespace koocadcam::gui
