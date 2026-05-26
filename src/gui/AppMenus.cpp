// @lat: [[architecture/overview#src/gui/]]

#include "AppMenus.hpp"
#include "MainWindow.hpp"
#include "OcctViewWidget.hpp"
#include "ParameterPanel.hpp"

#include <engine/PhoneFrontModel.hpp>
#include <engine/PlaceholderCylinder.hpp>
#include <engine/WatchFrontModel.hpp>
#include <io/StepIO.hpp>
#include <io/JsonSpec.hpp>

#include <TopoDS_Shape.hxx>

#include <QApplication>
#include <QFileDialog>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>

#include <nlohmann/json.hpp>

namespace koocadcam::gui {

AppMenus::AppMenus(MainWindow* parent)
    : QObject(parent)
    , m_window(parent)
{
}

void AppMenus::install(QMenuBar* menuBar)
{
    // ── File menu ────────────────────────────────────────────────────────────
    QMenu* fileMenu = menuBar->addMenu(tr("&File"));

    QAction* actNew = fileMenu->addAction(tr("New Sample &Cylinder"));
    actNew->setShortcut(QKeySequence("Ctrl+N"));
    connect(actNew, &QAction::triggered, this, &AppMenus::onNewSampleCylinder);

    QAction* actSave = fileMenu->addAction(tr("&Save STEP..."));
    actSave->setShortcut(QKeySequence("Ctrl+S"));
    connect(actSave, &QAction::triggered, this, &AppMenus::onSaveStep);

    QAction* actOpen = fileMenu->addAction(tr("&Open STEP..."));
    actOpen->setShortcut(QKeySequence("Ctrl+O"));
    connect(actOpen, &QAction::triggered, this, &AppMenus::onOpenStep);

    fileMenu->addSeparator();

    QAction* aNewWatch = new QAction(tr("New Watch (Round)"), this);
    aNewWatch->setShortcut(QKeySequence("Ctrl+W"));
    connect(aNewWatch, &QAction::triggered, this, &AppMenus::onNewWatchRound);
    fileMenu->addAction(aNewWatch);

    QAction* aOpenWatchSpec = new QAction(tr("Open Watch Spec…"), this);
    aOpenWatchSpec->setShortcut(QKeySequence("Ctrl+Shift+O"));
    connect(aOpenWatchSpec, &QAction::triggered, this, &AppMenus::onOpenWatchSpec);
    fileMenu->addAction(aOpenWatchSpec);

    QAction* aSaveWatchSpec = new QAction(tr("Save Watch Spec…"), this);
    aSaveWatchSpec->setShortcut(QKeySequence("Ctrl+Shift+S"));
    connect(aSaveWatchSpec, &QAction::triggered, this, &AppMenus::onSaveWatchSpec);
    fileMenu->addAction(aSaveWatchSpec);

    fileMenu->addSeparator();

    QAction* aNewPhoneRect = new QAction(tr("New Phone (Rectangular)"), this);
    aNewPhoneRect->setShortcut(QKeySequence("Ctrl+Shift+P"));
    connect(aNewPhoneRect, &QAction::triggered, this, &AppMenus::onNewPhoneRect);
    fileMenu->addAction(aNewPhoneRect);

    fileMenu->addSeparator();

    QAction* actExit = fileMenu->addAction(tr("E&xit"));
    actExit->setShortcut(QKeySequence("Ctrl+Q"));
    connect(actExit, &QAction::triggered, this, &AppMenus::onExit);

    // ── View menu ────────────────────────────────────────────────────────────
    QMenu* viewMenu = menuBar->addMenu(tr("&View"));

    QAction* actFit = viewMenu->addAction(tr("&Fit All"));
    actFit->setShortcut(QKeySequence("F"));
    connect(actFit, &QAction::triggered, this, &AppMenus::onFitAll);
}

void AppMenus::onNewSampleCylinder()
{
    TopoDS_Shape shape = koocadcam::engine::PlaceholderCylinder::make();
    m_window->viewWidget()->setShape(shape);
}

void AppMenus::onSaveStep()
{
    const QString path = QFileDialog::getSaveFileName(
        m_window,
        tr("Save STEP File"),
        QString(),
        tr("STEP files (*.step *.stp)"));

    if (path.isEmpty())
        return;

    const TopoDS_Shape shape = m_window->viewWidget()->currentShape();
    if (shape.IsNull())
    {
        QMessageBox::warning(m_window, tr("Save STEP"), tr("No shape to save."));
        return;
    }

    std::string err;
    if (!koocadcam::io::StepIO::write(shape, path.toStdString(), err))
    {
        QMessageBox::critical(m_window, tr("Save STEP"),
                              tr("Failed to write STEP file:\n%1").arg(QString::fromStdString(err)));
    }
}

void AppMenus::onOpenStep()
{
    const QString path = QFileDialog::getOpenFileName(
        m_window,
        tr("Open STEP File"),
        QString(),
        tr("STEP files (*.step *.stp)"));

    if (path.isEmpty())
        return;

    std::string err;
    auto shapeOpt = koocadcam::io::StepIO::read(path.toStdString(), err);
    if (!shapeOpt || shapeOpt->IsNull())
    {
        QMessageBox::critical(m_window, tr("Open STEP"),
                              tr("Failed to read STEP file:\n%1").arg(QString::fromStdString(err)));
        return;
    }

    m_window->viewWidget()->setShape(*shapeOpt);
}

void AppMenus::onFitAll()
{
    m_window->viewWidget()->fitAll();
}

void AppMenus::onExit()
{
    qApp->quit();
}

void AppMenus::onNewWatchRound()
{
    nlohmann::json defaultSpec = {
        {"schema_version", "1.0.0"},
        {"product_name",   "Default Round Watch 44"},
        {"form_factor",    "round"},
        {"base",           {{"diameter_mm", 44.0}, {"thickness_mm", 10.0}}},
        {"corner_radius",  {{"r_top_mm", 1.0}, {"r_side_mm", 0.6}}},
        {"bezel",          {{"width_mm", 3.0}, {"depth_mm", 1.0}, {"taper_deg", 0.0}}}
    };
    m_window->parameterPanel()->setSpec(defaultSpec);
    m_window->parameterPanel()->triggerRebuild();
}

void AppMenus::onOpenWatchSpec()
{
    const QString path = QFileDialog::getOpenFileName(
        m_window,
        tr("Open Watch Spec"),
        QString(),
        tr("Watch Spec (*.json)"));

    if (path.isEmpty())
        return;

    std::string err;
    auto specOpt = koocadcam::io::JsonSpec::read(path.toStdString(), err);
    if (!specOpt) {
        QMessageBox::critical(m_window, tr("Open Watch Spec"),
                              tr("Failed to read spec:\n%1").arg(QString::fromStdString(err)));
        return;
    }

    std::vector<std::string> validationErrors;
    koocadcam::io::JsonSpec::validateWatchSpec(*specOpt, validationErrors);
    if (!validationErrors.empty()) {
        QMessageBox::warning(
            m_window,
            tr("Open Watch Spec"),
            tr("Validation warning (loading anyway):\n%1")
                .arg(QString::fromStdString(validationErrors.front())));
    }

    m_window->parameterPanel()->setSpec(*specOpt);
    m_window->parameterPanel()->triggerRebuild();
}

void AppMenus::onNewPhoneRect()
{
    // M2-phase-1 visual smoke: build the default phone via PhoneFrontModel and
    // push the shape directly to the viewer.  ParameterPanel still drives the
    // watch spec (phone widget set lands in M2-phase-2), so this menu is a
    // one-shot view-only render.
    nlohmann::json spec = koocadcam::engine::PhoneFrontModel::defaultSpec();
    std::vector<koocadcam::engine::BuildWarning> warnings;
    TopoDS_Shape shape = koocadcam::engine::PhoneFrontModel::buildAll(spec, warnings);
    if (shape.IsNull()) {
        QMessageBox::critical(
            m_window, tr("New Phone"),
            tr("Failed to build phone shape (%1 warning(s)).").arg(warnings.size()));
        return;
    }
    m_window->viewWidget()->setShape(shape);
}

void AppMenus::onSaveWatchSpec()
{
    const QString path = QFileDialog::getSaveFileName(
        m_window,
        tr("Save Watch Spec"),
        QString(),
        tr("Watch Spec (*.json)"));

    if (path.isEmpty())
        return;

    nlohmann::json spec = m_window->parameterPanel()->currentSpec();
    std::string err;
    if (!koocadcam::io::JsonSpec::write(spec, path.toStdString(), err)) {
        QMessageBox::critical(m_window, tr("Save Watch Spec"),
                              tr("Failed to write spec:\n%1").arg(QString::fromStdString(err)));
    }
}

}  // namespace koocadcam::gui
