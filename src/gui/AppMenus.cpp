// @lat: [[architecture/overview#src/gui/]]

#include "AppMenus.hpp"
#include "MainWindow.hpp"
#include "OcctViewWidget.hpp"

#include <engine/PlaceholderCylinder.hpp>
#include <io/StepIO.hpp>

#include <QApplication>
#include <QFileDialog>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>

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
    TopoDS_Shape shape = koocadcam::io::StepIO::read(path.toStdString(), err);
    if (shape.IsNull())
    {
        QMessageBox::critical(m_window, tr("Open STEP"),
                              tr("Failed to read STEP file:\n%1").arg(QString::fromStdString(err)));
        return;
    }

    m_window->viewWidget()->setShape(shape);
}

void AppMenus::onFitAll()
{
    m_window->viewWidget()->fitAll();
}

void AppMenus::onExit()
{
    qApp->quit();
}

}  // namespace koocadcam::gui
