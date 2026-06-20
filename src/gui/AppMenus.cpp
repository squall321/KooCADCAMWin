// @lat: [[architecture/overview#src/gui/]]

#include "AppMenus.hpp"
#include "MainWindow.hpp"
#include "PhonePanel.hpp"
#include "OcctViewWidget.hpp"
#include "ParameterPanel.hpp"
#include "ProcessPlanEditor.hpp"

#include <adapt/EditOp.hpp>
#include <adapt/NaturalLanguageStub.hpp>
#include <adapt/PlanEditor.hpp>
#include <engine/PhoneFrontModel.hpp>
#include <engine/PlaceholderCylinder.hpp>
#include <engine/WatchFrontModel.hpp>
#include <io/StepIO.hpp>
#include <io/JsonSpec.hpp>
#include <cam/ToolpathPlan.hpp>
#include <process/Executor.hpp>
#include <process/ProcessPlan.hpp>
#include <process/StepInvocation.hpp>
#include <re/Recognizer.hpp>
#include <skills/Stock.hpp>
#include <skills/Workpiece.hpp>

#include <fstream>
#include <skills/Skill.hpp>
#include <skills/Workpiece.hpp>

#include <TopoDS_Shape.hxx>

#include <QApplication>
#include <QDockWidget>
#include <QFileDialog>
#include <QInputDialog>
#include <QLineEdit>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QStatusBar>

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

    QAction* actGcode = fileMenu->addAction(tr("Export &G-code..."));
    connect(actGcode, &QAction::triggered, this, &AppMenus::onExportGCode);

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

    QAction* aRenderFullWatch = new QAction(tr("Render Full Demo Watch (all 10 steps)"), this);
    aRenderFullWatch->setShortcut(QKeySequence("Ctrl+Shift+D"));
    connect(aRenderFullWatch, &QAction::triggered, this, &AppMenus::onRenderFullDemoWatch);
    fileMenu->addAction(aRenderFullWatch);

    fileMenu->addSeparator();

    QAction* actExit = fileMenu->addAction(tr("E&xit"));
    actExit->setShortcut(QKeySequence("Ctrl+Q"));
    connect(actExit, &QAction::triggered, this, &AppMenus::onExit);

    // ── View menu ────────────────────────────────────────────────────────────
    QMenu* viewMenu = menuBar->addMenu(tr("&View"));

    QAction* actFit = viewMenu->addAction(tr("&Fit All"));
    actFit->setShortcut(QKeySequence("F"));
    connect(actFit, &QAction::triggered, this, &AppMenus::onFitAll);

    // ── Plan menu (Layer 3 / 4 / 5 GUI) ──────────────────────────────────────
    QMenu* planMenu = menuBar->addMenu(tr("&Plan"));

    QAction* aShowEditor = new QAction(tr("Show Process Plan Editor"), this);
    aShowEditor->setShortcut(QKeySequence("Ctrl+Shift+L"));
    aShowEditor->setCheckable(true);
    connect(aShowEditor, &QAction::triggered,
            this, &AppMenus::onShowProcessPlanEditor);
    planMenu->addAction(aShowEditor);
    // Keep the toggle in sync with the dock's visibility (e.g. when the user
    // closes the dock from its title bar X).
    if (auto* dock = m_window->processPlanDock()) {
        aShowEditor->setChecked(dock->isVisible());
        connect(dock, &QDockWidget::visibilityChanged, aShowEditor,
                [aShowEditor](bool visible) { aShowEditor->setChecked(visible); });
    }

    QAction* aRecognize = new QAction(tr("Recognize Current Shape"), this);
    aRecognize->setShortcut(QKeySequence("Ctrl+R"));
    connect(aRecognize, &QAction::triggered,
            this, &AppMenus::onRecognizeCurrentShape);
    planMenu->addAction(aRecognize);

    QAction* aExecutePlan = new QAction(tr("Execute Plan from Editor"), this);
    aExecutePlan->setShortcut(QKeySequence("Ctrl+E"));
    connect(aExecutePlan, &QAction::triggered,
            this, &AppMenus::onExecutePlanFromEditor);
    planMenu->addAction(aExecutePlan);

    QAction* aNlEdit = new QAction(tr("NL → EditOp (try stub)…"), this);
    connect(aNlEdit, &QAction::triggered, this, &AppMenus::onNlToEditOpStub);
    planMenu->addAction(aNlEdit);
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

void AppMenus::onExportGCode()
{
    using namespace koocadcam;
    const TopoDS_Shape shape = m_window->viewWidget()->currentShape();
    if (shape.IsNull()) {
        QMessageBox::warning(m_window, tr("Export G-code"), tr("No shape to machine."));
        return;
    }
    const QString path = QFileDialog::getSaveFileName(
        m_window, tr("Export G-code"), QString(), tr("G-code (*.nc *.gcode *.tap)"));
    if (path.isEmpty())
        return;

    try {
        // Recognise the part's machining features, replay them onto a sized
        // stock to get a workpiece carrying those features, then plan + verify
        // the toolpaths and emit one G-code program.
        skill::Workpiece wp(shape);
        process::ProcessPlan plan = re::inferProcessPlan(wp, 0.7);
        double x0, y0, z0, x1, y1, z1;
        wp.boundingBox(x0, y0, z0, x1, y1, z1);
        auto stock = skill::createCuboidStock(x1 - x0, y1 - y0, z1 - z0);
        const auto result = process::Executor::execute(plan, stock);
        const auto& machined = result.workpiece ? *result.workpiece : wp;

        const cam::ToolpathReport report = cam::planAndVerify(machined);
        if (report.toolpaths.empty()) {
            m_window->statusBar()->showMessage(
                tr("No machinable features recognised — nothing to export."), 4000);
            return;
        }
        std::ofstream f(path.toStdString());
        f << cam::toGCodeProgram(report);
        m_window->statusBar()->showMessage(
            tr("Exported %1 toolpath(s) — %2")
                .arg(static_cast<int>(report.toolpaths.size()))
                .arg(report.ok ? tr("collision-free") : tr("WITH collisions")),
            5000);
    } catch (const std::exception& e) {
        QMessageBox::critical(m_window, tr("Export G-code"),
                              tr("G-code export failed:\n%1").arg(QString::fromUtf8(e.what())));
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

void AppMenus::onRenderFullDemoWatch()
{
    // Bypasses ParameterPanel — that widget set only covers M1.2 fields, so
    // it would silently strip speaker_grille / rear_sensors / lugs /
    // secondary_fillets keys on a rebuild.  Render directly via buildAll
    // against the canonical 10-step defaultSpec.  Phone widget set + watch
    // M1.5 widget expansion are M2-phase-2 work.
    nlohmann::json spec = koocadcam::engine::WatchFrontModel::defaultSpec();
    std::vector<koocadcam::engine::BuildWarning> warnings;
    TopoDS_Shape shape = koocadcam::engine::WatchFrontModel::buildAll(spec, warnings);
    if (shape.IsNull()) {
        QMessageBox::critical(
            m_window, tr("Render Full Demo Watch"),
            tr("buildAll failed (%1 warning(s)).").arg(warnings.size()));
        return;
    }
    auto report = koocadcam::engine::WatchFrontModel::runDFM(shape, spec);
    QString msg = tr("Rendered full watch.  DFM: %1, %2 finding(s).")
        .arg(report.passed ? tr("PASS") : tr("FAIL"))
        .arg(report.findings.size());
    m_window->statusBar()->showMessage(msg, 5000);
    m_window->viewWidget()->setShape(shape);
}

void AppMenus::onNewPhoneRect()
{
    // B6.4: load the default phone into the PhonePanel and switch the live
    // build loop to phone mode — the phone is now a fully editable, DFM-gated
    // product, not a one-shot view-only render.
    m_window->phonePanel()->setSpec(koocadcam::engine::PhoneFrontModel::defaultSpec());
    m_window->activatePhone();
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

// ─────────────────────────────────────────────────────────────────────────────
// Layer 3 / 4 / 5 menu slots
// ─────────────────────────────────────────────────────────────────────────────

void AppMenus::onShowProcessPlanEditor()
{
    QDockWidget* dock = m_window->processPlanDock();
    if (!dock) return;
    const bool show = !dock->isVisible();
    dock->setVisible(show);
    if (show) dock->raise();
}

void AppMenus::onRecognizeCurrentShape()
{
    const TopoDS_Shape shape = m_window->viewWidget()->currentShape();
    if (shape.IsNull()) {
        QMessageBox::information(m_window, tr("Recognize Current Shape"),
                                 tr("Viewer has no shape — build or open something first."));
        return;
    }

    try {
        skill::Workpiece wp(shape);

        // Run full pipeline: analyze → drop low-confidence → dedupe.  The
        // candidate-count message uses the full analyze() result so the user
        // sees the unfiltered scan size; the plan uses the inferred (filtered
        // + ordered) plan instead.
        auto candidates = re::analyze(wp);
        auto plan       = re::inferProcessPlan(wp, /* min_confidence */ 0.7);

        if (auto* editor = m_window->processPlanEditor()) {
            editor->setPlan(plan);
        }
        if (auto* dock = m_window->processPlanDock()) {
            dock->show();
            dock->raise();
        }

        QMessageBox::information(
            m_window,
            tr("Recognize Current Shape"),
            tr("Recognized %1 candidate feature(s); inferred %2 plan step(s).")
                .arg(static_cast<int>(candidates.size()))
                .arg(plan.size()));
    } catch (const skill::SkillError& e) {
        QMessageBox::warning(m_window, tr("Recognize Current Shape"),
                             tr("SkillError: %1").arg(QString::fromUtf8(e.what())));
    } catch (const std::exception& e) {
        QMessageBox::critical(m_window, tr("Recognize Current Shape"),
                              tr("Exception: %1").arg(QString::fromUtf8(e.what())));
    }
}

void AppMenus::onExecutePlanFromEditor()
{
    auto* editor = m_window->processPlanEditor();
    if (!editor) return;
    process::ProcessPlan plan = editor->currentPlan();
    if (plan.empty()) {
        QMessageBox::information(m_window, tr("Execute Plan"),
                                 tr("Plan editor is empty.  Add a step or run "
                                    "Recognize Current Shape first."));
        return;
    }
    // Delegate to MainWindow::onExecutePlan via the editor's signal so we
    // share a single execution code path (stock creation + Executor + view
    // push happens there).  TODO: stock-selection dialog.
    emit editor->executeRequested(plan);
}

void AppMenus::onNlToEditOpStub()
{
    auto* editor = m_window->processPlanEditor();
    if (!editor) return;

    bool ok = false;
    const QString text = QInputDialog::getText(
        m_window,
        tr("NL → EditOp (stub)"),
        tr("Natural-language instruction:"),
        QLineEdit::Normal,
        QString(),
        &ok);
    if (!ok || text.trimmed().isEmpty()) return;

    try {
        process::ProcessPlan plan = editor->currentPlan();
        adapt::EditOp op = adapt::stubNaturalLanguageToEditOp(
            text.toStdString(), plan);
        process::ProcessPlan newPlan = adapt::PlanEditor::apply(plan, op);
        editor->setPlan(newPlan);
        if (auto* dock = m_window->processPlanDock()) {
            dock->show();
            dock->raise();
        }
        m_window->statusBar()->showMessage(
            tr("Applied NL stub edit (%1).").arg(adapt::editKindName(op.kind)),
            4000);
    } catch (const skill::SkillError& e) {
        QMessageBox::warning(m_window, tr("NL → EditOp (stub)"),
                             tr("SkillError: %1").arg(QString::fromUtf8(e.what())));
    } catch (const std::exception& e) {
        QMessageBox::critical(m_window, tr("NL → EditOp (stub)"),
                              tr("Exception: %1").arg(QString::fromUtf8(e.what())));
    }
}

}  // namespace koocadcam::gui
