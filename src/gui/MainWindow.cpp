// @lat: [[architecture/overview#src/gui/]]

#include "MainWindow.hpp"
#include "AppMenus.hpp"
#include "OcctViewWidget.hpp"
#include "ParameterPanel.hpp"
#include "ProcessPlanEditor.hpp"

#include <adapt/LlmBridge.hpp>
#include <adapt/NaturalLanguageStub.hpp>
#include <adapt/PlanEditor.hpp>
#include <engine/WatchFrontModel.hpp>
#include <process/Executor.hpp>
#include <skills/Skill.hpp>
#include <skills/Stock.hpp>
#include <skills/Workpiece.hpp>

#include <QDockWidget>
#include <QMenuBar>
#include <QMessageBox>
#include <QStatusBar>
#include <QTimer>

#include <nlohmann/json.hpp>

namespace koocadcam::gui {

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("KooCADCAM — M1.1");
    resize(1280, 800);

    m_view = new OcctViewWidget(this);
    setCentralWidget(m_view);

    // ── Parameter panel dock ─────────────────────────────────────────────────
    m_paramPanel = new ParameterPanel(this);
    QDockWidget* dock = new QDockWidget(tr("Parameters"), this);
    dock->setWidget(m_paramPanel);
    dock->setAllowedAreas(Qt::RightDockWidgetArea | Qt::LeftDockWidgetArea);
    addDockWidget(Qt::RightDockWidgetArea, dock);

    connect(m_paramPanel, &ParameterPanel::specChanged,
            this, &MainWindow::onSpecChanged);

    // ── Process plan editor dock (Layer 3/4/5 GUI) ───────────────────────────
    // Constructed BEFORE AppMenus::install() because some Plan menu actions
    // need to query the dock's visibility to set their checkable state.
    m_processPlanEditor = new ProcessPlanEditor(this);
    m_processPlanDock   = new QDockWidget(tr("Process Plan"), this);
    m_processPlanDock->setWidget(m_processPlanEditor);
    m_processPlanDock->setAllowedAreas(
        Qt::RightDockWidgetArea | Qt::LeftDockWidgetArea | Qt::BottomDockWidgetArea);
    addDockWidget(Qt::LeftDockWidgetArea, m_processPlanDock);
    // Start hidden — toggled via the Ctrl+Shift+L menu action.
    m_processPlanDock->hide();

    connect(m_processPlanEditor, &ProcessPlanEditor::executeRequested,
            this, &MainWindow::onExecutePlan);
    connect(m_processPlanEditor, &ProcessPlanEditor::naturalLanguageRequested,
            this, &MainWindow::onApplyNL);

    // ── Menus (must come last — references the dock above) ───────────────────
    m_menus = new AppMenus(this);
    m_menus->install(menuBar());

    statusBar()->showMessage("Ready", 0);

    // Defer initial build until after the OcctViewWidget's initializeGL has
    // run (QOpenGLWidget creates its GL context lazily on first show + paint).
    // QTimer 0 fires from the event loop after Qt has had a chance to paint.
    QTimer::singleShot(0, this, [this]() {
        QTimer::singleShot(50, this, &MainWindow::onSpecChanged);
    });
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

// ── Slot: execute plan from editor ───────────────────────────────────────────
//
// Builds a hard-coded cuboid stock (50×50×20), runs the plan through the
// process Executor, pushes the resulting shape into the viewer.  Errors and
// partial-success results are surfaced via QMessageBox / statusBar.
//
// TODO: stock-selection dialog (cuboid vs cylinder, custom dimensions, material).

void MainWindow::onExecutePlan(const koocadcam::process::ProcessPlan& plan)
{
    try {
        auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
        if (!stock) {
            QMessageBox::critical(this, tr("Execute Plan"),
                                  tr("Failed to create cuboid stock."));
            return;
        }
        process::ExecutionResult result = process::Executor::execute(plan, stock);

        // Even on partial failure we still have the last good workpiece.
        if (result.workpiece && !result.workpiece->shape().IsNull()) {
            m_view->setShape(result.workpiece->shape());
        }

        if (result.ok()) {
            statusBar()->showMessage(
                tr("Plan executed: %1 step(s) OK")
                    .arg(static_cast<int>(result.signatures.size())),
                4000);
        } else {
            QString msg = tr("Plan failed at step %1").arg(result.failedAtStep);
            if (!result.errors.empty()) {
                msg += ":\n";
                msg += QString::fromStdString(result.errors.front());
            }
            QMessageBox::warning(this, tr("Execute Plan"), msg);
            statusBar()->showMessage(
                tr("Partial result — %1 step(s) succeeded before failure")
                    .arg(static_cast<int>(result.signatures.size())),
                5000);
        }
    } catch (const skill::SkillError& e) {
        QMessageBox::critical(this, tr("Execute Plan"),
                              tr("SkillError: %1").arg(QString::fromUtf8(e.what())));
    } catch (const std::exception& e) {
        QMessageBox::critical(this, tr("Execute Plan"),
                              tr("Exception: %1").arg(QString::fromUtf8(e.what())));
    }
}

// ── Slot: apply natural-language instruction ────────────────────────────────
//
// Calls into the slice-1 rule-based stub.  Once `LlmBridge`'s real-provider
// branch is wired, this will route via `LlmBridge::run(LlmRequest{...})`
// instead.  For now the bridge is used in fallback-via-stub mode so the GUI
// already exercises the same return shape.

void MainWindow::onApplyNL(const QString& instruction)
{
    if (!m_processPlanEditor) return;
    if (instruction.trimmed().isEmpty()) return;

    try {
        process::ProcessPlan plan = m_processPlanEditor->currentPlan();

        // Route through LlmBridge in "mock" mode → falls back to the stub so
        // we get the same uniform LlmResponse envelope as the eventual real
        // provider.  Once real Anthropic / OpenAI HTTP lands, switch the
        // provider field here and Settings will expose a model picker.
        adapt::LlmRequest req;
        req.config.provider = "mock";
        req.current_plan    = plan;
        req.instruction     = instruction.toStdString();
        adapt::LlmResponse resp = adapt::LlmBridge::run(req);

        if (!resp.success || !resp.edit) {
            QString msg = tr("Could not map instruction to an EditOp.");
            if (!resp.error.empty()) {
                msg += QStringLiteral("\n") + QString::fromStdString(resp.error);
            }
            QMessageBox::information(this, tr("NL → EditOp"), msg);
            return;
        }

        process::ProcessPlan newPlan = adapt::PlanEditor::apply(plan, *resp.edit);
        m_processPlanEditor->setPlan(newPlan);
        statusBar()->showMessage(tr("Applied NL edit (%1).")
                                     .arg(adapt::editKindName(resp.edit->kind)),
                                 4000);
    } catch (const skill::SkillError& e) {
        QMessageBox::warning(this, tr("NL → EditOp"),
                             tr("SkillError: %1").arg(QString::fromUtf8(e.what())));
    } catch (const std::exception& e) {
        QMessageBox::critical(this, tr("NL → EditOp"),
                              tr("Exception: %1").arg(QString::fromUtf8(e.what())));
    }
}

}  // namespace koocadcam::gui
