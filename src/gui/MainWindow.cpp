// @lat: [[architecture/overview#src/gui/]]

#include "MainWindow.hpp"
#include "AppMenus.hpp"
#include "OcctViewWidget.hpp"
#include "ParameterPanel.hpp"
#include "PhonePanel.hpp"
#include "ProcessPlanEditor.hpp"

#include <adapt/LlmBridge.hpp>
#include <adapt/NaturalLanguageStub.hpp>
#include <adapt/PlanEditor.hpp>
#include <engine/PhoneFrontModel.hpp>
#include <cam/ToolpathPlan.hpp>
#include <engine/ProductRegistry.hpp>
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

    // ── Parameter panel dock (watch) ─────────────────────────────────────────
    m_paramPanel = new ParameterPanel(this);
    m_paramDock = new QDockWidget(tr("Watch Parameters"), this);
    m_paramDock->setWidget(m_paramPanel);
    m_paramDock->setAllowedAreas(Qt::RightDockWidgetArea | Qt::LeftDockWidgetArea);
    addDockWidget(Qt::RightDockWidgetArea, m_paramDock);

    connect(m_paramPanel, &ParameterPanel::specChanged,
            this, &MainWindow::onSpecChanged);

    // ── Phone panel dock (tabbed behind the watch dock, hidden until New Phone) ─
    m_phonePanel = new PhonePanel(this);
    m_phoneDock = new QDockWidget(tr("Phone Parameters"), this);
    m_phoneDock->setWidget(m_phonePanel);
    m_phoneDock->setAllowedAreas(Qt::RightDockWidgetArea | Qt::LeftDockWidgetArea);
    addDockWidget(Qt::RightDockWidgetArea, m_phoneDock);
    tabifyDockWidget(m_paramDock, m_phoneDock);
    m_paramDock->raise();

    connect(m_phonePanel, &PhonePanel::specChanged,
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
    using koocadcam::engine::PhoneFrontModel;
    using koocadcam::engine::BuildWarning;
    try {
        // The product follows whichever panel emitted the change (robust to
        // tab switching); the initial timer-driven build has no panel sender
        // and defaults to the watch.  m_phoneMode tracks the last activation
        // for the menu/UX, but the build decision keys off the sender.
        const bool phone = (sender() == m_phonePanel) ||
                           (sender() != m_paramPanel && m_phoneMode);
        const char* product = phone ? "Phone" : "Watch";
        nlohmann::json spec = phone ? m_phonePanel->currentSpec()
                                    : m_paramPanel->currentSpec();
        std::vector<BuildWarning> warnings;
        // Dispatch through the product registry: the spec's "product" tag (set
        // by each panel's default, else inferred) selects the builder, so the
        // GUI no longer hard-codes the watch-vs-phone branch for the build.
        TopoDS_Shape shape = koocadcam::engine::buildProduct(spec, warnings);
        if (shape.IsNull()) {
            QString msg = QString("%1 build failed").arg(product);
            if (!warnings.empty())
                msg += QString::fromStdString(": " + warnings.front().message);
            statusBar()->showMessage(msg, 5000);
            return;
        }
        m_view->setShape(shape);

        // Run the manufacturability gate and surface the verdict (B6.4: the
        // live loop now reports DFM for both products, not just the demo menu).
        auto report = koocadcam::engine::runDFMForProduct(shape, spec);
        QString status = QString("%1 rebuilt — DFM %2 (%3 finding(s))")
            .arg(product)
            .arg(report.passed ? "PASS" : "FAIL")
            .arg(static_cast<int>(report.findings.size()));
        if (!warnings.empty())
            status += QString(", %1 warning(s)").arg(static_cast<int>(warnings.size()));
        statusBar()->showMessage(status, 4000);
    } catch (const std::exception& e) {
        statusBar()->showMessage(
            QString("Build exception: %1").arg(e.what()), 8000);
    }
}

void MainWindow::activatePhone()
{
    m_phoneMode = true;
    m_phoneDock->show();
    m_phoneDock->raise();
    m_phonePanel->triggerRebuild();
}

void MainWindow::activateWatch()
{
    m_phoneMode = false;
    m_paramDock->show();
    m_paramDock->raise();
    m_paramPanel->triggerRebuild();
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
            QString msg = tr("Plan executed: %1 step(s) OK")
                              .arg(static_cast<int>(result.signatures.size()));
            // CAM: generate + collision-verify toolpaths for the machined
            // features of the executed (e.g. reverse-engineered) plan.
            if (result.workpiece) {
                const auto cam = koocadcam::cam::planAndVerify(*result.workpiece);
                if (!cam.toolpaths.empty()) {
                    msg += tr(" — CAM: %1 toolpath(s), %2")
                               .arg(static_cast<int>(cam.toolpaths.size()))
                               .arg(cam.ok
                                        ? tr("collision-free")
                                        : tr("%1 collision(s)!")
                                              .arg(static_cast<int>(cam.collisions.size())));
                }
            }
            statusBar()->showMessage(msg, 5000);
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
