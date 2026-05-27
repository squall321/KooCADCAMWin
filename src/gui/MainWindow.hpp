#pragma once
// @lat: [[architecture/overview#src/gui/]]

#include <process/ProcessPlan.hpp>

#include <QMainWindow>
#include <QString>

class QDockWidget;

namespace koocadcam::gui {

class OcctViewWidget;
class AppMenus;
class ParameterPanel;
class ProcessPlanEditor;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    OcctViewWidget* viewWidget() const { return m_view; }
    [[nodiscard]] ParameterPanel* parameterPanel() const { return m_paramPanel; }

    // Layer 3/4/5 GUI accessors — used by AppMenus to drive recognition,
    // plan execution, and natural-language editing.
    [[nodiscard]] ProcessPlanEditor* processPlanEditor() const { return m_processPlanEditor; }
    [[nodiscard]] QDockWidget*       processPlanDock()   const { return m_processPlanDock; }

private slots:
    void onSpecChanged();
    void onExecutePlan(const koocadcam::process::ProcessPlan& plan);
    void onApplyNL(const QString& instruction);

private:
    OcctViewWidget*    m_view              {nullptr};
    AppMenus*          m_menus             {nullptr};
    ParameterPanel*    m_paramPanel        {nullptr};
    ProcessPlanEditor* m_processPlanEditor {nullptr};
    QDockWidget*       m_processPlanDock   {nullptr};
};

}  // namespace koocadcam::gui
