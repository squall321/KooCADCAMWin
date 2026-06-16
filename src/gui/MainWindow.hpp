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
class PhonePanel;
class ProcessPlanEditor;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    OcctViewWidget* viewWidget() const { return m_view; }
    [[nodiscard]] ParameterPanel* parameterPanel() const { return m_paramPanel; }
    [[nodiscard]] PhonePanel*     phonePanel()     const { return m_phonePanel; }

    // Product mode switches: raise the matching dock and rebuild from its spec.
    void activatePhone();
    void activateWatch();

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
    QDockWidget*       m_paramDock         {nullptr};
    PhonePanel*        m_phonePanel        {nullptr};
    QDockWidget*       m_phoneDock         {nullptr};
    bool               m_phoneMode         {false};
    ProcessPlanEditor* m_processPlanEditor {nullptr};
    QDockWidget*       m_processPlanDock   {nullptr};
};

}  // namespace koocadcam::gui
