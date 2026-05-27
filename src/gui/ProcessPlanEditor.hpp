#pragma once
// @lat: [[architecture/overview#src/gui/]]
//
// ProcessPlanEditor — dock widget that displays the current ProcessPlan as
// an editable QListWidget.  Each row is one StepInvocation rendered with a
// short one-line summary "[N] skill_id ({param summary})".  Double-clicking
// a row opens a QPlainTextEdit dialog with the step's JSON for free-form
// editing.
//
// The widget owns its own ProcessPlan (mutated by Add/Remove/Clear/Replace).
// `executeRequested` is emitted on Execute click — MainWindow then re-builds
// the shape with this plan.  `naturalLanguageRequested` is emitted on NL
// Submit click — MainWindow runs the LLM stub / bridge, applies the EditOp,
// and pushes the resulting plan back via `setPlan()`.

#include <process/ProcessPlan.hpp>

#include <QWidget>

class QLineEdit;
class QListWidget;
class QPushButton;

namespace koocadcam::gui {

class ProcessPlanEditor : public QWidget
{
    Q_OBJECT
public:
    explicit ProcessPlanEditor(QWidget* parent = nullptr);
    ~ProcessPlanEditor() override;

    void setPlan(const process::ProcessPlan& plan);
    [[nodiscard]] process::ProcessPlan currentPlan() const;

signals:
    // Emitted when the user clicks "Execute Plan" — MainWindow re-builds the
    // shape with this plan.
    void executeRequested(const process::ProcessPlan& plan);

    // Emitted when the user clicks "Apply NL Instruction" — MainWindow runs
    // the LLM stub or real bridge, then re-emits with a modified plan.
    void naturalLanguageRequested(const QString& instruction);

private slots:
    void onAddStepClicked();
    void onRemoveStepClicked();
    void onClearClicked();
    void onExecuteClicked();
    void onNlSubmitClicked();
    void onStepRowDoubleClicked(int row);  // open params edit dialog

private:
    void buildUi();
    void rebuildList();

    process::ProcessPlan m_plan;
    QListWidget* m_stepList   {nullptr};
    QLineEdit*   m_nlInput    {nullptr};
    QPushButton* m_executeBtn {nullptr};
    QPushButton* m_addBtn     {nullptr};
    QPushButton* m_removeBtn  {nullptr};
    QPushButton* m_clearBtn   {nullptr};
    QPushButton* m_nlSubmitBtn{nullptr};
};

}  // namespace koocadcam::gui
