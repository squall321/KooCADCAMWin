// @lat: [[architecture/overview#src/gui/]]

#include "ProcessPlanEditor.hpp"

#include <process/StepInvocation.hpp>

#include <QDialog>
#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include <nlohmann/json.hpp>

#include <sstream>

namespace koocadcam::gui {

namespace {

// Build a short one-line param summary for the list row.  We pick a small
// number of common keys to surface, then fall back to "{N keys}" for the
// rest.  Long JSON dump is reserved for the edit dialog.
QString summariseParams(const nlohmann::json& params)
{
    if (!params.is_object() || params.empty()) {
        return "{}";
    }

    static const std::vector<std::string> kKeysOfInterest = {
        "diameter_mm",
        "depth_mm",
        "width_mm",
        "height_mm",
        "length_mm",
        "radius_mm",
        "position_x_mm",
        "position_y_mm",
        "angle_deg",
        "pitch_mm",
    };

    std::ostringstream oss;
    bool first = true;
    int  shown = 0;
    for (const auto& k : kKeysOfInterest) {
        if (params.contains(k)) {
            if (!first) oss << ", ";
            first = false;
            oss << k << "=";
            const auto& v = params.at(k);
            if (v.is_number()) {
                oss << v.get<double>();
            } else if (v.is_string()) {
                oss << v.get<std::string>();
            } else {
                oss << v.dump();
            }
            ++shown;
            if (shown >= 4) break;
        }
    }
    if (shown == 0) {
        oss << params.size() << " keys";
    } else if (static_cast<int>(params.size()) > shown) {
        oss << ", +" << (static_cast<int>(params.size()) - shown) << " more";
    }
    return QString::fromStdString(oss.str());
}

QString stepRowText(int idx, const process::StepInvocation& step)
{
    QString summary = summariseParams(step.params);
    return QString("[%1] %2 (%3)")
        .arg(idx)
        .arg(QString::fromStdString(step.skill_id))
        .arg(summary);
}

}  // namespace

ProcessPlanEditor::ProcessPlanEditor(QWidget* parent)
    : QWidget(parent)
{
    buildUi();
    rebuildList();
}

ProcessPlanEditor::~ProcessPlanEditor() = default;

void ProcessPlanEditor::buildUi()
{
    QVBoxLayout* vlay = new QVBoxLayout(this);
    vlay->setContentsMargins(6, 6, 6, 6);
    vlay->setSpacing(6);

    QLabel* header = new QLabel(tr("Process Plan"), this);
    QFont f = header->font();
    f.setBold(true);
    header->setFont(f);
    vlay->addWidget(header);

    m_stepList = new QListWidget(this);
    m_stepList->setSelectionMode(QAbstractItemView::SingleSelection);
    m_stepList->setToolTip(tr("Double-click a step to edit its JSON params"));
    vlay->addWidget(m_stepList, /* stretch */ 1);

    connect(m_stepList, &QListWidget::itemDoubleClicked,
            this, [this](QListWidgetItem* item) {
                onStepRowDoubleClicked(m_stepList->row(item));
            });

    // ── Step manipulation buttons ────────────────────────────────────────────
    QHBoxLayout* btnRow = new QHBoxLayout;
    btnRow->setSpacing(4);

    m_addBtn    = new QPushButton(tr("Add Step"),    this);
    m_removeBtn = new QPushButton(tr("Remove"),      this);
    m_clearBtn  = new QPushButton(tr("Clear"),       this);
    btnRow->addWidget(m_addBtn);
    btnRow->addWidget(m_removeBtn);
    btnRow->addWidget(m_clearBtn);
    btnRow->addStretch();
    vlay->addLayout(btnRow);

    connect(m_addBtn,    &QPushButton::clicked, this, &ProcessPlanEditor::onAddStepClicked);
    connect(m_removeBtn, &QPushButton::clicked, this, &ProcessPlanEditor::onRemoveStepClicked);
    connect(m_clearBtn,  &QPushButton::clicked, this, &ProcessPlanEditor::onClearClicked);

    // ── Execute row ──────────────────────────────────────────────────────────
    m_executeBtn = new QPushButton(tr("Execute Plan"), this);
    vlay->addWidget(m_executeBtn);
    connect(m_executeBtn, &QPushButton::clicked, this, &ProcessPlanEditor::onExecuteClicked);

    // ── NL input row ─────────────────────────────────────────────────────────
    QLabel* nlLabel = new QLabel(tr("Natural-language instruction:"), this);
    vlay->addWidget(nlLabel);

    QHBoxLayout* nlRow = new QHBoxLayout;
    nlRow->setSpacing(4);
    m_nlInput     = new QLineEdit(this);
    m_nlInput->setPlaceholderText(tr("e.g. \"increase diameter_mm by 2\", \"remove last step\""));
    m_nlSubmitBtn = new QPushButton(tr("Submit"), this);
    nlRow->addWidget(m_nlInput, /* stretch */ 1);
    nlRow->addWidget(m_nlSubmitBtn);
    vlay->addLayout(nlRow);

    connect(m_nlSubmitBtn, &QPushButton::clicked, this, &ProcessPlanEditor::onNlSubmitClicked);
    connect(m_nlInput,     &QLineEdit::returnPressed,
            this, &ProcessPlanEditor::onNlSubmitClicked);
}

// ── Plan accessors ───────────────────────────────────────────────────────────

void ProcessPlanEditor::setPlan(const process::ProcessPlan& plan)
{
    m_plan = plan;
    rebuildList();
}

process::ProcessPlan ProcessPlanEditor::currentPlan() const
{
    return m_plan;
}

void ProcessPlanEditor::rebuildList()
{
    m_stepList->clear();
    const auto& steps = m_plan.steps();
    for (int i = 0; i < static_cast<int>(steps.size()); ++i) {
        m_stepList->addItem(stepRowText(i, steps[i]));
    }
}

// ── Slot: add step ───────────────────────────────────────────────────────────

void ProcessPlanEditor::onAddStepClicked()
{
    bool ok = false;
    const QString skillId = QInputDialog::getText(
        this,
        tr("Add Step"),
        tr("Skill id (e.g. drill_hole, mill_rect_pocket):"),
        QLineEdit::Normal,
        QString(),
        &ok);
    if (!ok || skillId.isEmpty()) return;

    process::StepInvocation step;
    step.skill_id = skillId.toStdString();
    step.params   = nlohmann::json::object();

    m_plan.append(step);
    rebuildList();
    m_stepList->setCurrentRow(m_stepList->count() - 1);
}

// ── Slot: remove selected step ───────────────────────────────────────────────

void ProcessPlanEditor::onRemoveStepClicked()
{
    int row = m_stepList->currentRow();
    if (row < 0 || row >= m_plan.size()) {
        return;
    }
    // ProcessPlan has no built-in remove; rebuild via toJson + drop + fromJson.
    nlohmann::json j = m_plan.toJson();
    j["steps"].erase(j["steps"].begin() + row);
    m_plan = process::ProcessPlan::fromJson(j);
    rebuildList();
}

// ── Slot: clear all steps ────────────────────────────────────────────────────

void ProcessPlanEditor::onClearClicked()
{
    m_plan = process::ProcessPlan{};
    rebuildList();
}

// ── Slot: execute plan ───────────────────────────────────────────────────────

void ProcessPlanEditor::onExecuteClicked()
{
    emit executeRequested(m_plan);
}

// ── Slot: submit natural-language instruction ────────────────────────────────

void ProcessPlanEditor::onNlSubmitClicked()
{
    const QString text = m_nlInput->text().trimmed();
    if (text.isEmpty()) return;
    emit naturalLanguageRequested(text);
    m_nlInput->clear();
}

// ── Slot: edit step JSON params via dialog ───────────────────────────────────

void ProcessPlanEditor::onStepRowDoubleClicked(int row)
{
    if (row < 0 || row >= m_plan.size()) return;

    const process::StepInvocation& step = m_plan.steps().at(row);

    // Build a JSON object representing the full step so the user can edit
    // skill_id / params / depends_on / note together.
    nlohmann::json j = {
        { "skill_id",   step.skill_id },
        { "params",     step.params },
        { "depends_on", step.depends_on },
        { "note",       step.note },
    };
    const QString initialText = QString::fromStdString(j.dump(2));

    QDialog dlg(this);
    dlg.setWindowTitle(tr("Edit Step [%1]").arg(row));
    dlg.resize(540, 420);

    QVBoxLayout* lay = new QVBoxLayout(&dlg);

    QLabel* hint = new QLabel(
        tr("Edit the step's JSON representation.  Fields: skill_id, params, "
           "depends_on, note."),
        &dlg);
    hint->setWordWrap(true);
    lay->addWidget(hint);

    QPlainTextEdit* editor = new QPlainTextEdit(&dlg);
    editor->setPlainText(initialText);
    editor->setLineWrapMode(QPlainTextEdit::NoWrap);
    QFont fixed("Courier New");
    fixed.setStyleHint(QFont::Monospace);
    editor->setFont(fixed);
    lay->addWidget(editor, /* stretch */ 1);

    QDialogButtonBox* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    lay->addWidget(buttons);
    connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    if (dlg.exec() != QDialog::Accepted) return;

    const std::string newText = editor->toPlainText().toStdString();
    nlohmann::json parsed;
    try {
        parsed = nlohmann::json::parse(newText);
    } catch (const std::exception& e) {
        QMessageBox::warning(
            this,
            tr("Edit Step"),
            tr("JSON parse error:\n%1").arg(QString::fromUtf8(e.what())));
        return;
    }
    if (!parsed.is_object()) {
        QMessageBox::warning(this, tr("Edit Step"),
                             tr("Top-level JSON value must be an object."));
        return;
    }

    // Surgically replace the step at `row`.  Round-trip through toJson +
    // fromJson keeps things simple and matches the read path elsewhere.
    nlohmann::json planJ = m_plan.toJson();
    planJ["steps"][row]  = parsed;
    m_plan = process::ProcessPlan::fromJson(planJ);
    rebuildList();
    m_stepList->setCurrentRow(row);
}

}  // namespace koocadcam::gui
