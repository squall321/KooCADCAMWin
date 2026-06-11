// @lat: [[architecture/overview#src/gui/]]

#include "FieldSchema.hpp"

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QString>
#include <QVBoxLayout>
#include <QWidget>

#include <nlohmann/json.hpp>

#include <cmath>

namespace koocadcam::gui {

// ── FieldControl ─────────────────────────────────────────────────────────────

double FieldControl::value() const
{
    if (ispin)
        return static_cast<double>(ispin->value());
    return dspin ? dspin->value() : 0.0;
}

void FieldControl::setValue(double v) const
{
    if (ispin) {
        const QSignalBlocker blocker(ispin);
        ispin->setValue(static_cast<int>(std::lround(v)));
    } else if (dspin) {
        const QSignalBlocker blocker(dspin);
        dspin->setValue(v);
    }
}

// ── buildGroupWidgets ────────────────────────────────────────────────────────

QWidget* buildGroupWidgets(QWidget* parent, const GroupSchema& schema,
                           ControlMap& outControls)
{
    QWidget* container = new QWidget(parent);
    QVBoxLayout* vlay = new QVBoxLayout(container);
    vlay->setContentsMargins(0, 0, 0, 0);
    vlay->setSpacing(4);

    if (schema.title) {
        QLabel* sectionLabel =
            new QLabel(QString::fromUtf8(schema.title), container);
        QFont sf = sectionLabel->font();
        sf.setBold(true);
        sectionLabel->setFont(sf);
        vlay->addWidget(sectionLabel);
    }

    QFormLayout* form = new QFormLayout();
    form->setContentsMargins(0, 0, 0, 0);
    form->setSpacing(4);

    for (const FieldDescriptor& fd : schema.fields) {
        const QString objectName = QString::fromUtf8(schema.groupKey)
                                   + QLatin1Char('.')
                                   + QString::fromUtf8(fd.key);
        FieldControl ctl;
        QWidget* editor = nullptr;
        if (fd.isInt) {
            QSpinBox* sb = new QSpinBox(container);
            sb->setRange(static_cast<int>(std::lround(fd.min)),
                         static_cast<int>(std::lround(fd.max)));
            sb->setSingleStep(static_cast<int>(std::lround(fd.step)));
            sb->setValue(static_cast<int>(std::lround(fd.defaultValue)));
            sb->setObjectName(objectName);
            ctl.ispin = sb;
            editor = sb;
        } else {
            QDoubleSpinBox* sb = new QDoubleSpinBox(container);
            sb->setDecimals(fd.decimals);
            sb->setRange(fd.min, fd.max);
            sb->setSingleStep(fd.step);
            sb->setValue(fd.defaultValue);
            sb->setObjectName(objectName);
            ctl.dspin = sb;
            editor = sb;
        }
        form->addRow(QString::fromUtf8(fd.label), editor);
        outControls[fd.key] = ctl;
    }

    vlay->addLayout(form);
    return container;
}

// ── readGroup (widgets → spec) ───────────────────────────────────────────────

void readGroup(nlohmann::json& spec, const GroupSchema& schema,
               const ControlMap& controls)
{
    nlohmann::json obj = nlohmann::json::object();
    for (const FieldDescriptor& fd : schema.fields) {
        const auto it = controls.find(fd.key);
        if (it == controls.end())
            continue;
        if (fd.isInt)
            obj[fd.key] = static_cast<int>(std::lround(it->second.value()));
        else
            obj[fd.key] = it->second.value();
    }
    spec[schema.groupKey] = std::move(obj);
}

// ── writeGroup (spec → widgets) ──────────────────────────────────────────────

void writeGroup(const nlohmann::json& spec, const GroupSchema& schema,
                const ControlMap& controls)
{
    if (!spec.contains(schema.groupKey))
        return;
    const nlohmann::json& obj = spec[schema.groupKey];
    if (!obj.is_object())
        return;

    for (const FieldDescriptor& fd : schema.fields) {
        if (!obj.contains(fd.key) || !obj[fd.key].is_number())
            continue;
        const auto it = controls.find(fd.key);
        if (it == controls.end())
            continue;
        it->second.setValue(obj[fd.key].get<double>());  // signal-blocked
    }
}

// ── connectGroup ─────────────────────────────────────────────────────────────

void connectGroup(const ControlMap& controls, QObject* receiver,
                  const std::function<void()>& slot)
{
    for (const auto& entry : controls) {
        const FieldControl& ctl = entry.second;
        if (ctl.dspin) {
            QObject::connect(ctl.dspin,
                             qOverload<double>(&QDoubleSpinBox::valueChanged),
                             receiver, [slot](double) { slot(); });
        }
        if (ctl.ispin) {
            QObject::connect(ctl.ispin,
                             qOverload<int>(&QSpinBox::valueChanged),
                             receiver, [slot](int) { slot(); });
        }
    }
}

}  // namespace koocadcam::gui
