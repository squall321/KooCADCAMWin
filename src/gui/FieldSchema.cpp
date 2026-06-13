// @lat: [[architecture/overview#src/gui/]]

#include "FieldSchema.hpp"

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QString>
#include <QStringList>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>
#include <QWidget>

#include <nlohmann/json.hpp>

#include <algorithm>
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

// ── Array-of-struct table (B6.3) ─────────────────────────────────────────────

QWidget* buildArrayTable(QWidget* parent, const ArrayTableSchema& schema,
                         QTableWidget*& outTable,
                         QPushButton*&  outAdd,
                         QPushButton*&  outRemove)
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

    QStringList headers;
    headers.reserve(static_cast<int>(schema.cols.size()));
    for (const ColumnDescriptor& cd : schema.cols)
        headers << QString::fromUtf8(cd.header);

    QTableWidget* table =
        new QTableWidget(0, static_cast<int>(schema.cols.size()), container);
    table->setHorizontalHeaderLabels(headers);
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table->setMinimumHeight(80);  // ~3 rows visible (matches side_buttons)
    table->setObjectName(QString::fromUtf8(schema.arrayKey) + QLatin1String(".table"));
    vlay->addWidget(table);

    QWidget* btnRow = new QWidget(container);
    QHBoxLayout* btnLay = new QHBoxLayout(btnRow);
    btnLay->setContentsMargins(0, 0, 0, 0);
    QPushButton* addBtn = new QPushButton(QStringLiteral("Add row"), btnRow);
    QPushButton* removeBtn = new QPushButton(QStringLiteral("Remove last"), btnRow);
    addBtn->setObjectName(QString::fromUtf8(schema.arrayKey) + QLatin1String(".add"));
    removeBtn->setObjectName(QString::fromUtf8(schema.arrayKey) + QLatin1String(".remove"));
    btnLay->addWidget(addBtn);
    btnLay->addWidget(removeBtn);
    vlay->addWidget(btnRow);

    outTable  = table;
    outAdd    = addBtn;
    outRemove = removeBtn;
    return container;
}

void appendArrayRow(QTableWidget* table, const ArrayTableSchema& schema,
                    const nlohmann::json& row)
{
    // Block cellChanged so populating cells doesn't trigger the debounce.
    const QSignalBlocker blocker(table);

    const int r = table->rowCount();
    table->insertRow(r);

    for (int c = 0; c < static_cast<int>(schema.cols.size()); ++c) {
        const ColumnDescriptor& cd = schema.cols[c];
        const bool present =
            row.is_object() && row.contains(cd.key) && row[cd.key].is_number();

        QString text;
        if (present) {
            text = QString::number(row[cd.key].get<double>(), 'f', cd.decimals);
        } else if (cd.optional) {
            // Missing optional value → blank cell (the "no key" state).
            text = QString();
        } else {
            text = QString::number(cd.defaultValue, 'f', cd.decimals);
        }
        table->setItem(r, c, new QTableWidgetItem(text));
    }
}

nlohmann::json readArrayTable(const QTableWidget* table,
                              const ArrayTableSchema& schema)
{
    nlohmann::json arr = nlohmann::json::array();
    for (int r = 0; r < table->rowCount(); ++r) {
        nlohmann::json obj = nlohmann::json::object();
        for (int c = 0; c < static_cast<int>(schema.cols.size()); ++c) {
            const ColumnDescriptor& cd = schema.cols[c];
            const QTableWidgetItem* it = table->item(r, c);
            const QString raw = it ? it->text().trimmed() : QString();

            if (cd.optional) {
                // Blank cell → omit the key entirely.
                if (raw.isEmpty())
                    continue;
                bool ok = false;
                const double v = raw.toDouble(&ok);
                // A zero (or unparsable) optional cell also means "no key".
                if (!ok || v == 0.0)
                    continue;
                obj[cd.key] = std::clamp(v, cd.min, cd.max);
            } else {
                const double v = raw.isEmpty() ? cd.defaultValue : raw.toDouble();
                obj[cd.key] = std::clamp(v, cd.min, cd.max);
            }
        }
        arr.push_back(std::move(obj));
    }
    return arr;
}

void writeArrayTable(QTableWidget* table, const ArrayTableSchema& schema,
                     const nlohmann::json& array)
{
    {
        const QSignalBlocker blocker(table);
        table->setRowCount(0);
    }
    if (!array.is_array())
        return;
    for (const auto& row : array)
        appendArrayRow(table, schema, row);
}

}  // namespace koocadcam::gui
