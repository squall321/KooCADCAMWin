#pragma once
// @lat: [[architecture/overview#src/gui/]]
// B6.2 schema-driven widget factory for ParameterPanel.
//
// A GroupSchema describes one spec JSON group ("bezel", "speaker_grille", …)
// as data; buildGroupWidgets() turns it into a section label + QFormLayout of
// spin boxes, and readGroup()/writeGroup() move values between the widgets
// and the spec JSON.  Field ranges follow data/schemas/watch.schema.json.

#include <nlohmann/json_fwd.hpp>

#include <functional>
#include <map>
#include <string>
#include <vector>

class QDoubleSpinBox;
class QObject;
class QPushButton;
class QSpinBox;
class QTableWidget;
class QWidget;

namespace koocadcam::gui {

// One spin-box row.  isInt selects QSpinBox (integer counts such as
// speaker_grille rows/cols) instead of QDoubleSpinBox.
struct FieldDescriptor {
    const char* key;           // JSON key inside the group object
    const char* label;         // human-readable row label (UTF-8)
    double      min;
    double      max;
    double      step;
    int         decimals;      // ignored when isInt
    double      defaultValue;
    bool        isInt = false; // true → QSpinBox
};

// One spec group rendered as a section.
struct GroupSchema {
    const char*                  groupKey; // spec JSON key ("bezel", …)
    const char*                  title;    // "── … ──" section label; nullptr = none
    std::vector<FieldDescriptor> fields;
};

// Either a double or an int spin box (exactly one pointer is non-null).
struct FieldControl {
    QDoubleSpinBox* dspin = nullptr;
    QSpinBox*       ispin = nullptr;

    [[nodiscard]] double value() const;
    // Sets the widget value WITHOUT emitting valueChanged (QSignalBlocker).
    void setValue(double v) const;
};

using ControlMap = std::map<std::string, FieldControl>;

// Build the section label (optional) + one QFormLayout row per field and
// return the container widget.  Each spin box gets
// objectName == "<groupKey>.<fieldKey>" so tests can findChild() it.
// Created controls are inserted into outControls keyed by field key.
QWidget* buildGroupWidgets(QWidget* parent, const GroupSchema& schema,
                           ControlMap& outControls);

// Widgets → spec: overwrite spec[groupKey] with a fresh object holding every
// field's current widget value (isInt fields are written as JSON integers).
void readGroup(nlohmann::json& spec, const GroupSchema& schema,
               const ControlMap& controls);

// Spec → widgets: for every field present in spec[groupKey], set the widget
// value silently (QSignalBlocker).  Absent keys leave the widget untouched.
void writeGroup(const nlohmann::json& spec, const GroupSchema& schema,
                const ControlMap& controls);

// Wire every control's valueChanged to `slot` (e.g. onAnyValueChanged).
// `receiver` is the connection context object (lifetime guard).
void connectGroup(const ControlMap& controls, QObject* receiver,
                  const std::function<void()>& slot);

// ── Array-of-struct table (B6.3) ─────────────────────────────────────────────
// rear_sensors / lugs are JSON arrays of homogeneous objects.  A generic
// QTableWidget renders one column per ColumnDescriptor and one row per array
// element; Add/Remove buttons grow/shrink the array.  Cell text is a plain
// number (no per-cell spin box) edited inline — ranges/decimals drive cell
// formatting and clamping in readArrayTable().

struct ColumnDescriptor {
    const char* key;          // JSON key inside each row object
    const char* header;       // column header text (UTF-8)
    double      min;
    double      max;
    double      step;         // reserved for future inline editors; unused today
    int         decimals;     // cell formatting precision
    double      defaultValue; // value for a freshly added row
    bool        optional = false; // true → blank/zero cell omits the key entirely
};

struct ArrayTableSchema {
    const char*                   arrayKey; // spec JSON key ("rear_sensors", "lugs")
    const char*                   title;    // section label (UTF-8); nullptr = none
    std::vector<ColumnDescriptor> cols;
};

// Build the section label (optional) + a QTableWidget + an Add/Remove button
// row, and return the container widget.  The table, add button and remove
// button get objectNames "<arrayKey>.table", "<arrayKey>.add",
// "<arrayKey>.remove" so tests can findChild() them.  Out-params receive the
// created widgets (the panel owns the connections + row population).
QWidget* buildArrayTable(QWidget* parent, const ArrayTableSchema& schema,
                         QTableWidget*& outTable,
                         QPushButton*&  outAdd,
                         QPushButton*&  outRemove);

// Append one row to the table, filling each column from `row` (falling back to
// the column default when the key is missing).  Signal-blocked so it does not
// fire cellChanged.  Pass an empty object to append a default row.
void appendArrayRow(QTableWidget* table, const ArrayTableSchema& schema,
                    const nlohmann::json& row);

// Table → JSON array: one object per row.  Optional columns whose cell is
// blank or zero are omitted from that row's object; non-optional columns are
// always written (parsed as a double, clamped to [min, max]).
[[nodiscard]] nlohmann::json readArrayTable(const QTableWidget* table,
                                            const ArrayTableSchema& schema);

// JSON array → table: clear the table and append one row per array element
// (signal-blocked).  A non-array / absent value leaves the table empty.
void writeArrayTable(QTableWidget* table, const ArrayTableSchema& schema,
                     const nlohmann::json& array);

}  // namespace koocadcam::gui
