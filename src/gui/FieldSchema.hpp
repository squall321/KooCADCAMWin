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
class QSpinBox;
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

}  // namespace koocadcam::gui
