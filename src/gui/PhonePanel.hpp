#pragma once
// @lat: [[architecture/overview#src/gui/]]
// Sidebar dock widget for the PhoneFrontModel spec parameters (B6.4).
//
// Unlike ParameterPanel (which is hand-built for the watch), PhonePanel is
// fully schema-driven: it builds its whole UI from a list of GroupSchema +
// ArrayTableSchema using the FieldSchema factory, so currentSpec()/setSpec()
// are generic loops.  side_buttons (a string-enum "side" field the numeric
// factory does not cover) is preserved as pass-through via m_baseSpec.

#include <QWidget>

#include <nlohmann/json_fwd.hpp>

#include "gui/FieldSchema.hpp"

#include <memory>
#include <vector>

class QPushButton;
class QTableWidget;
class QTimer;

namespace koocadcam::gui {

class PhonePanel : public QWidget
{
    Q_OBJECT
public:
    explicit PhonePanel(QWidget* parent = nullptr);
    ~PhonePanel() override;

    // Read current panel state as a Phone spec JSON.
    [[nodiscard]] nlohmann::json currentSpec() const;

    // Update panel widgets from the given spec (silently — no specChanged).
    void setSpec(const nlohmann::json& spec);

    // Emit specChanged immediately (for callers who just called setSpec).
    void triggerRebuild() { emit specChanged(); }

signals:
    // Emitted ~200 ms after the user's last change (debounced) + on Rebuild Now.
    void specChanged();

private slots:
    void onAnyValueChanged();
    void onDebounceTimeout();
    void onRebuildClicked();

private:
    void buildUi();
    void scheduleEmit();

    // One spec group rendered via FieldSchema::buildGroupWidgets.
    struct GroupBinding { const GroupSchema* schema; ControlMap controls; };
    // One spec array rendered via FieldSchema::buildArrayTable.
    struct TableBinding {
        const ArrayTableSchema* schema;
        QTableWidget*           table  = nullptr;
        QPushButton*            add    = nullptr;
        QPushButton*            remove = nullptr;
    };

    std::vector<GroupBinding> m_groups;
    std::vector<TableBinding> m_tables;

    QPushButton* m_rebuildButton{nullptr};
    QTimer*      m_debounce{nullptr};

    // Last spec handed to setSpec(), retained so currentSpec() preserves keys
    // the panel has no widgets for (e.g. side_buttons, product_name).
    std::unique_ptr<nlohmann::json> m_baseSpec;
};

}  // namespace koocadcam::gui
