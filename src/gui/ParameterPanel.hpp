#pragma once
// @lat: [[architecture/overview#src/gui/]]
// Sidebar dock widget for the Watch spec parameters.
// Field ranges follow feature-watch.md JSON Schema Draft 2020-12.

#include <QWidget>

#include <nlohmann/json_fwd.hpp>

#include "gui/FieldSchema.hpp"

#include <memory>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QPushButton;
class QTableWidget;
class QTimer;
class QFormLayout;

namespace koocadcam::gui {

class ParameterPanel : public QWidget
{
    Q_OBJECT
public:
    explicit ParameterPanel(QWidget* parent = nullptr);
    ~ParameterPanel() override;

    // Read current panel state as a Watch spec JSON.
    [[nodiscard]] nlohmann::json currentSpec() const;

    // Update panel widgets from the given spec (silently — does not emit specChanged).
    void setSpec(const nlohmann::json& spec);

    // Emit specChanged immediately (for callers who just called setSpec and want a rebuild).
    void triggerRebuild() { emit specChanged(); }

signals:
    // Emitted ~200 ms after the user's last change (debounced) and on Rebuild Now click.
    void specChanged();

private slots:
    void onFormFactorChanged(int idx);
    void onAnyValueChanged();
    void onRebuildClicked();
    void onDebounceTimeout();
    void onCrownEnabledToggled(bool checked);
    void onGrilleEnabledToggled(bool checked);
    void onFilletsEnabledToggled(bool checked);
    void onAddButtonRow();
    void onRemoveLastButtonRow();

private:
    void buildUi();
    void updateFieldVisibility();
    void scheduleEmit();
    void appendButtonRow(const nlohmann::json& obj);

    // Form factor selector
    QComboBox*       m_formFactor{nullptr};
    QFormLayout*     m_form{nullptr};

    // Base fields
    QDoubleSpinBox*  m_diameter{nullptr};   // round only
    QDoubleSpinBox*  m_width{nullptr};      // square only
    QDoubleSpinBox*  m_height{nullptr};     // square only
    QDoubleSpinBox*  m_thickness{nullptr};
    QWidget*         m_diameterRow{nullptr};
    QWidget*         m_widthRow{nullptr};
    QWidget*         m_heightRow{nullptr};

    // Corner radius
    QDoubleSpinBox*  m_rTop{nullptr};
    QDoubleSpinBox*  m_rSide{nullptr};

    // Bezel (B6.2 PoC: factory-built — see bezelSchema() in the .cpp)
    ControlMap       m_bezelControls;

    // Display pocket
    QDoubleSpinBox*  m_displayDiameter{nullptr};
    QDoubleSpinBox*  m_displayDepth{nullptr};
    QDoubleSpinBox*  m_glassOffset{nullptr};

    // Crown cavity
    QCheckBox*       m_crownEnabled{nullptr};
    QWidget*         m_crownGroup{nullptr};
    QDoubleSpinBox*  m_crownAngle{nullptr};
    QDoubleSpinBox*  m_crownHeight{nullptr};
    QDoubleSpinBox*  m_crownDepth{nullptr};
    QDoubleSpinBox*  m_crownDiameter{nullptr};
    QDoubleSpinBox*  m_crownShaft{nullptr};

    // Speaker grille (step 7, factory-built; checkbox mirrors crown pattern)
    QCheckBox*       m_grilleEnabled{nullptr};
    QWidget*         m_grilleGroup{nullptr};
    ControlMap       m_grilleControls;

    // Secondary fillets (step 10, factory-built)
    QCheckBox*       m_filletsEnabled{nullptr};
    QWidget*         m_filletsGroup{nullptr};
    ControlMap       m_filletsControls;

    // Side buttons
    QTableWidget*    m_sideButtonsTable{nullptr};
    QPushButton*     m_addButtonBtn{nullptr};
    QPushButton*     m_removeButtonBtn{nullptr};

    // Rear sensors (step 8) — generic array-of-struct table (B6.3)
    QTableWidget*    m_rearSensorsTable{nullptr};
    QPushButton*     m_rearSensorsAddBtn{nullptr};
    QPushButton*     m_rearSensorsRemoveBtn{nullptr};

    // Lugs (step 9) — generic array-of-struct table (B6.3)
    QTableWidget*    m_lugsTable{nullptr};
    QPushButton*     m_lugsAddBtn{nullptr};
    QPushButton*     m_lugsRemoveBtn{nullptr};

    QPushButton*     m_rebuildButton{nullptr};
    QTimer*          m_debounce{nullptr};

    // Last spec handed to setSpec(), retained so currentSpec() can preserve
    // keys the panel has no widgets for (any unknown future keys).  Without
    // this, a load → edit-one-field → save/rebuild silently strips those
    // steps.  speaker_grille / secondary_fillets (B6.3) and rear_sensors /
    // lugs (B6.3 array tables) are now panel-owned and are always overwritten
    // or erased on top — they are no longer pass-through.
    // unique_ptr keeps the heavy json type out of this header (json_fwd only).
    std::unique_ptr<nlohmann::json> m_baseSpec;
};

}  // namespace koocadcam::gui
