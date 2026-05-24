#pragma once
// @lat: [[architecture/overview#src/gui/]]
// Sidebar dock widget for the Watch spec parameters.
// Field ranges follow feature-watch.md JSON Schema Draft 2020-12.

#include <QWidget>

#include <nlohmann/json_fwd.hpp>

#include <memory>

class QComboBox;
class QDoubleSpinBox;
class QPushButton;
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

private:
    void buildUi();
    void updateFieldVisibility();
    void scheduleEmit();

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

    // Bezel
    QDoubleSpinBox*  m_bezelWidth{nullptr};
    QDoubleSpinBox*  m_bezelDepth{nullptr};
    QDoubleSpinBox*  m_bezelTaper{nullptr};

    QPushButton*     m_rebuildButton{nullptr};
    QTimer*          m_debounce{nullptr};
};

}  // namespace koocadcam::gui
