// @lat: [[architecture/overview#src/gui/]]

#include "ParameterPanel.hpp"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <nlohmann/json.hpp>

namespace {

nlohmann::json initialSpec()
{
    return {
        {"schema_version", "1.0.0"},
        {"product_name", "Default Round Watch 44"},
        {"form_factor", "round"},
        {"base", {{"diameter_mm", 44.0}, {"thickness_mm", 10.0}}},
        {"corner_radius", {{"r_top_mm", 1.0}, {"r_side_mm", 0.6}}},
        {"bezel", {{"width_mm", 3.0}, {"depth_mm", 1.0}, {"taper_deg", 0.0}}}
    };
}

// Helper: create a labeled row widget containing a single child widget.
// Returns the row widget (for show/hide) and sets labelOut/childOut.
QWidget* makeRow(QWidget* parent, const QString& labelText, QWidget* child)
{
    QWidget* row = new QWidget(parent);
    QHBoxLayout* lay = new QHBoxLayout(row);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->addWidget(new QLabel(labelText, row));
    lay->addWidget(child);
    row->setLayout(lay);
    return row;
}

}  // namespace

namespace koocadcam::gui {

ParameterPanel::ParameterPanel(QWidget* parent)
    : QWidget(parent)
{
    m_debounce = new QTimer(this);
    m_debounce->setSingleShot(true);
    m_debounce->setInterval(200);
    connect(m_debounce, &QTimer::timeout, this, &ParameterPanel::onDebounceTimeout);

    buildUi();

    // Apply defaults without emitting a signal.
    setSpec(initialSpec());
}

ParameterPanel::~ParameterPanel() = default;

// ── buildUi ──────────────────────────────────────────────────────────────────

void ParameterPanel::buildUi()
{
    QVBoxLayout* vlay = new QVBoxLayout(this);
    vlay->setContentsMargins(6, 6, 6, 6);
    vlay->setSpacing(6);

    // Header
    QLabel* header = new QLabel(tr("Watch Parameters"), this);
    QFont f = header->font();
    f.setBold(true);
    header->setFont(f);
    vlay->addWidget(header);

    // Form factor combo
    m_formFactor = new QComboBox(this);
    m_formFactor->addItem(tr("round"),  QVariant(QString("round")));
    m_formFactor->addItem(tr("square"), QVariant(QString("square")));
    vlay->addWidget(m_formFactor);

    // ── Spin-box factory helpers ─────────────────────────────────────────────
    auto makeSpin = [this](double lo, double hi, double step, double def) -> QDoubleSpinBox* {
        QDoubleSpinBox* sb = new QDoubleSpinBox(this);
        sb->setDecimals(2);
        sb->setRange(lo, hi);
        sb->setSingleStep(step);
        sb->setValue(def);
        return sb;
    };

    // ── Base fields ──────────────────────────────────────────────────────────
    m_diameter  = makeSpin(30.0, 55.0, 0.1, 44.0);
    m_width     = makeSpin(30.0, 55.0, 0.1, 44.0);
    m_height    = makeSpin(30.0, 60.0, 0.1, 44.0);
    m_thickness = makeSpin( 5.0, 20.0, 0.1, 10.0);

    // Row widgets (for visibility toggling)
    m_diameterRow = makeRow(this, tr("Diameter (mm)"), m_diameter);
    m_widthRow    = makeRow(this, tr("Width (mm)"),    m_width);
    m_heightRow   = makeRow(this, tr("Height (mm)"),   m_height);

    // ── Corner radius ────────────────────────────────────────────────────────
    m_rTop  = makeSpin(0.2, 5.0, 0.1, 1.0);
    m_rSide = makeSpin(0.2, 3.0, 0.1, 0.6);

    // ── Bezel ────────────────────────────────────────────────────────────────
    m_bezelWidth = makeSpin(1.0, 8.0, 0.1, 3.0);
    m_bezelDepth = makeSpin(0.3, 3.0, 0.1, 1.0);
    m_bezelTaper = makeSpin(0.0, 10.0, 0.5, 0.0);

    // ── Form layout ──────────────────────────────────────────────────────────
    QWidget* formContainer = new QWidget(this);
    m_form = new QFormLayout(formContainer);
    m_form->setContentsMargins(0, 0, 0, 0);
    m_form->setSpacing(4);

    m_form->addRow(m_diameterRow);
    m_form->addRow(m_widthRow);
    m_form->addRow(m_heightRow);
    m_form->addRow(tr("Thickness (mm)"),   m_thickness);
    m_form->addRow(tr("R-top (mm)"),       m_rTop);
    m_form->addRow(tr("R-side (mm)"),      m_rSide);
    m_form->addRow(tr("Bezel width (mm)"), m_bezelWidth);
    m_form->addRow(tr("Bezel depth (mm)"), m_bezelDepth);
    m_form->addRow(tr("Bezel taper (°)"), m_bezelTaper);

    vlay->addWidget(formContainer);

    // Rebuild button
    m_rebuildButton = new QPushButton(tr("Rebuild Now"), this);
    vlay->addWidget(m_rebuildButton);

    vlay->addStretch();

    // ── Connections ──────────────────────────────────────────────────────────
    connect(m_formFactor, &QComboBox::currentIndexChanged,
            this, &ParameterPanel::onFormFactorChanged);

    auto connectSpin = [this](QDoubleSpinBox* sb) {
        connect(sb, qOverload<double>(&QDoubleSpinBox::valueChanged),
                this, &ParameterPanel::onAnyValueChanged);
    };
    connectSpin(m_diameter);
    connectSpin(m_width);
    connectSpin(m_height);
    connectSpin(m_thickness);
    connectSpin(m_rTop);
    connectSpin(m_rSide);
    connectSpin(m_bezelWidth);
    connectSpin(m_bezelDepth);
    connectSpin(m_bezelTaper);

    connect(m_rebuildButton, &QPushButton::clicked,
            this, &ParameterPanel::onRebuildClicked);
}

// ── Visibility ───────────────────────────────────────────────────────────────

void ParameterPanel::updateFieldVisibility()
{
    const bool isRound = (m_formFactor->currentData().toString() == QLatin1String("round"));
    m_diameterRow->setVisible(isRound);
    m_widthRow->setVisible(!isRound);
    m_heightRow->setVisible(!isRound);
}

// ── Slots ────────────────────────────────────────────────────────────────────

void ParameterPanel::onFormFactorChanged(int /*idx*/)
{
    updateFieldVisibility();
    scheduleEmit();
}

void ParameterPanel::onAnyValueChanged()
{
    scheduleEmit();
}

void ParameterPanel::scheduleEmit()
{
    m_debounce->start();  // restarts if already running
}

void ParameterPanel::onDebounceTimeout()
{
    emit specChanged();
}

void ParameterPanel::onRebuildClicked()
{
    m_debounce->stop();
    emit specChanged();
}

// ── currentSpec ──────────────────────────────────────────────────────────────

nlohmann::json ParameterPanel::currentSpec() const
{
    const std::string ff = m_formFactor->currentData().toString().toStdString();

    nlohmann::json base;
    if (ff == "round") {
        base["diameter_mm"] = m_diameter->value();
    } else {
        base["width_mm"]  = m_width->value();
        base["height_mm"] = m_height->value();
    }
    base["thickness_mm"] = m_thickness->value();

    return {
        {"schema_version", "1.0.0"},
        {"product_name",   "Custom"},
        {"form_factor",    ff},
        {"base",           base},
        {"corner_radius",  {{"r_top_mm",  m_rTop->value()},
                            {"r_side_mm", m_rSide->value()}}},
        {"bezel",          {{"width_mm",   m_bezelWidth->value()},
                            {"depth_mm",   m_bezelDepth->value()},
                            {"taper_deg",  m_bezelTaper->value()}}}
    };
}

// ── setSpec ──────────────────────────────────────────────────────────────────

void ParameterPanel::setSpec(const nlohmann::json& spec)
{
    // Block all signals while we update widgets.
    const QSignalBlocker bCombo(m_formFactor);
    const QSignalBlocker bDiam(m_diameter);
    const QSignalBlocker bW(m_width);
    const QSignalBlocker bH(m_height);
    const QSignalBlocker bThick(m_thickness);
    const QSignalBlocker bRTop(m_rTop);
    const QSignalBlocker bRSide(m_rSide);
    const QSignalBlocker bBW(m_bezelWidth);
    const QSignalBlocker bBD(m_bezelDepth);
    const QSignalBlocker bBT(m_bezelTaper);

    // form_factor
    if (spec.contains("form_factor")) {
        const std::string ff = spec["form_factor"].get<std::string>();
        const int idx = m_formFactor->findData(QVariant(QString::fromStdString(ff)));
        if (idx >= 0)
            m_formFactor->setCurrentIndex(idx);
    }

    // base
    if (spec.contains("base")) {
        const auto& base = spec["base"];
        if (base.contains("diameter_mm"))
            m_diameter->setValue(base["diameter_mm"].get<double>());
        if (base.contains("width_mm"))
            m_width->setValue(base["width_mm"].get<double>());
        if (base.contains("height_mm"))
            m_height->setValue(base["height_mm"].get<double>());
        if (base.contains("thickness_mm"))
            m_thickness->setValue(base["thickness_mm"].get<double>());
    }

    // corner_radius
    if (spec.contains("corner_radius")) {
        const auto& cr = spec["corner_radius"];
        if (cr.contains("r_top_mm"))
            m_rTop->setValue(cr["r_top_mm"].get<double>());
        if (cr.contains("r_side_mm"))
            m_rSide->setValue(cr["r_side_mm"].get<double>());
    }

    // bezel
    if (spec.contains("bezel")) {
        const auto& bz = spec["bezel"];
        if (bz.contains("width_mm"))
            m_bezelWidth->setValue(bz["width_mm"].get<double>());
        if (bz.contains("depth_mm"))
            m_bezelDepth->setValue(bz["depth_mm"].get<double>());
        if (bz.contains("taper_deg"))
            m_bezelTaper->setValue(bz["taper_deg"].get<double>());
    }

    updateFieldVisibility();
    // Does NOT emit specChanged.
}

}  // namespace koocadcam::gui
