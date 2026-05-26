// @lat: [[architecture/overview#src/gui/]]

#include "ParameterPanel.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTableWidget>
#include <QTableWidgetItem>
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

    // ── Display pocket ───────────────────────────────────────────────────────
    {
        QLabel* sectionLabel = new QLabel(tr("── Display Pocket ──"), this);
        QFont sf = sectionLabel->font();
        sf.setBold(true);
        sectionLabel->setFont(sf);
        vlay->addWidget(sectionLabel);

        m_displayDiameter = new QDoubleSpinBox(this);
        m_displayDiameter->setRange(20.0, 50.0);
        m_displayDiameter->setSingleStep(0.1);
        m_displayDiameter->setDecimals(2);
        m_displayDiameter->setValue(36.0);

        m_displayDepth = new QDoubleSpinBox(this);
        m_displayDepth->setRange(0.3, 2.0);
        m_displayDepth->setSingleStep(0.1);
        m_displayDepth->setDecimals(2);
        m_displayDepth->setValue(0.8);

        m_glassOffset = new QDoubleSpinBox(this);
        m_glassOffset->setRange(-1.0, 0.5);
        m_glassOffset->setSingleStep(0.05);
        m_glassOffset->setDecimals(3);
        m_glassOffset->setValue(-0.1);

        QWidget* dpContainer = new QWidget(this);
        QFormLayout* dpForm = new QFormLayout(dpContainer);
        dpForm->setContentsMargins(0, 0, 0, 0);
        dpForm->setSpacing(4);
        dpForm->addRow(tr("Display pocket diameter (mm)"), m_displayDiameter);
        dpForm->addRow(tr("Display pocket depth (mm)"),    m_displayDepth);
        dpForm->addRow(tr("Glass offset (mm, negative = recessed)"), m_glassOffset);
        vlay->addWidget(dpContainer);
    }

    // ── Crown cavity ─────────────────────────────────────────────────────────
    {
        QLabel* sectionLabel = new QLabel(tr("── Crown Cavity ──"), this);
        QFont sf = sectionLabel->font();
        sf.setBold(true);
        sectionLabel->setFont(sf);
        vlay->addWidget(sectionLabel);

        m_crownEnabled = new QCheckBox(tr("Enable crown cavity"), this);
        m_crownEnabled->setChecked(true);
        vlay->addWidget(m_crownEnabled);

        m_crownGroup = new QWidget(this);
        QFormLayout* cgForm = new QFormLayout(m_crownGroup);
        cgForm->setContentsMargins(0, 0, 0, 0);
        cgForm->setSpacing(4);

        m_crownAngle = new QDoubleSpinBox(m_crownGroup);
        m_crownAngle->setRange(0.0, 360.0);
        m_crownAngle->setSingleStep(1.0);
        m_crownAngle->setDecimals(2);
        m_crownAngle->setValue(0.0);

        m_crownHeight = new QDoubleSpinBox(m_crownGroup);
        m_crownHeight->setRange(0.0, 20.0);
        m_crownHeight->setSingleStep(0.1);
        m_crownHeight->setDecimals(2);
        m_crownHeight->setValue(5.0);

        m_crownDepth = new QDoubleSpinBox(m_crownGroup);
        m_crownDepth->setRange(1.0, 5.0);
        m_crownDepth->setSingleStep(0.1);
        m_crownDepth->setDecimals(2);
        m_crownDepth->setValue(2.5);

        m_crownDiameter = new QDoubleSpinBox(m_crownGroup);
        m_crownDiameter->setRange(2.0, 10.0);
        m_crownDiameter->setSingleStep(0.1);
        m_crownDiameter->setDecimals(2);
        m_crownDiameter->setValue(3.0);

        m_crownShaft = new QDoubleSpinBox(m_crownGroup);
        m_crownShaft->setRange(0.8, 3.0);
        m_crownShaft->setSingleStep(0.1);
        m_crownShaft->setDecimals(2);
        m_crownShaft->setValue(1.5);

        cgForm->addRow(tr("Crown angle (°, 0=3 o'clock)"), m_crownAngle);
        cgForm->addRow(tr("Crown height-pos (mm)"),         m_crownHeight);
        cgForm->addRow(tr("Crown cavity depth (mm)"),       m_crownDepth);
        cgForm->addRow(tr("Crown diameter (mm)"),           m_crownDiameter);
        cgForm->addRow(tr("Crown shaft hole (mm)"),         m_crownShaft);

        vlay->addWidget(m_crownGroup);
        m_crownGroup->setVisible(true);
    }

    // ── Side buttons ─────────────────────────────────────────────────────────
    {
        QLabel* sectionLabel = new QLabel(tr("── Side Buttons ──"), this);
        QFont sf = sectionLabel->font();
        sf.setBold(true);
        sectionLabel->setFont(sf);
        vlay->addWidget(sectionLabel);

        m_sideButtonsTable = new QTableWidget(0, 6, this);
        m_sideButtonsTable->setHorizontalHeaderLabels(
            {tr("angle"), tr("height"), tr("length"), tr("width"), tr("depth"), tr("taper")});
        m_sideButtonsTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        m_sideButtonsTable->setMinimumHeight(80);  // ~3 rows visible
        vlay->addWidget(m_sideButtonsTable);

        QWidget* btnRow = new QWidget(this);
        QHBoxLayout* btnLay = new QHBoxLayout(btnRow);
        btnLay->setContentsMargins(0, 0, 0, 0);
        m_addButtonBtn    = new QPushButton(tr("Add button"), btnRow);
        m_removeButtonBtn = new QPushButton(tr("Remove last"), btnRow);
        btnLay->addWidget(m_addButtonBtn);
        btnLay->addWidget(m_removeButtonBtn);
        vlay->addWidget(btnRow);

        // Pre-seed one default row
        {
            nlohmann::json defaultRow = {
                {"angle_deg", 60.0}, {"height_mm", 5.0}, {"length_mm", 5.0},
                {"width_mm",  2.0},  {"depth_mm",  0.8}, {"taper_deg", 0.0}
            };
            appendButtonRow(defaultRow);
        }
    }

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

    // Display pocket spins
    connectSpin(m_displayDiameter);
    connectSpin(m_displayDepth);
    connectSpin(m_glassOffset);

    // Crown cavity connections
    connectSpin(m_crownAngle);
    connectSpin(m_crownHeight);
    connectSpin(m_crownDepth);
    connectSpin(m_crownDiameter);
    connectSpin(m_crownShaft);
    connect(m_crownEnabled, &QCheckBox::toggled,
            this, &ParameterPanel::onCrownEnabledToggled);

    // Side buttons connections
    connect(m_sideButtonsTable, &QTableWidget::cellChanged,
            this, [this](int, int) { onAnyValueChanged(); });
    connect(m_addButtonBtn,    &QPushButton::clicked,
            this, &ParameterPanel::onAddButtonRow);
    connect(m_removeButtonBtn, &QPushButton::clicked,
            this, &ParameterPanel::onRemoveLastButtonRow);

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

void ParameterPanel::onCrownEnabledToggled(bool checked)
{
    m_crownGroup->setVisible(checked);
    scheduleEmit();
}

void ParameterPanel::onAddButtonRow()
{
    nlohmann::json newRow = {
        {"angle_deg", 90.0}, {"height_mm", 5.0}, {"length_mm", 5.0},
        {"width_mm",  2.0},  {"depth_mm",  0.8}, {"taper_deg", 0.0}
    };
    appendButtonRow(newRow);
    scheduleEmit();
}

void ParameterPanel::onRemoveLastButtonRow()
{
    const int rows = m_sideButtonsTable->rowCount();
    if (rows > 0) {
        m_sideButtonsTable->removeRow(rows - 1);
        scheduleEmit();
    }
}

void ParameterPanel::appendButtonRow(const nlohmann::json& obj)
{
    // Temporarily block cellChanged so populating cells doesn't trigger debounce
    const QSignalBlocker blocker(m_sideButtonsTable);

    auto getDouble = [&](const char* key, double fallback = 0.0) -> double {
        if (obj.contains(key) && obj[key].is_number())
            return obj[key].get<double>();
        return fallback;
    };

    const int row = m_sideButtonsTable->rowCount();
    m_sideButtonsTable->insertRow(row);

    auto setCell = [&](int col, double val) {
        auto* item = new QTableWidgetItem(QString::number(val, 'f', 4));
        m_sideButtonsTable->setItem(row, col, item);
    };

    setCell(0, getDouble("angle_deg"));
    setCell(1, getDouble("height_mm"));
    setCell(2, getDouble("length_mm"));
    setCell(3, getDouble("width_mm"));
    setCell(4, getDouble("depth_mm"));
    setCell(5, getDouble("taper_deg"));
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

    nlohmann::json spec = {
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

    // Display pocket
    spec["display_pocket"] = {
        {"d_pocket_mm",     m_displayDiameter->value()},
        {"depth_pocket_mm", m_displayDepth->value()},
        {"glass_offset_mm", m_glassOffset->value()}
    };

    // Crown cavity (omitted when unchecked — engine treats missing key as pass-through)
    if (m_crownEnabled->isChecked()) {
        spec["crown_cavity"] = {
            {"side_angle_deg", m_crownAngle->value()},
            {"height_pos_mm",  m_crownHeight->value()},
            {"depth_mm",       m_crownDepth->value()},
            {"diameter_mm",    m_crownDiameter->value()},
            {"shaft_dia_mm",   m_crownShaft->value()}
        };
    }

    // Side buttons
    nlohmann::json buttons = nlohmann::json::array();
    for (int r = 0; r < m_sideButtonsTable->rowCount(); ++r) {
        auto cell = [&](int c) -> double {
            auto* it = m_sideButtonsTable->item(r, c);
            return it ? it->text().toDouble() : 0.0;
        };
        buttons.push_back({
            {"angle_deg",  cell(0)}, {"height_mm", cell(1)},
            {"length_mm",  cell(2)}, {"width_mm",  cell(3)},
            {"depth_mm",   cell(4)}, {"taper_deg", cell(5)}
        });
    }
    spec["side_buttons"] = buttons;

    return spec;
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
    const QSignalBlocker bDispDiam(m_displayDiameter);
    const QSignalBlocker bDispDepth(m_displayDepth);
    const QSignalBlocker bGlassOff(m_glassOffset);
    const QSignalBlocker bCrownEn(m_crownEnabled);
    const QSignalBlocker bCrownAng(m_crownAngle);
    const QSignalBlocker bCrownH(m_crownHeight);
    const QSignalBlocker bCrownD(m_crownDepth);
    const QSignalBlocker bCrownDia(m_crownDiameter);
    const QSignalBlocker bCrownShaft(m_crownShaft);
    const QSignalBlocker bTable(m_sideButtonsTable);

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

    // display_pocket
    if (spec.contains("display_pocket")) {
        const auto& dp = spec["display_pocket"];
        if (dp.contains("d_pocket_mm"))
            m_displayDiameter->setValue(dp["d_pocket_mm"].get<double>());
        if (dp.contains("depth_pocket_mm"))
            m_displayDepth->setValue(dp["depth_pocket_mm"].get<double>());
        if (dp.contains("glass_offset_mm"))
            m_glassOffset->setValue(dp["glass_offset_mm"].get<double>());
    }

    // crown_cavity
    if (spec.contains("crown_cavity")) {
        const auto& cc = spec["crown_cavity"];
        m_crownEnabled->setChecked(true);
        if (cc.contains("side_angle_deg"))
            m_crownAngle->setValue(cc["side_angle_deg"].get<double>());
        if (cc.contains("height_pos_mm"))
            m_crownHeight->setValue(cc["height_pos_mm"].get<double>());
        if (cc.contains("depth_mm"))
            m_crownDepth->setValue(cc["depth_mm"].get<double>());
        if (cc.contains("diameter_mm"))
            m_crownDiameter->setValue(cc["diameter_mm"].get<double>());
        if (cc.contains("shaft_dia_mm"))
            m_crownShaft->setValue(cc["shaft_dia_mm"].get<double>());
        m_crownGroup->setVisible(true);
    } else {
        m_crownEnabled->setChecked(false);
        m_crownGroup->setVisible(false);
    }

    // side_buttons
    m_sideButtonsTable->setRowCount(0);
    if (spec.contains("side_buttons") && spec["side_buttons"].is_array()) {
        for (const auto& btnObj : spec["side_buttons"])
            appendButtonRow(btnObj);
    }

    updateFieldVisibility();
    // Does NOT emit specChanged.
}

}  // namespace koocadcam::gui
