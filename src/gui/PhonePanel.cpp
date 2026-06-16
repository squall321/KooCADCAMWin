// @lat: [[architecture/overview#src/gui/]]

#include "PhonePanel.hpp"

#include <QFont>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <nlohmann/json.hpp>

namespace {

using koocadcam::gui::ArrayTableSchema;
using koocadcam::gui::ColumnDescriptor;
using koocadcam::gui::FieldDescriptor;
using koocadcam::gui::GroupSchema;

// ── Phone spec schemas (mirror PhoneFrontModel::defaultSpec ranges) ──────────

const GroupSchema& baseSchema()
{
    static const GroupSchema s{ "base", "── Base ──", {
        { "width_mm",            "Width (mm)",      40.0, 120.0, 0.5, 1, 76.0 },
        { "height_mm",           "Height (mm)",     80.0, 220.0, 0.5, 1, 160.0 },
        { "thickness_mm",        "Thickness (mm)",   4.0,  16.0, 0.1, 2, 8.0 },
        { "initial_corner_r_mm", "Corner R (mm)",    0.0,  20.0, 0.1, 1, 8.0 },
    } };
    return s;
}

const GroupSchema& cornerRadiusSchema()
{
    static const GroupSchema s{ "corner_radius", "── Rim Fillets ──", {
        { "r_top_mm",  "Top rim R (mm)",    0.0, 5.0, 0.1, 2, 1.0 },
        { "r_side_mm", "Bottom rim R (mm)", 0.0, 5.0, 0.1, 2, 0.6 },
    } };
    return s;
}

const GroupSchema& displayPocketSchema()
{
    static const GroupSchema s{ "display_pocket", "── Display Pocket ──", {
        { "width_mm",    "Width (mm)",    20.0, 120.0, 0.5, 1, 70.0 },
        { "height_mm",   "Height (mm)",   40.0, 210.0, 0.5, 1, 152.0 },
        { "depth_mm",    "Depth (mm)",     0.1,   3.0, 0.1, 2, 0.6 },
        { "corner_r_mm", "Corner R (mm)",  0.0,  20.0, 0.1, 1, 6.0 },
        { "offset_x_mm", "Offset X (mm)", -30.0,  30.0, 0.5, 1, 0.0 },
        { "offset_y_mm", "Offset Y (mm)", -40.0,  40.0, 0.5, 1, 0.0 },
    } };
    return s;
}

const GroupSchema& portHoleSchema()
{
    static const GroupSchema s{ "port_hole", "── USB-C Port ──", {
        { "width_mm",    "Width (mm)",    4.0, 20.0, 0.1, 2, 8.6 },
        { "height_mm",   "Height (mm)",   2.0,  8.0, 0.1, 2, 3.2 },
        { "depth_mm",    "Depth (mm)",    2.0, 12.0, 0.5, 1, 6.0 },
        { "center_x_mm", "Center X (mm)", -30.0, 30.0, 0.5, 1, 0.0 },
        { "center_z_mm", "Center Z (mm)",  0.0, 16.0, 0.5, 1, 4.0 },
    } };
    return s;
}

const ArrayTableSchema& camerasSchema()
{
    static const ArrayTableSchema s{ "cameras", "── Rear Cameras ──", {
        { "offset_x_mm", "X (mm)",   -40.0, 40.0,  0.5, 1, -22.0 },
        { "offset_y_mm", "Y (mm)",  -100.0, 100.0, 0.5, 1, -55.0 },
        { "hole_dia_mm", "Dia (mm)",   1.0, 20.0,  0.1, 1,  8.0 },
        { "depth_mm",    "Depth (mm)", 0.5,  6.0,  0.1, 1,  2.0 },
    } };
    return s;
}

const ArrayTableSchema& decoRingsSchema()
{
    static const ArrayTableSchema s{ "camera_deco_rings", "── Camera Deco Rings ──", {
        { "offset_x_mm",  "X (mm)",      -40.0, 40.0,  0.5, 1, -22.0 },
        { "offset_y_mm",  "Y (mm)",     -100.0, 100.0, 0.5, 1, -55.0 },
        { "outer_dia_mm", "Outer (mm)",    2.0, 24.0,  0.1, 1, 10.0 },
        { "inner_dia_mm", "Inner (mm)",    1.0, 22.0,  0.1, 1,  9.0 },
        { "depth_mm",     "Depth (mm)",    0.1,  2.0,  0.1, 2,  0.3 },
    } };
    return s;
}

}  // namespace

namespace koocadcam::gui {

PhonePanel::PhonePanel(QWidget* parent)
    : QWidget(parent)
{
    m_debounce = new QTimer(this);
    m_debounce->setSingleShot(true);
    m_debounce->setInterval(200);
    connect(m_debounce, &QTimer::timeout, this, &PhonePanel::onDebounceTimeout);

    buildUi();
}

PhonePanel::~PhonePanel() = default;

void PhonePanel::buildUi()
{
    QVBoxLayout* vlay = new QVBoxLayout(this);
    vlay->setContentsMargins(6, 6, 6, 6);
    vlay->setSpacing(6);

    QLabel* header = new QLabel(tr("Phone Parameters"), this);
    QFont f = header->font();
    f.setBold(true);
    header->setFont(f);
    vlay->addWidget(header);

    // ── Groups (schema-driven) ───────────────────────────────────────────────
    for (const GroupSchema* gs : { &baseSchema(), &cornerRadiusSchema(),
                                   &displayPocketSchema(), &portHoleSchema() }) {
        GroupBinding gb;
        gb.schema = gs;
        vlay->addWidget(buildGroupWidgets(this, *gs, gb.controls));
        m_groups.push_back(std::move(gb));
    }
    for (auto& gb : m_groups)
        connectGroup(gb.controls, this, [this] { onAnyValueChanged(); });

    // ── Array tables (schema-driven) ─────────────────────────────────────────
    for (const ArrayTableSchema* as : { &camerasSchema(), &decoRingsSchema() }) {
        TableBinding tb;
        tb.schema = as;
        vlay->addWidget(buildArrayTable(this, *as, tb.table, tb.add, tb.remove));
        m_tables.push_back(tb);
    }
    for (const TableBinding& tb : m_tables) {
        QTableWidget* table = tb.table;
        const ArrayTableSchema* schema = tb.schema;
        connect(table, &QTableWidget::cellChanged, this,
                [this](int, int) { onAnyValueChanged(); });
        connect(tb.add, &QPushButton::clicked, this, [this, table, schema] {
            appendArrayRow(table, *schema, nlohmann::json::object());
            scheduleEmit();
        });
        connect(tb.remove, &QPushButton::clicked, this, [this, table] {
            const int rows = table->rowCount();
            if (rows > 0) { table->removeRow(rows - 1); scheduleEmit(); }
        });
    }

    m_rebuildButton = new QPushButton(tr("Rebuild Now"), this);
    connect(m_rebuildButton, &QPushButton::clicked, this, &PhonePanel::onRebuildClicked);
    vlay->addWidget(m_rebuildButton);
    vlay->addStretch();
}

void PhonePanel::onAnyValueChanged() { scheduleEmit(); }
void PhonePanel::scheduleEmit()      { m_debounce->start(); }
void PhonePanel::onDebounceTimeout() { emit specChanged(); }
void PhonePanel::onRebuildClicked()  { m_debounce->stop(); emit specChanged(); }

nlohmann::json PhonePanel::currentSpec() const
{
    // Start from the retained base spec so keys the panel has no widgets for
    // (side_buttons, product_name, schema_version) are preserved, then
    // overwrite every panel-owned group / table.
    nlohmann::json spec = m_baseSpec ? *m_baseSpec : nlohmann::json::object();
    spec["schema_version"] = "0.1.0";
    if (!spec.contains("product_name"))
        spec["product_name"] = "Custom Phone";

    for (const GroupBinding& gb : m_groups)
        readGroup(spec, *gb.schema, gb.controls);
    for (const TableBinding& tb : m_tables)
        spec[tb.schema->arrayKey] = readArrayTable(tb.table, *tb.schema);

    return spec;
}

void PhonePanel::setSpec(const nlohmann::json& spec)
{
    m_baseSpec = std::make_unique<nlohmann::json>(spec);
    for (GroupBinding& gb : m_groups)
        writeGroup(spec, *gb.schema, gb.controls);
    for (TableBinding& tb : m_tables) {
        const auto it = spec.find(tb.schema->arrayKey);
        writeArrayTable(tb.table, *tb.schema,
                        it != spec.end() ? *it : nlohmann::json::array());
    }
}

}  // namespace koocadcam::gui
