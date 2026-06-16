// @lat: [[architecture/overview#src/gui/]]
//
// PhonePanel round-trip + editing (headless, Qt "offscreen" platform).
//
// PhonePanel is fully schema-driven: groups (base / corner_radius /
// display_pocket / port_hole) + array tables (cameras / camera_deco_rings).
// side_buttons (a string-enum field the numeric factory does not cover) is
// preserved as pass-through.  These tests lock the round-trip + that the
// round-tripped spec still builds and passes DFM via PhoneFrontModel.

#include <gtest/gtest.h>

#include <QApplication>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QtGlobal>

#include "gui/PhonePanel.hpp"
#include "engine/PhoneFrontModel.hpp"

#include <TopoDS_Shape.hxx>

#include <nlohmann/json.hpp>

#include <vector>

using namespace koocadcam;

// 1. side_buttons (the one pass-through section) survives the round trip.
TEST(PhonePanel, PassThroughSurvivesRoundTrip)
{
    gui::PhonePanel panel;
    const nlohmann::json full = engine::PhoneFrontModel::defaultSpec();
    ASSERT_TRUE(full.contains("side_buttons"));
    panel.setSpec(full);

    const nlohmann::json out = panel.currentSpec();
    ASSERT_TRUE(out.contains("side_buttons")) << "side_buttons was dropped";
    EXPECT_EQ(out["side_buttons"], full["side_buttons"]) << "side_buttons mutated";
}

// 2. Panel-owned keys reflect the loaded widget state.
TEST(PhonePanel, OwnedKeysReflectWidgets)
{
    gui::PhonePanel panel;
    panel.setSpec(engine::PhoneFrontModel::defaultSpec());

    const nlohmann::json out = panel.currentSpec();
    ASSERT_TRUE(out.contains("base"));
    EXPECT_NEAR(out["base"].value("width_mm", 0.0),  76.0,  1e-6);
    EXPECT_NEAR(out["base"].value("height_mm", 0.0), 160.0, 1e-6);
    EXPECT_NEAR(out["base"].value("thickness_mm", 0.0), 8.0, 1e-6);
    ASSERT_TRUE(out.contains("cameras") && out["cameras"].is_array());
    EXPECT_EQ(out["cameras"].size(), engine::PhoneFrontModel::defaultSpec()["cameras"].size());
}

// 3. End-to-end: the round-tripped spec still builds AND passes DFM.
TEST(PhonePanel, RoundTrippedSpecBuildsAndPassesDFM)
{
    gui::PhonePanel panel;
    panel.setSpec(engine::PhoneFrontModel::defaultSpec());
    const nlohmann::json out = panel.currentSpec();

    std::vector<engine::BuildWarning> warnings;
    TopoDS_Shape shape = engine::PhoneFrontModel::buildAll(out, warnings);
    ASSERT_FALSE(shape.IsNull()) << "round-tripped phone spec failed to build";

    auto report = engine::PhoneFrontModel::runDFM(shape, out);
    EXPECT_TRUE(report.passed)
        << "round-tripped phone spec must pass DFM";
}

// 4. Editing a camera cell shows up in currentSpec; side_buttons preserved.
TEST(PhonePanel, CamerasTableEditableRoundTrip)
{
    gui::PhonePanel panel;
    const nlohmann::json full = engine::PhoneFrontModel::defaultSpec();
    panel.setSpec(full);

    auto* table = panel.findChild<QTableWidget*>("cameras.table");
    ASSERT_NE(table, nullptr) << "factory did not name the cameras table";
    ASSERT_EQ(table->rowCount(), static_cast<int>(full["cameras"].size()));

    // cameras columns: offset_x(0) offset_y(1) hole_dia(2) depth(3)
    table->setItem(0, 2, new QTableWidgetItem(QStringLiteral("7.50")));

    const nlohmann::json out = panel.currentSpec();
    ASSERT_TRUE(out["cameras"].is_array() && !out["cameras"].empty());
    EXPECT_NEAR(out["cameras"][0].value("hole_dia_mm", 0.0), 7.5, 1e-9);
    EXPECT_EQ(out["side_buttons"], full["side_buttons"]);   // unrelated section intact
}

// 5. Add/Remove grows and shrinks the deco-ring array.
TEST(PhonePanel, DecoRingsAddRemoveRow)
{
    gui::PhonePanel panel;
    panel.setSpec(engine::PhoneFrontModel::defaultSpec());

    const std::size_t base = panel.currentSpec()["camera_deco_rings"].size();
    ASSERT_GT(base, 0u);

    auto* add    = panel.findChild<QPushButton*>("camera_deco_rings.add");
    auto* remove = panel.findChild<QPushButton*>("camera_deco_rings.remove");
    ASSERT_NE(add, nullptr);
    ASSERT_NE(remove, nullptr);

    add->click();
    EXPECT_EQ(panel.currentSpec()["camera_deco_rings"].size(), base + 1);
    remove->click();
    EXPECT_EQ(panel.currentSpec()["camera_deco_rings"].size(), base);
}

int main(int argc, char** argv)
{
    qputenv("QT_QPA_PLATFORM", QByteArrayLiteral("offscreen"));
    QApplication app(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
