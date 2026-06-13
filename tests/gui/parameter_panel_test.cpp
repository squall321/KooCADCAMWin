// @lat: [[architecture/overview#src/gui/]]
//
// ParameterPanel round-trip preservation (headless, Qt "offscreen" platform).
//
// The panel only has widgets for steps 1-6.  Before the pass-through fix,
// currentSpec() rebuilt the spec from those widgets alone, so a
//   load full spec  →  edit one field  →  save / rebuild
// silently DROPPED steps 7-10 (speaker_grille / rear_sensors / lugs /
// secondary_fillets).  These tests lock the fix: currentSpec() now starts
// from the last setSpec() and overwrites only the panel-owned keys, so the
// hidden steps survive — and the round-tripped spec still builds + passes DFM.

#include <gtest/gtest.h>

#include <QApplication>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QEventLoop>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimer>
#include <QtGlobal>

#include "gui/ParameterPanel.hpp"
#include "engine/WatchFrontModel.hpp"

#include <TopoDS_Shape.hxx>

#include <nlohmann/json.hpp>

#include <vector>

using namespace koocadcam;

// 1. Every step the panel cannot edit survives setSpec → currentSpec verbatim.
TEST(ParameterPanelPreservation, HiddenStepsSurviveRoundTrip)
{
    gui::ParameterPanel panel;
    const nlohmann::json full = engine::WatchFrontModel::defaultSpec();
    panel.setSpec(full);

    const nlohmann::json out = panel.currentSpec();

    for (const char* key : { "speaker_grille", "rear_sensors",
                             "lugs", "secondary_fillets" }) {
        ASSERT_TRUE(out.contains(key)) << "currentSpec() dropped '" << key << "'";
        EXPECT_EQ(out[key], full[key]) << "'" << key << "' was mutated";
    }
}

// 2. Panel-owned keys still reflect widget state after the round trip.
TEST(ParameterPanelPreservation, OwnedKeysReflectWidgets)
{
    gui::ParameterPanel panel;
    panel.setSpec(engine::WatchFrontModel::defaultSpec());

    const nlohmann::json out = panel.currentSpec();
    ASSERT_TRUE(out.contains("base"));
    ASSERT_TRUE(out.contains("bezel"));
    // defaultSpec is a 44 mm round watch — the widgets loaded those values.
    EXPECT_NEAR(out["base"].value("diameter_mm", 0.0), 44.0, 1e-6);
    EXPECT_NEAR(out["base"].value("thickness_mm", 0.0), 10.0, 1e-6);
}

// 3. End-to-end: the round-tripped spec still builds AND passes DFM.  This
// proves the preserved steps 7-10 are not just present but complete & valid
// (ties the GUI fix to the runDFM manufacturability gate).
TEST(ParameterPanelPreservation, RoundTrippedSpecBuildsAndPassesDFM)
{
    gui::ParameterPanel panel;
    panel.setSpec(engine::WatchFrontModel::defaultSpec());
    const nlohmann::json out = panel.currentSpec();

    std::vector<engine::BuildWarning> warnings;
    TopoDS_Shape shape = engine::WatchFrontModel::buildAll(out, warnings);
    ASSERT_FALSE(shape.IsNull()) << "round-tripped spec failed to build";

    auto report = engine::WatchFrontModel::runDFM(shape, out);
    EXPECT_TRUE(report.passed)
        << "round-tripped spec must still pass DFM (7-10 preserved + valid)";
}

// 4. Unchecking crown removes a crown that arrived via the base spec.
TEST(ParameterPanelPreservation, DisablingCrownErasesItFromBaseSpec)
{
    gui::ParameterPanel panel;
    nlohmann::json full = engine::WatchFrontModel::defaultSpec();
    ASSERT_TRUE(full.contains("crown_cavity")) << "fixture precondition";
    panel.setSpec(full);                 // crown checkbox now checked

    // Round-trip with crown enabled keeps it.
    EXPECT_TRUE(panel.currentSpec().contains("crown_cavity"));

    // Build a spec WITHOUT crown, load it → checkbox unchecks → crown gone.
    nlohmann::json noCrown = full;
    noCrown.erase("crown_cavity");
    panel.setSpec(noCrown);
    EXPECT_FALSE(panel.currentSpec().contains("crown_cavity"))
        << "crown must not reappear once the loaded spec omits it";
    // ...but the other hidden steps are still preserved.
    EXPECT_TRUE(panel.currentSpec().contains("lugs"));
}

// ── B6.2/B6.3 schema-driven editing tests ────────────────────────────────────
// Factory-built spin boxes carry objectName "<groupKey>.<fieldKey>" and the
// enable checkboxes "<groupKey>.enabled", so tests reach them via findChild()
// without any test-only panel API.

// 5. Step 7 (speaker_grille) is now EDITABLE: changing a grille widget shows
// up in currentSpec(), untouched grille fields keep their loaded values, and
// the still-hidden steps (rear_sensors / lugs) survive unchanged.
TEST(ParameterPanelEditing, SpeakerGrilleEditableRoundTrip)
{
    gui::ParameterPanel panel;
    const nlohmann::json full = engine::WatchFrontModel::defaultSpec();
    panel.setSpec(full);

    auto* angle = panel.findChild<QDoubleSpinBox*>("speaker_grille.angle_deg");
    ASSERT_NE(angle, nullptr) << "factory did not name the grille angle spin";
    angle->setValue(90.0);

    auto* rows = panel.findChild<QSpinBox*>("speaker_grille.rows");
    ASSERT_NE(rows, nullptr) << "isInt field must be a QSpinBox";
    rows->setValue(3);

    const nlohmann::json out = panel.currentSpec();
    ASSERT_TRUE(out.contains("speaker_grille"));
    EXPECT_NEAR(out["speaker_grille"].value("angle_deg", 0.0), 90.0, 1e-9);
    EXPECT_EQ(out["speaker_grille"]["rows"].get<int>(), 3);
    // Untouched grille fields keep the values loaded from the spec.
    EXPECT_EQ(out["speaker_grille"]["cols"], full["speaker_grille"]["cols"]);
    EXPECT_EQ(out["speaker_grille"]["hole_dia_mm"],
              full["speaker_grille"]["hole_dia_mm"]);
    // Other hidden steps are still preserved verbatim.
    EXPECT_EQ(out["lugs"],         full["lugs"]);
    EXPECT_EQ(out["rear_sensors"], full["rear_sensors"]);
}

// 6. Step 10 (secondary_fillets) enable checkbox: uncheck → key erased from
// currentSpec() even though it arrived via the base spec; recheck → key
// restored with the widget value.
TEST(ParameterPanelEditing, SecondaryFilletsToggle)
{
    gui::ParameterPanel panel;
    const nlohmann::json full = engine::WatchFrontModel::defaultSpec();
    ASSERT_TRUE(full.contains("secondary_fillets")) << "fixture precondition";
    panel.setSpec(full);
    EXPECT_TRUE(panel.currentSpec().contains("secondary_fillets"));

    auto* enable = panel.findChild<QCheckBox*>("secondary_fillets.enabled");
    ASSERT_NE(enable, nullptr);

    enable->setChecked(false);
    EXPECT_FALSE(panel.currentSpec().contains("secondary_fillets"))
        << "unchecking must erase the key (crown-cavity semantics)";

    enable->setChecked(true);
    const nlohmann::json out = panel.currentSpec();
    ASSERT_TRUE(out.contains("secondary_fillets"))
        << "rechecking must restore the key";
    EXPECT_NEAR(out["secondary_fillets"].value("r_mm", -1.0),
                full["secondary_fillets"].value("r_mm", -2.0), 1e-9);
}

// 7. Signal regression (B6 verifier ask): editing a grille spin box fires the
// debounced specChanged exactly ONCE within the 200 ms window — rapid edits
// coalesce, and nothing fires synchronously.  parameter_panel_test does not
// link Qt6::Test (tests/CMakeLists.txt is frozen), so instead of QSignalSpy
// we count emissions with a plain connect and pump the event loop with
// QEventLoop + QTimer::singleShot.
TEST(ParameterPanelEditing, SpecChangedDebouncedOnceAfterGrilleEdit)
{
    gui::ParameterPanel panel;
    panel.setSpec(engine::WatchFrontModel::defaultSpec());

    int emissions = 0;
    QObject::connect(&panel, &gui::ParameterPanel::specChanged,
                     &panel, [&emissions] { ++emissions; });

    auto* angle = panel.findChild<QDoubleSpinBox*>("speaker_grille.angle_deg");
    ASSERT_NE(angle, nullptr);

    // Three rapid edits — the debounce must coalesce them.
    angle->setValue(10.0);
    angle->setValue(20.0);
    angle->setValue(30.0);
    EXPECT_EQ(emissions, 0) << "specChanged must not fire synchronously";

    // Pump the event loop well past the 200 ms debounce window.
    QEventLoop loop;
    QTimer::singleShot(500, &loop, &QEventLoop::quit);
    loop.exec();

    EXPECT_EQ(emissions, 1)
        << "debounced specChanged must fire exactly once after grille edits";
}

// ── B6.3 array-table editing tests (rear_sensors / lugs) ─────────────────────
// The generic array tables carry objectName "<arrayKey>.table"/".add"/".remove"
// so tests reach them via findChild() without any test-only panel API.  Cells
// are plain numeric text edited in place; setItem(...) mimics a user edit.

// 8. Step 9 (lugs) is now EDITABLE: the table loads one row per default lug,
// editing a cell shows up in currentSpec(), and an unrelated step
// (speaker_grille) survives the round trip unchanged.
TEST(ParameterPanelEditing, LugsTableEditableRoundTrip)
{
    gui::ParameterPanel panel;
    const nlohmann::json full = engine::WatchFrontModel::defaultSpec();
    ASSERT_TRUE(full.contains("lugs") && full["lugs"].is_array());
    panel.setSpec(full);

    auto* table = panel.findChild<QTableWidget*>("lugs.table");
    ASSERT_NE(table, nullptr) << "factory did not name the lugs table";
    ASSERT_EQ(table->rowCount(), static_cast<int>(full["lugs"].size()))
        << "row count must match the loaded lugs array size";

    // lugs columns: angle_deg(0) length_mm(1) width_mm(2) thickness_mm(3) pin(4)
    table->setItem(0, 1, new QTableWidgetItem(QStringLiteral("7.50")));

    const nlohmann::json out = panel.currentSpec();
    ASSERT_TRUE(out.contains("lugs") && out["lugs"].is_array());
    ASSERT_GE(out["lugs"].size(), 1u);
    EXPECT_NEAR(out["lugs"][0].value("length_mm", 0.0), 7.5, 1e-9);
    // Other steps preserved verbatim.
    EXPECT_EQ(out["speaker_grille"], full["speaker_grille"]);
    EXPECT_EQ(out["rear_sensors"],   full["rear_sensors"]);
}

// 9. Step 8 (rear_sensors) Add/Remove buttons grow/shrink the array.
TEST(ParameterPanelEditing, RearSensorsAddRemoveRow)
{
    gui::ParameterPanel panel;
    const nlohmann::json full = engine::WatchFrontModel::defaultSpec();
    panel.setSpec(full);

    const std::size_t base = panel.currentSpec()["rear_sensors"].size();
    ASSERT_GT(base, 0u) << "fixture precondition: default has rear sensors";

    auto* add    = panel.findChild<QPushButton*>("rear_sensors.add");
    auto* remove = panel.findChild<QPushButton*>("rear_sensors.remove");
    ASSERT_NE(add, nullptr);
    ASSERT_NE(remove, nullptr);

    add->click();
    EXPECT_EQ(panel.currentSpec()["rear_sensors"].size(), base + 1);

    remove->click();
    EXPECT_EQ(panel.currentSpec()["rear_sensors"].size(), base);
}

// 10. lugs.pin_hole_dia_mm is optional: blanking (or zeroing) the cell drops
// the key from that lug's object entirely (engine → lug with no pin hole).
TEST(ParameterPanelEditing, LugsOptionalPinHoleOmitted)
{
    gui::ParameterPanel panel;
    const nlohmann::json full = engine::WatchFrontModel::defaultSpec();
    panel.setSpec(full);

    auto* table = panel.findChild<QTableWidget*>("lugs.table");
    ASSERT_NE(table, nullptr);
    ASSERT_GE(table->rowCount(), 1);

    // Default lugs DO carry a pin hole — verify the precondition, then clear it.
    EXPECT_TRUE(panel.currentSpec()["lugs"][0].contains("pin_hole_dia_mm"));

    // pin_hole_dia_mm is column 4; blank it (a user clearing the cell).
    table->setItem(0, 4, new QTableWidgetItem(QString()));

    const nlohmann::json out = panel.currentSpec();
    EXPECT_FALSE(out["lugs"][0].contains("pin_hole_dia_mm"))
        << "a blank optional cell must omit the key";
    // The required columns are still present.
    EXPECT_TRUE(out["lugs"][0].contains("angle_deg"));
    EXPECT_TRUE(out["lugs"][0].contains("thickness_mm"));
}

int main(int argc, char** argv)
{
    // Headless: render with the offscreen platform so no display is needed.
    qputenv("QT_QPA_PLATFORM", QByteArrayLiteral("offscreen"));
    QApplication app(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
