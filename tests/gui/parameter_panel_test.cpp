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

int main(int argc, char** argv)
{
    // Headless: render with the offscreen platform so no display is needed.
    qputenv("QT_QPA_PLATFORM", QByteArrayLiteral("offscreen"));
    QApplication app(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
