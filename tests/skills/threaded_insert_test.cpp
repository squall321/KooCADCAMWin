// @lat: [[process/test-strategy#skill round-trip]]
//
// threaded_insert skill — covers pilot-table composition with drill_and_tap,
// length-vs-diameter DFM, material metadata, and history-replay recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_and_tap.hpp"
#include "skills/threaded_insert.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. Apply drills the install pilot ───────────────────────────────────
TEST(SkillThreadedInsert, ApplyDrillsInsertPilot)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 12.0);

    skill::threaded_insert::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 25.0;
    in.position_y_mm     = 25.0;
    in.thread_size       = "M4";          // tap_pilot 3.3 + 0.2 allowance = 3.5 mm
    in.insert_length_mm  = 8.0;            // ≥ 1.5 × 4 = 6 mm OK
    in.insert_material   = "stainless_316";

    auto out = skill::threaded_insert::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double pilotDia   = 3.5;
    const double drillDepth = 8.0 + 1.0;   // insert + 1 mm clearance
    const double approx = M_PI * (pilotDia/2.0) * (pilotDia/2.0) * drillDepth;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.05);

    EXPECT_EQ(out.signature.skill_id, std::string("threaded_insert"));
    EXPECT_NEAR(out.signature.params["pilot_dia_mm"].get<double>(), pilotDia, 1e-6);
    EXPECT_EQ  (out.signature.params["thread_size"].get<std::string>(), std::string("M4"));
    EXPECT_EQ  (out.signature.params["insert_material"].get<std::string>(),
                std::string("stainless_316"));
}

// ─── 2. Pilot-table composition reuses drill_and_tap chart + 0.2 mm offset ─
TEST(SkillThreadedInsert, PilotTableUsesDrillAndTap)
{
    // M3..M6 overlap with drill_and_tap chart → shared values + 0.2 mm.
    EXPECT_NEAR(skill::threaded_insert::insertPilotDiameter("M3"),
                skill::drill_and_tap::pilotDiameterFor("M3") + 0.2, 1e-9);
    EXPECT_NEAR(skill::threaded_insert::insertPilotDiameter("M5"),
                skill::drill_and_tap::pilotDiameterFor("M5") + 0.2, 1e-9);
    // M8 is not in drill_and_tap; threaded_insert keeps its own row.
    EXPECT_NEAR(skill::threaded_insert::insertPilotDiameter("M8"), 7.0, 1e-9);
    EXPECT_NEAR(skill::threaded_insert::threadNominalDia("M8"),  8.0, 1e-9);

    // M2 is in drill_and_tap chart but NOT in threaded_insert's whitelist.
    EXPECT_EQ(skill::threaded_insert::insertPilotDiameter("M2"), 0.0);

    // Reverse lookup.
    EXPECT_EQ(skill::threaded_insert::threadSizeForInsertPilot(
                  skill::threaded_insert::insertPilotDiameter("M6"), 0.05),
              std::string("M6"));
    EXPECT_TRUE(skill::threaded_insert::threadSizeForInsertPilot(7.7, 0.05).empty());
}

// ─── 3. DFM rejects sub-1.5×D insert length ──────────────────────────────
TEST(SkillThreadedInsert, ValidateRejectsShortInsert)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::threaded_insert::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.thread_size      = "M5";          // need ≥ 7.5 mm
    in.insert_length_mm = 5.0;            // too short
    in.insert_material  = "stainless_316";

    auto r = skill::threaded_insert::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-TI-LEN") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::threaded_insert::apply(*stock, in), skill::SkillError);
}

// ─── 4. DFM rejects unknown thread size ──────────────────────────────────
TEST(SkillThreadedInsert, ValidateRejectsUnknownSize)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::threaded_insert::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.thread_size      = "M99";
    in.insert_length_mm = 10.0;
    in.insert_material  = "stainless_316";

    auto r = skill::threaded_insert::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-TI-SIZE") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 5. Recognize replays insert metadata ────────────────────────────────
TEST(SkillThreadedInsert, RecognizeReplaysMetadata)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 15.0);

    skill::threaded_insert::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.thread_size      = "M6";
    in.insert_length_mm = 10.0;
    in.insert_material  = "stainless_316";

    auto out = skill::threaded_insert::apply(*stock, in);
    auto cands = skill::threaded_insert::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_GT(c.confidence, 0.9);
    EXPECT_EQ(c.recovered_params["thread_size"].get<std::string>(), std::string("M6"));
    EXPECT_NEAR(c.recovered_params["insert_length_mm"].get<double>(), 10.0, 1e-6);
    EXPECT_EQ(c.recovered_params["insert_material"].get<std::string>(),
              std::string("stainless_316"));
}

// ─── 6. Topology fallback matches standard install pilot ─────────────────
TEST(SkillThreadedInsert, TopologyFallbackOnSecondWorkpiece)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 12.0);

    skill::threaded_insert::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.thread_size      = "M5";
    in.insert_length_mm = 8.0;
    in.insert_material  = "stainless_316";
    auto out = skill::threaded_insert::apply(*stock, in);

    // Wrap the shape WITHOUT preserving feature history (mimics post-STEP).
    skill::Workpiece bare(out.workpiece->shape(), "aluminum_6061");
    auto cands = skill::threaded_insert::recognize(bare);
    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands.front().recovered_params["thread_size"].get<std::string>(),
              std::string("M5"));
}
