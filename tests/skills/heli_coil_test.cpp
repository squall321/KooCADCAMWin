// @lat: [[process/test-strategy#skill round-trip]]
//
// heli_coil skill — verifies that geometry composes from threaded_insert
// while signature is re-stamped, and that coil_pitch / coil_count DFM
// checks fire correctly.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/heli_coil.hpp"

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

// ─── 1. Apply: same geometry as threaded_insert with heli_coil signature ─
TEST(SkillHeliCoil, ApplyMakesInstallHole)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 14.0);

    skill::heli_coil::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 25.0;
    in.position_y_mm     = 25.0;
    in.thread_size       = "M5";
    in.insert_length_mm  = 8.0;
    in.coil_pitch_mm     = 0.8;          // ISO coarse for M5
    in.coil_count        = 1;
    in.insert_material   = "stainless_316";

    auto out = skill::heli_coil::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Hole geometry matches threaded_insert(M5): pilot = 4.4 mm, depth = 9 mm.
    const double pilot = 4.4;
    const double depth = 9.0;
    const double approx  = M_PI * (pilot/2.0) * (pilot/2.0) * depth;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.05);

    EXPECT_EQ(out.signature.skill_id, std::string("heli_coil"));
    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(), std::string("heli_coil"));
    EXPECT_NEAR(out.signature.params["coil_pitch_mm"].get<double>(), 0.8, 1e-9);
    EXPECT_EQ  (out.signature.params["coil_count"].get<int>(),       1);
}

// ─── 2. Default-pitch lookup ─────────────────────────────────────────────
TEST(SkillHeliCoil, DefaultPitchLookup)
{
    EXPECT_NEAR(skill::heli_coil::defaultCoarsePitch("M3"), 0.5,  1e-9);
    EXPECT_NEAR(skill::heli_coil::defaultCoarsePitch("M4"), 0.7,  1e-9);
    EXPECT_NEAR(skill::heli_coil::defaultCoarsePitch("M5"), 0.8,  1e-9);
    EXPECT_NEAR(skill::heli_coil::defaultCoarsePitch("M6"), 1.0,  1e-9);
    EXPECT_NEAR(skill::heli_coil::defaultCoarsePitch("M8"), 1.25, 1e-9);
    EXPECT_EQ  (skill::heli_coil::defaultCoarsePitch("M99"), 0.0);
}

// ─── 3. DFM rejects zero coil_pitch ──────────────────────────────────────
TEST(SkillHeliCoil, ValidateRejectsZeroPitch)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::heli_coil::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.thread_size      = "M4";
    in.insert_length_mm = 6.0;
    in.coil_pitch_mm    = 0.0;             // invalid
    in.coil_count       = 1;
    in.insert_material  = "stainless_316";

    auto r = skill::heli_coil::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-HC-PITCH") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::heli_coil::apply(*stock, in), skill::SkillError);
}

// ─── 4. DFM rejects coil_count > 3 ───────────────────────────────────────
TEST(SkillHeliCoil, ValidateRejectsTooManyCoils)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::heli_coil::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.thread_size      = "M4";
    in.insert_length_mm = 6.0;
    in.coil_pitch_mm    = 0.7;
    in.coil_count       = 4;               // > 3
    in.insert_material  = "stainless_316";

    auto r = skill::heli_coil::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-HC-COUNT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 5. Recognize replays heli_coil from history ─────────────────────────
TEST(SkillHeliCoil, RecognizeReplaysCoilMetadata)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 12.0);

    skill::heli_coil::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.thread_size      = "M6";
    in.insert_length_mm = 10.0;
    in.coil_pitch_mm    = 1.0;
    in.coil_count       = 2;
    in.insert_material  = "stainless_316";

    auto out = skill::heli_coil::apply(*stock, in);
    auto cands = skill::heli_coil::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_GT(c.confidence, 0.9);
    EXPECT_EQ(c.recovered_params["thread_size"].get<std::string>(), std::string("M6"));
    EXPECT_NEAR(c.recovered_params["coil_pitch_mm"].get<double>(), 1.0, 1e-9);
    EXPECT_EQ  (c.recovered_params["coil_count"].get<int>(),       2);
}

// ─── 6. Topology fallback tags heli_coil with low confidence ─────────────
TEST(SkillHeliCoil, TopologyFallbackLowConfidence)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 12.0);

    skill::heli_coil::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.thread_size      = "M4";
    in.insert_length_mm = 6.0;
    in.coil_pitch_mm    = 0.7;
    in.coil_count       = 1;
    in.insert_material  = "stainless_316";
    auto out = skill::heli_coil::apply(*stock, in);

    // Discard history → forces the topology fallback.
    skill::Workpiece bare(out.workpiece->shape(), "aluminum_6061");
    auto cands = skill::heli_coil::recognize(bare);
    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_LT(c.confidence, 0.5);                          // low because B-rep is ambiguous
    EXPECT_EQ(c.recovered_params["thread_size"].get<std::string>(), std::string("M4"));
    // Default pitch backfill kicks in.
    EXPECT_NEAR(c.recovered_params["coil_pitch_mm"].get<double>(), 0.7, 1e-9);
}
