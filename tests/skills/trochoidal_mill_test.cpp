// @lat: [[process/test-strategy#skill round-trip]]
//
// trochoidal_mill — high-feed pocket strategy.  Geometric result is
// identical to mill_rect_pocket; we test the metadata-stamping behaviour.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/trochoidal_mill.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. Apply produces a pocket of the right volume ───────────────────────
TEST(SkillTrochoidalMill, ApplyCreatesPocket)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);

    skill::trochoidal_mill::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm  = 30.0;
    in.center_y_mm  = 20.0;
    in.length_mm    = 20.0;
    in.width_mm     = 10.0;
    in.depth_mm     = 3.0;
    in.corner_r_mm  = 1.0;
    in.tool_dia_mm  = 6.0;
    in.stepover_mm  = 0.5;     // tool_dia * 0.083 — within limit

    auto out = skill::trochoidal_mill::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double approx  = 20.0 * 10.0 * 3.0;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.10);

    EXPECT_EQ(out.signature.skill_id, std::string("trochoidal_mill"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("trochoidal_mill"));
}

// ── 2. DFM-002: tool_dia < 1.0 mm rejected ───────────────────────────────
TEST(SkillTrochoidalMill, ValidateRejectsSubMinTool)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::trochoidal_mill::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm  = 25.0;
    in.center_y_mm  = 25.0;
    in.length_mm    = 10.0;
    in.width_mm     = 6.0;
    in.depth_mm     = 2.0;
    in.corner_r_mm  = 0.5;
    in.tool_dia_mm  = 0.8;      // < 1.0 — error
    in.stepover_mm  = 0.05;

    auto r = skill::trochoidal_mill::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-002") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::trochoidal_mill::apply(*stock, in), skill::SkillError);
}

// ── 3. DFM-TROCH-SO: stepover > tool_dia × 0.3 rejected ──────────────────
TEST(SkillTrochoidalMill, ValidateRejectsHighStepover)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::trochoidal_mill::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm  = 25.0;
    in.center_y_mm  = 25.0;
    in.length_mm    = 12.0;
    in.width_mm     = 8.0;
    in.depth_mm     = 2.0;
    in.corner_r_mm  = 1.0;
    in.tool_dia_mm  = 6.0;
    in.stepover_mm  = 3.0;      // = tool_dia × 0.5 → error (max 1.8)

    auto r = skill::trochoidal_mill::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-TROCH-SO") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 4. Signature carries strategy metadata + high-feed tooling ───────────
TEST(SkillTrochoidalMill, SignatureMarksHighFeedStrategy)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    skill::trochoidal_mill::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm  = 30.0;
    in.center_y_mm  = 20.0;
    in.length_mm    = 20.0;
    in.width_mm     = 10.0;
    in.depth_mm     = 3.0;
    in.corner_r_mm  = 1.0;
    in.tool_dia_mm  = 6.0;
    in.stepover_mm  = 0.6;       // tool_dia × 0.1 boundary OK

    auto out = skill::trochoidal_mill::apply(*stock, in);

    EXPECT_EQ(out.signature.tooling.tool_type, std::string("end_mill_high_feed"));
    EXPECT_GT(out.signature.tooling.cutting_speed_sfm, 600.0);
    EXPECT_GT(out.signature.tooling.feed_per_tooth_mm, 0.025);
    EXPECT_NEAR(out.signature.tooling.tool_dia_mm, 6.0, 1e-6);
    ASSERT_TRUE(out.signature.tooling.extra.contains("strategy"));
    EXPECT_EQ(out.signature.tooling.extra["strategy"].get<std::string>(),
              std::string("trochoidal"));
    EXPECT_NEAR(out.signature.tooling.extra["stepover_mm"].get<double>(),
                0.6, 1e-6);
}

// ── 5. Recognize finds trochoidal via feature history ────────────────────
TEST(SkillTrochoidalMill, RecognizeFindsViaFeatureHistory)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    skill::trochoidal_mill::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm  = 30.0;
    in.center_y_mm  = 20.0;
    in.length_mm    = 20.0;
    in.width_mm     = 10.0;
    in.depth_mm     = 3.0;
    in.corner_r_mm  = 1.0;
    in.tool_dia_mm  = 6.0;
    in.stepover_mm  = 0.5;

    auto out  = skill::trochoidal_mill::apply(*stock, in);
    auto cands = skill::trochoidal_mill::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands[0].skill_id, std::string("trochoidal_mill"));
    EXPECT_GE(cands[0].confidence, 0.80);    // history-backed → high conf
    EXPECT_NEAR(cands[0].recovered_params["tool_dia_mm"].get<double>(),
                6.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["stepover_mm"].get<double>(),
                0.5, 1e-6);
}

// ── 6. Default stepover = tool_dia × 0.1 when caller leaves it 0 ─────────
TEST(SkillTrochoidalMill, DefaultStepoverAutoFilled)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    skill::trochoidal_mill::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm  = 30.0;
    in.center_y_mm  = 20.0;
    in.length_mm    = 20.0;
    in.width_mm     = 10.0;
    in.depth_mm     = 2.0;
    in.corner_r_mm  = 1.0;
    in.tool_dia_mm  = 5.0;
    in.stepover_mm  = 0.0;        // caller left it default → 5.0 * 0.1

    auto out = skill::trochoidal_mill::apply(*stock, in);
    EXPECT_NEAR(out.signature.params["stepover_mm"].get<double>(),
                0.5, 1e-6);
}
