// @lat: [[process/test-strategy#skill round-trip]]
//
// adaptive_clearing — constant-engagement modern roughing.  Geometric
// result identical to mill_rect_pocket; we test metadata.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/adaptive_clearing.hpp"

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

// ── 1. Apply creates a pocket of the right volume ────────────────────────
TEST(SkillAdaptiveClearing, ApplyCreatesPocket)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);

    skill::adaptive_clearing::Input in;
    in.entry_face              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm             = 30.0;
    in.center_y_mm             = 20.0;
    in.length_mm               = 20.0;
    in.width_mm                = 10.0;
    in.depth_mm                = 3.0;
    in.corner_r_mm             = 1.0;
    in.tool_dia_mm             = 6.0;
    in.stepover_mm             = 0.5;
    in.optimal_engagement_pct  = 30.0;

    auto out = skill::adaptive_clearing::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double approx = 20.0 * 10.0 * 3.0;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.10);

    EXPECT_EQ(out.signature.skill_id, std::string("adaptive_clearing"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("adaptive_clearing"));
}

// ── 2. DFM-002: sub-min tool rejected ────────────────────────────────────
TEST(SkillAdaptiveClearing, ValidateRejectsSubMinTool)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::adaptive_clearing::Input in;
    in.entry_face              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm             = 25.0;
    in.center_y_mm             = 25.0;
    in.length_mm               = 10.0;
    in.width_mm                = 6.0;
    in.depth_mm                = 2.0;
    in.corner_r_mm             = 0.5;
    in.tool_dia_mm             = 0.5;
    in.stepover_mm             = 0.05;
    in.optimal_engagement_pct  = 30.0;

    auto r = skill::adaptive_clearing::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-002") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 3. DFM-ADAPT-ENG: engagement % out of range rejected ─────────────────
TEST(SkillAdaptiveClearing, ValidateRejectsBadEngagementPct)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    skill::adaptive_clearing::Input in;
    in.entry_face              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm             = 30.0;
    in.center_y_mm             = 20.0;
    in.length_mm               = 20.0;
    in.width_mm                = 10.0;
    in.depth_mm                = 3.0;
    in.corner_r_mm             = 1.0;
    in.tool_dia_mm             = 6.0;
    in.stepover_mm             = 0.5;
    in.optimal_engagement_pct  = 80.0;   // > 60 → error

    auto r = skill::adaptive_clearing::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ADAPT-ENG") { found = true; break; }
    EXPECT_TRUE(found);

    in.optimal_engagement_pct = 5.0;     // < 10 → error
    auto r2 = skill::adaptive_clearing::validate(*stock, in);
    EXPECT_FALSE(r2.passed);
}

// ── 4. Signature carries adaptive tooling + engagement % ─────────────────
TEST(SkillAdaptiveClearing, SignatureMarksAdaptiveStrategy)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    skill::adaptive_clearing::Input in;
    in.entry_face              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm             = 30.0;
    in.center_y_mm             = 20.0;
    in.length_mm               = 20.0;
    in.width_mm                = 10.0;
    in.depth_mm                = 3.0;
    in.corner_r_mm             = 1.0;
    in.tool_dia_mm             = 6.0;
    in.stepover_mm             = 0.6;
    in.optimal_engagement_pct  = 35.0;

    auto out = skill::adaptive_clearing::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("end_mill_adaptive"));
    // 2× SFM
    EXPECT_GE(out.signature.tooling.cutting_speed_sfm, 1100.0);
    // 3× feed/tooth
    EXPECT_GT(out.signature.tooling.feed_per_tooth_mm, 0.07);
    ASSERT_TRUE(out.signature.tooling.extra.contains("optimal_engagement_pct"));
    EXPECT_NEAR(
        out.signature.tooling.extra["optimal_engagement_pct"].get<double>(),
        35.0, 1e-6);
}

// ── 5. Recognize via feature history (high conf) ─────────────────────────
TEST(SkillAdaptiveClearing, RecognizeFindsViaHistory)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    skill::adaptive_clearing::Input in;
    in.entry_face              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm             = 30.0;
    in.center_y_mm             = 20.0;
    in.length_mm               = 20.0;
    in.width_mm                = 10.0;
    in.depth_mm                = 3.0;
    in.corner_r_mm             = 1.0;
    in.tool_dia_mm             = 6.0;
    in.stepover_mm             = 0.5;
    in.optimal_engagement_pct  = 30.0;

    auto out = skill::adaptive_clearing::apply(*stock, in);
    auto cands = skill::adaptive_clearing::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands[0].skill_id, std::string("adaptive_clearing"));
    EXPECT_GE(cands[0].confidence, 0.80);
    EXPECT_NEAR(
        cands[0].recovered_params["optimal_engagement_pct"].get<double>(),
        30.0, 1e-6);
}
