// @lat: [[process/test-strategy#skill round-trip]]
//
// hydroforming skill — high-pressure fluid forming of a cup-shaped feature.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/hydroforming.hpp"

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

// ── 1. Apply: forms a cup similar to deep_draw, tagged hydroforming ────
TEST(SkillHydroforming, ApplyBuildsValidSolid)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::hydroforming::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.cup_position_x_mm = 50.0;
    in.cup_position_y_mm = 40.0;
    in.cup_dia_mm        = 25.0;
    in.cup_depth_mm      = 20.0;            // ratio 0.8 — allowed (≤ 1.0)
    in.corner_r_mm       = 4.0;             // ≥ 2 × 1.5 = 3.0
    in.sheet_thickness_mm = 1.5;
    in.pressure_mpa      = 200.0;

    auto out = skill::hydroforming::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_GT(volumeOf(out.workpiece->shape()), 0.0);
    EXPECT_EQ(out.signature.skill_id, std::string("hydroforming"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(), "hydroforming");
    EXPECT_NEAR(out.signature.pattern["pressure_mpa"].get<double>(), 200.0, 1e-6);
}

// ── 2. DFM-HF-PRESS: out-of-range pressure rejected ───────────────────
TEST(SkillHydroforming, ValidateRejectsBadPressure)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::hydroforming::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.cup_position_x_mm = 50.0;
    in.cup_position_y_mm = 40.0;
    in.cup_dia_mm        = 20.0;
    in.cup_depth_mm      = 10.0;
    in.corner_r_mm       = 4.0;
    in.pressure_mpa      = 700.0;          // > 600

    auto r = skill::hydroforming::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-HF-PRESS") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::hydroforming::apply(*stock, in), skill::SkillError);
}

// ── 3. DFM-HF-RATIO: too-deep draw rejected ───────────────────────────
TEST(SkillHydroforming, ValidateRejectsTooDeepDraw)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::hydroforming::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.cup_position_x_mm = 50.0;
    in.cup_position_y_mm = 40.0;
    in.cup_dia_mm        = 20.0;
    in.cup_depth_mm      = 25.0;          // ratio 1.25 > 1.0
    in.corner_r_mm       = 4.0;
    in.pressure_mpa      = 200.0;

    auto r = skill::hydroforming::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-HF-RATIO") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 4. Hydroforming allows deeper draws than deep_draw ─────────────────
TEST(SkillHydroforming, AllowsHigherDrawingRatioThanDeepDraw)
{
    // ratio 0.85 — too deep for deep_draw (> 0.7), allowed for hydroform.
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::hydroforming::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.cup_position_x_mm = 50.0;
    in.cup_position_y_mm = 40.0;
    in.cup_dia_mm        = 20.0;
    in.cup_depth_mm      = 17.0;          // ratio 0.85
    in.corner_r_mm       = 4.0;
    in.pressure_mpa      = 200.0;

    auto r = skill::hydroforming::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    auto out = skill::hydroforming::apply(*stock, in);
    EXPECT_FALSE(out.workpiece->shape().IsNull());
}

// ── 5. Recognize: same topology as deep_draw but tagged hydroforming ───
// TODO(slice-9): same findCupWall issue as deep_draw::RecognizeFindsCup.
TEST(SkillHydroforming, DISABLED_RecognizeFindsCup)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::hydroforming::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.cup_position_x_mm = 50.0;
    in.cup_position_y_mm = 40.0;
    in.cup_dia_mm        = 20.0;
    in.cup_depth_mm      = 16.0;          // ratio 0.8
    in.corner_r_mm       = 4.0;
    in.pressure_mpa      = 200.0;

    auto out = skill::hydroforming::apply(*stock, in);
    auto cands = skill::hydroforming::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("hydroforming"));
    EXPECT_NEAR(c.recovered_params["cup_dia_mm"].get<double>(), 20.0, 0.1);
    // Metadata-aware: confidence boosted because of prior hydroforming step.
    EXPECT_GE(c.confidence, 0.7);
}
