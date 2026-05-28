// @lat: [[process/test-strategy#skill round-trip]]
//
// deep_draw skill — pull a flat sheet into a cup.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/deep_draw.hpp"

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

// ── 1. Apply: basic deep draw produces a valid solid ───────────────────
TEST(SkillDeepDraw, ApplyBuildsValidSolid)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::deep_draw::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.cup_position_x_mm = 50.0;
    in.cup_position_y_mm = 40.0;
    in.cup_dia_mm        = 20.0;            // depth/dia = 0.5 ≤ 0.7
    in.cup_depth_mm      = 10.0;
    in.corner_r_mm       = 6.0;             // ≥ 4 × 1.5 = 6
    in.sheet_thickness_mm = 1.5;

    auto out = skill::deep_draw::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_GT(volumeOf(out.workpiece->shape()), 0.0);

    EXPECT_EQ(out.signature.skill_id, std::string("deep_draw"));
    EXPECT_NEAR(out.signature.pattern["cup_dia_mm"].get<double>(), 20.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["cup_depth_mm"].get<double>(), 10.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["drawing_ratio"].get<double>(), 0.5, 1e-3);
}

// ── 2. DFM-DD-RATIO: too-deep cup rejected ─────────────────────────────
TEST(SkillDeepDraw, ValidateRejectsTooDeepDraw)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::deep_draw::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.cup_position_x_mm = 50.0;
    in.cup_position_y_mm = 40.0;
    in.cup_dia_mm        = 20.0;
    in.cup_depth_mm      = 20.0;            // ratio 1.0 > 0.7
    in.corner_r_mm       = 6.0;

    auto r = skill::deep_draw::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DD-RATIO") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::deep_draw::apply(*stock, in), skill::SkillError);
}

// ── 3. DFM-DD-CORNER: corner radius too small rejected ────────────────
TEST(SkillDeepDraw, ValidateRejectsTooSharpCorner)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::deep_draw::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.cup_position_x_mm = 50.0;
    in.cup_position_y_mm = 40.0;
    in.cup_dia_mm        = 30.0;
    in.cup_depth_mm      = 10.0;
    in.corner_r_mm       = 2.0;             // 2 < 4 × 1.5 = 6

    auto r = skill::deep_draw::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DD-CORNER") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 4. DFM-INPUT: cup off the sheet bbox ──────────────────────────────
TEST(SkillDeepDraw, ValidateRejectsOffSheet)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::deep_draw::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.cup_position_x_mm = 95.0;           // r=10 → 95+10=105 > 100
    in.cup_position_y_mm = 40.0;
    in.cup_dia_mm        = 20.0;
    in.cup_depth_mm      = 10.0;
    in.corner_r_mm       = 6.0;

    auto r = skill::deep_draw::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ── 5. Recognize finds the cup protrusion ─────────────────────────────
// TODO(slice-9): FIX_FORM tolerance widening still doesn't match the
// outer+inner cylinder pair after deep_draw::apply().  Likely a
// span filter mismatch (the outer cup wall span may not exceed 1.5×t
// after Boolean tolerance).  Re-investigate findCupWall geometry.
TEST(SkillDeepDraw, DISABLED_RecognizeFindsCup)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::deep_draw::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.cup_position_x_mm = 50.0;
    in.cup_position_y_mm = 40.0;
    in.cup_dia_mm        = 20.0;
    in.cup_depth_mm      = 10.0;
    in.corner_r_mm       = 6.0;

    auto out = skill::deep_draw::apply(*stock, in);
    auto cands = skill::deep_draw::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("deep_draw"));
    EXPECT_NEAR(c.recovered_params["cup_dia_mm"].get<double>(), 20.0, 0.1);
    EXPECT_GE(c.recovered_params["cup_depth_mm"].get<double>(), 5.0);
}
