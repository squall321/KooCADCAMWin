// @lat: [[process/test-strategy#skill round-trip]]
//
// flange skill — 90° edge bend.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/flange.hpp"

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

// ── 1. Apply: x_max flange of width 5 mm on a 100×80×1.5 sheet ─────────
TEST(SkillFlange, ApplyBuildsValidSolid)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::flange::Input in;
    in.edge_selector   = skill::flange::EdgeSide::XMax;
    in.flange_width_mm = 5.0;
    in.bend_radius_mm  = 1.0;

    auto out = skill::flange::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_EQ(out.signature.skill_id, std::string("flange"));
    EXPECT_EQ(out.signature.pattern["edge_length_mm"].get<double>(), 80.0);
    EXPECT_NEAR(out.signature.pattern["flange_width_mm"].get<double>(), 5.0, 1e-6);
}

// ── 2. All 4 edge selectors produce a valid solid ──────────────────────
TEST(SkillFlange, EachEdgeSideWorks)
{
    using ES = skill::flange::EdgeSide;
    for (ES s : { ES::XMin, ES::XMax, ES::YMin, ES::YMax }) {
        auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);
        skill::flange::Input in;
        in.edge_selector   = s;
        in.flange_width_mm = 4.0;
        in.bend_radius_mm  = 1.0;
        auto out = skill::flange::apply(*stock, in);
        EXPECT_FALSE(out.workpiece->shape().IsNull())
            << "edge_side " << skill::flange::edgeSideToString(s);
    }
}

// ── 3. DFM-F-MIN-R: too-small bend radius rejected ─────────────────────
TEST(SkillFlange, ValidateRejectsTooSmallRadius)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::flange::Input in;
    in.edge_selector   = skill::flange::EdgeSide::XMax;
    in.flange_width_mm = 5.0;
    in.bend_radius_mm  = 0.3;       // < 0.5 × 1.5 = 0.75

    auto r = skill::flange::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-F-MIN-R") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::flange::apply(*stock, in), skill::SkillError);
}

// ── 4. DFM-F-WIDTH: negative flange_width rejected ─────────────────────
TEST(SkillFlange, ValidateRejectsBadWidth)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::flange::Input in;
    in.edge_selector   = skill::flange::EdgeSide::XMax;
    in.flange_width_mm = 0.0;
    in.bend_radius_mm  = 1.0;

    auto r = skill::flange::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-F-WIDTH") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 5. Recognize finds the flange ──────────────────────────────────────
TEST(SkillFlange, RecognizeFindsFlange)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::flange::Input in;
    in.edge_selector   = skill::flange::EdgeSide::XMax;
    in.flange_width_mm = 5.0;
    in.bend_radius_mm  = 1.0;

    auto out = skill::flange::apply(*stock, in);
    auto cands = skill::flange::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("flange"));
    EXPECT_NEAR(c.recovered_params["bend_radius_mm"].get<double>(), 1.0, 0.05);
    EXPECT_GE(c.confidence, 0.7);
}
