// @lat: [[process/test-strategy#skill round-trip]]
//
// sketch_driven_pattern — N holes at user-specified (x,y) positions.
//
// Cases (5):
//   1. 4 specified positions remove ~4 × per-hole volume.
//   2. Empty positions vector fails DFM.
//   3. DFM rejects overlapping positions.
//   4. DFM rejects diameter < 0.8 mm.
//   5. recognize identifies the pattern (history replay).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/sketch_driven_pattern.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. 4 positions removes ~4 × hole volume ───────────────────────────────
TEST(SkillSketchDrivenPattern, FourPositionsRemoveExpected)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::sketch_driven_pattern::Input in;
    in.hole_dia_mm   = 4.0;
    in.hole_depth_mm = 5.0;
    in.positions     = { { -20.0, -20.0 }, { 20.0, -20.0 },
                         {  20.0,  20.0 }, { -20.0, 20.0 } };
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };

    auto out = skill::sketch_driven_pattern::apply(*stock, in);
    const double removed = volBefore - volumeOf(out.workpiece->shape());
    const double per      = M_PI * 2.0 * 2.0 * 5.0;
    const double expected = 4.0 * per;
    EXPECT_NEAR(removed, expected, expected * 0.15);
    EXPECT_EQ(out.signature.pattern["instance_count"].get<int>(), 4);
    EXPECT_TRUE(out.signature.pattern["is_pattern"].get<bool>());
}

// ─── 2. Empty positions → DFM fails ────────────────────────────────────────
TEST(SkillSketchDrivenPattern, ValidateRejectsEmpty)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 10.0);
    skill::sketch_driven_pattern::Input in;
    in.hole_dia_mm   = 4.0;
    in.hole_depth_mm = 5.0;
    in.positions.clear();
    auto r = skill::sketch_driven_pattern::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 3. Overlapping positions → DFM-PATT-5 ─────────────────────────────────
TEST(SkillSketchDrivenPattern, ValidateRejectsOverlap)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 10.0);
    skill::sketch_driven_pattern::Input in;
    in.hole_dia_mm   = 5.0;
    in.hole_depth_mm = 4.0;
    in.positions     = { { 0.0, 0.0 }, { 2.0, 0.0 } };  // sep = 2 < 5
    auto r = skill::sketch_driven_pattern::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings) if (f.code == "DFM-PATT-5") found = true;
    EXPECT_TRUE(found);
}

// ─── 4. Sub-min diameter → DFM-002 ─────────────────────────────────────────
TEST(SkillSketchDrivenPattern, ValidateRejectsSubMinDia)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 10.0);
    skill::sketch_driven_pattern::Input in;
    in.hole_dia_mm   = 0.5;       // < 0.8
    in.hole_depth_mm = 3.0;
    in.positions     = { { 0.0, 0.0 } };
    auto r = skill::sketch_driven_pattern::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 5. recognize identifies pattern (history replay) ─────────────────────
TEST(SkillSketchDrivenPattern, RecognizeIdentifiesPattern)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 10.0);
    skill::sketch_driven_pattern::Input in;
    in.hole_dia_mm   = 4.0;
    in.hole_depth_mm = 5.0;
    in.positions     = { { -20.0, -20.0 }, { 20.0, -20.0 },
                         {  20.0,  20.0 }, { -20.0, 20.0 } };
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    auto out = skill::sketch_driven_pattern::apply(*stock, in);
    auto cands = skill::sketch_driven_pattern::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("sketch_driven_pattern"));
    EXPECT_GT(cands[0].confidence, 0.5);
}
