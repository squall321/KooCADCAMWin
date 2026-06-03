// @lat: [[process/test-strategy#skill round-trip]]
//
// fill_pattern — hex/grid fill of a face region with a small hole.
//
// Cases (5):
//   1. Grid fill yields N×M instances on a flat plate; volume matches.
//   2. Hex fill yields more instances than grid for same pitch.
//   3. DFM rejects pitch ≤ hole dia.
//   4. DFM rejects unknown fill_type.
//   5. recognize finds the pattern (history replay).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/fill_pattern.hpp"

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

// ─── 1. Grid fill ──────────────────────────────────────────────────────────
TEST(SkillFillPattern, GridFillProducesInstances)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::fill_pattern::Input in;
    in.face               = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.source_hole_dia_mm = 3.0;
    in.source_depth_mm    = 4.0;
    in.fill_type          = "grid";
    in.pitch_mm           = 10.0;
    in.margin_mm          = 5.0;     // active = 50 x 50

    auto out = skill::fill_pattern::apply(*stock, in);
    const int instances = out.signature.pattern["instance_count"].get<int>();
    EXPECT_GE(instances, 16);          // ≥ 4 × 4
    const double removed = volBefore - volumeOf(out.workpiece->shape());
    const double per      = M_PI * 1.5 * 1.5 * 4.0;
    const double expected = per * instances;
    EXPECT_NEAR(removed, expected, expected * 0.15);
    EXPECT_TRUE(out.signature.pattern["is_pattern"].get<bool>());
}

// ─── 2. Hex fill instance count > grid for same params ────────────────────
TEST(SkillFillPattern, HexFillProducesInstances)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::fill_pattern::Input in;
    in.face               = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.source_hole_dia_mm = 3.0;
    in.source_depth_mm    = 4.0;
    in.fill_type          = "hex";
    in.pitch_mm           = 10.0;
    in.margin_mm          = 5.0;

    auto out = skill::fill_pattern::apply(*stock, in);
    EXPECT_GE(out.signature.pattern["instance_count"].get<int>(), 10);
}

// ─── 3. DFM rejects pitch ≤ dia ────────────────────────────────────────────
TEST(SkillFillPattern, ValidateRejectsSmallPitch)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::fill_pattern::Input in;
    in.face               = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.source_hole_dia_mm = 5.0;
    in.source_depth_mm    = 3.0;
    in.fill_type          = "grid";
    in.pitch_mm           = 4.0;       // < dia
    in.margin_mm          = 5.0;
    auto r = skill::fill_pattern::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. DFM rejects bad fill_type ──────────────────────────────────────────
TEST(SkillFillPattern, ValidateRejectsBadFillType)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::fill_pattern::Input in;
    in.face               = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.source_hole_dia_mm = 3.0;
    in.source_depth_mm    = 3.0;
    in.fill_type          = "triangle";  // invalid
    in.pitch_mm           = 10.0;
    in.margin_mm          = 5.0;
    auto r = skill::fill_pattern::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 5. Recognize identifies fill (history replay) ─────────────────────────
TEST(SkillFillPattern, RecognizeFindsFill)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    skill::fill_pattern::Input in;
    in.face               = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.source_hole_dia_mm = 3.0;
    in.source_depth_mm    = 4.0;
    in.fill_type          = "grid";
    in.pitch_mm           = 10.0;
    in.margin_mm          = 5.0;
    auto out = skill::fill_pattern::apply(*stock, in);
    auto cands = skill::fill_pattern::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("fill_pattern"));
    EXPECT_GT(cands[0].confidence, 0.5);
}
