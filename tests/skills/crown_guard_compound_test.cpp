// @lat: [[process/test-strategy#compound crown_guard_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/crown_guard_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::crown_guard_compound::Input goodInput() {
    skill::crown_guard_compound::Input in;
    in.case_side_face        = skill::FaceByNormal{ gp_Dir(1, 0, 0) };
    in.case_axis             = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    in.case_outer_radius_mm  = 20.0;
    in.crown_axis_origin     = gp_Pnt(20.0, 0.0, 4.0);   // 3 o'clock, mid-height
    in.crown_axis_dir        = gp_Dir(1, 0, 0);          // radial outward
    in.guard_height_mm       = 4.0;
    in.guard_thickness_mm    = 1.5;
    in.guard_radial_extent_mm = 2.5;
    in.guard_separation_mm   = 6.0;
    in.guard_chamfer_mm      = 0.4;
    return in;
}
}  // namespace

// ─── 1. Apply ADDS material (fuse) → volume increases ─────────────────────
TEST(SkillCrownGuard, ApplyAddsTwoGuardsByFusing)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::crown_guard_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Two guards × box volume.  The 0.02 mm radial bias means a tiny sliver
    // overlaps the case; net added ≈ 2 × radial_extent × thickness × height.
    const double perGuardVol = 2.5 * 1.5 * 4.0;
    const double added = volumeOf(out.workpiece->shape()) - volumeOf(stock->shape());

    // Fuse should add roughly 2x perGuardVol (allowing OCCT tolerance).
    EXPECT_GT(added, perGuardVol * 1.5);   // at least 75 % of 2 guards
    EXPECT_LT(added, perGuardVol * 2.2);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects guard_thickness < 0.4 mm ────────────────────────
TEST(SkillCrownGuard, ValidateRejectsThinGuard)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    in.guard_thickness_mm = 0.3;   // < 0.4
    auto r = skill::crown_guard_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GUARD-THICK") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::crown_guard_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects bad separation ──────────────────────────────────
TEST(SkillCrownGuard, ValidateRejectsBadSeparation)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    in.guard_separation_mm = 1.0;   // < 2 × 1.5 = 3.0
    auto r = skill::crown_guard_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GUARD-SEP") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records compound + watch + protrusion_pair ─────────────
TEST(SkillCrownGuard, SignatureRecordsProtrusionPair)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::crown_guard_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("crown_guard_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_TRUE(out.signature.pattern.at("is_watch_feature").get<bool>());
    EXPECT_TRUE(out.signature.pattern.at("protrusion_pair").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("guard_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 2);
}

// ─── 5. Recognize via metadata replay ────────────────────────────────────
TEST(SkillCrownGuard, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::crown_guard_compound::apply(*stock, in);

    auto cands = skill::crown_guard_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_NEAR(cands[0].recovered_params.at("guard_height_mm").get<double>(),
                4.0, 1e-6);
}
