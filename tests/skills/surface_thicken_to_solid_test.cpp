// @lat: [[process/test-strategy#skill round-trip]]
//
// surface_thicken_to_solid — fuse a thickened skin onto the workpiece.
//
// Cases (5):
//   1. ApplyProducesNonNull — result shape exists, volume grows.
//   2. ValidateRejectsZeroThickness.
//   3. ValidateRejectsBadDirection.
//   4. SignatureRecordsKindAndThickness.
//   5. RecognizeMetadataReplay.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/surface_thicken_to_solid.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. apply produces non-null and grows volume ──────────────────────────
TEST(SkillSurfaceThickenToSolid, ApplyProducesNonNullAndGrowsVolume)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::surface_thicken_to_solid::Input in;
    in.source_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.thickness_mm  = 2.0;
    in.direction     = "outward";

    auto out = skill::surface_thicken_to_solid::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double volAfter = volumeOf(out.workpiece->shape());
    EXPECT_GT(volAfter, volBefore);                       // skin added mass
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. DFM rejects zero thickness ────────────────────────────────────────
TEST(SkillSurfaceThickenToSolid, ValidateRejectsZeroThickness)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    skill::surface_thicken_to_solid::Input in;
    in.source_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.thickness_mm  = 0.0;
    auto r = skill::surface_thicken_to_solid::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::surface_thicken_to_solid::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. DFM rejects bad direction ─────────────────────────────────────────
TEST(SkillSurfaceThickenToSolid, ValidateRejectsBadDirection)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    skill::surface_thicken_to_solid::Input in;
    in.source_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.thickness_mm  = 1.5;
    in.direction     = "sideways";    // invalid
    auto r = skill::surface_thicken_to_solid::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DIRECTION") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}

// ─── 4. signature records kind + thickness ────────────────────────────────
TEST(SkillSurfaceThickenToSolid, SignatureRecordsKindAndThickness)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    skill::surface_thicken_to_solid::Input in;
    in.source_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.thickness_mm  = 1.5;
    in.direction     = "outward";
    auto out = skill::surface_thicken_to_solid::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id,
              std::string("surface_thicken_to_solid"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("surface_thicken_to_solid"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["source_face_count"].get<int>(), 1);
    EXPECT_NEAR(out.signature.pattern["thickness_mm"].get<double>(),
                1.5, 1e-9);
}

// ─── 5. recognize replay (confidence 1.0) ─────────────────────────────────
TEST(SkillSurfaceThickenToSolid, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    skill::surface_thicken_to_solid::Input in;
    in.source_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.thickness_mm  = 2.0;
    in.direction     = "outward";
    auto out = skill::surface_thicken_to_solid::apply(*stock, in);
    auto cands = skill::surface_thicken_to_solid::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id,
              std::string("surface_thicken_to_solid"));
    EXPECT_DOUBLE_EQ(cands.front().confidence, 1.0);
}
