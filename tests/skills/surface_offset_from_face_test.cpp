// @lat: [[process/test-strategy#skill round-trip]]
//
// surface_offset_from_face — translate a face along its normal as a new Face.
//
// Cases (5):
//   1. ApplyProducesNonNull — original shape preserved, offset face added.
//   2. ValidateRejectsZeroOffset.
//   3. SignatureRecordsKindAndOffset.
//   4. OffsetFaceAreaMatchesSourceArea (translation preserves area).
//   5. RecognizeMetadataReplay.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/surface_offset_from_face.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

// ─── 1. apply produces non-null with offset face attached ─────────────────
TEST(SkillSurfaceOffsetFromFace, ApplyProducesNonNull)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    skill::surface_offset_from_face::Input in;
    in.source_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.offset_mm   = 2.0;
    auto out = skill::surface_offset_from_face::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. DFM rejects zero offset ───────────────────────────────────────────
TEST(SkillSurfaceOffsetFromFace, ValidateRejectsZeroOffset)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    skill::surface_offset_from_face::Input in;
    in.source_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.offset_mm   = 0.0;
    auto r = skill::surface_offset_from_face::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::surface_offset_from_face::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. signature records kind & offset ───────────────────────────────────
TEST(SkillSurfaceOffsetFromFace, SignatureRecordsKindAndOffset)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    skill::surface_offset_from_face::Input in;
    in.source_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.offset_mm   = 2.5;
    auto out = skill::surface_offset_from_face::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("surface_offset_from_face"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["source_face_count"].get<int>(), 1);
    EXPECT_NEAR(out.signature.pattern["offset_mm"].get<double>(),
                2.5, 1e-9);
}

// ─── 4. offset face area equals source face area (pure translation) ───────
TEST(SkillSurfaceOffsetFromFace, OffsetFaceAreaMatchesSource)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    skill::surface_offset_from_face::Input in;
    in.source_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.offset_mm   = 3.0;
    auto out = skill::surface_offset_from_face::apply(*stock, in);
    // Expected source face area = 40 × 30 = 1200.
    EXPECT_NEAR(out.signature.pattern["derived_area"].get<double>(),
                1200.0, 1.0);
}

// ─── 5. recognize replay ──────────────────────────────────────────────────
TEST(SkillSurfaceOffsetFromFace, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    skill::surface_offset_from_face::Input in;
    in.source_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.offset_mm   = 2.0;
    auto out = skill::surface_offset_from_face::apply(*stock, in);
    auto cands = skill::surface_offset_from_face::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id,
              std::string("surface_offset_from_face"));
    EXPECT_DOUBLE_EQ(cands.front().confidence, 1.0);
    EXPECT_NEAR(cands.front().recovered_params["offset_mm"].get<double>(),
                2.0, 1e-6);
}
