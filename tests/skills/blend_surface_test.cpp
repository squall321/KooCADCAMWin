// @lat: [[process/test-strategy#skill round-trip]]
//
// blend_surface — fill the gap between two faces.
//
// Cases (6):
//   1. ApplyProducesNonNull shape.
//   2. ValidateRejectsBadInput — same face_a == face_b.
//   3. SignatureRecordsKind — pattern.kind == "blend_surface".
//   4. RecognizeMetadataReplay — history replay confidence 1.0.
//   5. ValidateAcceptsDistinctFaces — passes when two distinct faces selected.
//   6. ApplyPreservesOriginalFaces — the new shape still contains the two
//      source faces' areas (i.e. the blend is purely additive).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/blend_surface.hpp"

#include <TopoDS_Shape.hxx>

using namespace koocadcam;

// ─── 1. ApplyProducesNonNull shape ────────────────────────────────────────
TEST(SkillBlendSurface, ApplyProducesNonNull)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::blend_surface::Input in;
    // Top face (largest +Z) and bottom face (largest -Z).
    in.face_a = skill::FaceByNormal{ gp_Dir(0, 0,  1), 5.0, "largest" };
    in.face_b = skill::FaceByNormal{ gp_Dir(0, 0, -1), 5.0, "largest" };
    in.continuity = 0;

    auto out = skill::blend_surface::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. ValidateRejectsBadInput (same face) ───────────────────────────────
TEST(SkillBlendSurface, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::blend_surface::Input in;
    // Both datums resolve to the same top face → must error.
    in.face_a = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.face_b = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.continuity = 1;

    auto r = skill::blend_surface::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BLEND-SAME") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);

    EXPECT_THROW(skill::blend_surface::apply(*stock, in), skill::SkillError);
}

// ─── 3. SignatureRecordsKind ──────────────────────────────────────────────
TEST(SkillBlendSurface, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::blend_surface::Input in;
    in.face_a = skill::FaceByNormal{ gp_Dir(0, 0,  1), 5.0, "largest" };
    in.face_b = skill::FaceByNormal{ gp_Dir(0, 0, -1), 5.0, "largest" };

    auto out = skill::blend_surface::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("blend_surface"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("blend_surface"));
}

// ─── 4. RecognizeMetadataReplay (confidence 1.0) ──────────────────────────
TEST(SkillBlendSurface, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::blend_surface::Input in;
    in.face_a = skill::FaceByNormal{ gp_Dir(0, 0,  1), 5.0, "largest" };
    in.face_b = skill::FaceByNormal{ gp_Dir(0, 0, -1), 5.0, "largest" };
    in.continuity = 1;

    auto out = skill::blend_surface::apply(*stock, in);
    auto cands = skill::blend_surface::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("blend_surface"));
    EXPECT_DOUBLE_EQ(c.confidence, 1.0);
    EXPECT_EQ(c.recovered_params["continuity"].get<int>(), 1);
}

// ─── 5. ValidateAcceptsDistinctFaces ──────────────────────────────────────
TEST(SkillBlendSurface, ValidateAcceptsDistinctFaces)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::blend_surface::Input in;
    in.face_a = skill::FaceByNormal{ gp_Dir(0, 0,  1), 5.0, "largest" };
    in.face_b = skill::FaceByNormal{ gp_Dir(0, 0, -1), 5.0, "largest" };
    in.continuity = 0;

    auto r = skill::blend_surface::validate(*stock, in);
    EXPECT_TRUE(r.passed) << "two distinct faces should pass DFM";
}

// ─── 6. ApplyPreservesOriginalFaces ───────────────────────────────────────
TEST(SkillBlendSurface, ApplyPreservesOriginalFaces)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const int origFaceCount = stock->faceCount();

    skill::blend_surface::Input in;
    in.face_a = skill::FaceByNormal{ gp_Dir(0, 0,  1), 5.0, "largest" };
    in.face_b = skill::FaceByNormal{ gp_Dir(0, 0, -1), 5.0, "largest" };

    auto out = skill::blend_surface::apply(*stock, in);

    // The new shape should contain at least the original faces (additive
    // skill — Boolean fuse never removes existing geometry without a cutter).
    EXPECT_GE(out.workpiece->faceCount(), origFaceCount);
}
