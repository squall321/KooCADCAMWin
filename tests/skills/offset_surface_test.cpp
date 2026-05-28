// @lat: [[process/test-strategy#skill round-trip]]
//
// offset_surface — parallel surface at a constant distance from a face.
//
// Cases (6):
//   1. ApplyProducesNonNull shape.
//   2. ValidateRejectsBadInput — zero offset.
//   3. SignatureRecordsKind — pattern.kind == "offset_surface".
//   4. RecognizeMetadataReplay — history replay confidence 1.0.
//   5. ValidateRejectsTooLargeOffset — |offset| ≥ 0.5 × min-extent.
//   6. ApplyRecordsOffsetMagnitudeInSignature.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/offset_surface.hpp"

#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

// ─── 1. ApplyProducesNonNull shape ────────────────────────────────────────
TEST(SkillOffsetSurface, ApplyProducesNonNull)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::offset_surface::Input in;
    in.target_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.offset_mm = 1.0;

    auto out = skill::offset_surface::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. ValidateRejectsBadInput (zero offset) ─────────────────────────────
TEST(SkillOffsetSurface, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::offset_surface::Input in;
    in.target_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.offset_mm = 0.0;

    auto r = skill::offset_surface::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);

    EXPECT_THROW(skill::offset_surface::apply(*stock, in), skill::SkillError);
}

// ─── 3. SignatureRecordsKind ──────────────────────────────────────────────
TEST(SkillOffsetSurface, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::offset_surface::Input in;
    in.target_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.offset_mm = 1.5;

    auto out = skill::offset_surface::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("offset_surface"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("offset_surface"));
}

// ─── 4. RecognizeMetadataReplay (confidence 1.0) ──────────────────────────
TEST(SkillOffsetSurface, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::offset_surface::Input in;
    in.target_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.offset_mm = 2.0;

    auto out = skill::offset_surface::apply(*stock, in);
    auto cands = skill::offset_surface::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("offset_surface"));
    EXPECT_DOUBLE_EQ(c.confidence, 1.0);
    EXPECT_NEAR(c.recovered_params["offset_mm"].get<double>(), 2.0, 1e-6);
}

// ─── 5. ValidateRejectsTooLargeOffset (≥ 0.5 × min-extent) ────────────────
TEST(SkillOffsetSurface, ValidateRejectsTooLargeOffset)
{
    // Min extent is 10 (Z); 0.5 × 10 = 5; so offset = 6 is over the limit.
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::offset_surface::Input in;
    in.target_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.offset_mm = 6.0;

    auto r = skill::offset_surface::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-OFFSET-MAG") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}

// ─── 6. ApplyRecordsOffsetMagnitudeInSignature ────────────────────────────
TEST(SkillOffsetSurface, ApplyRecordsOffsetMagnitudeInSignature)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::offset_surface::Input in;
    in.target_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.offset_mm = -1.25;

    auto out = skill::offset_surface::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["offset_mm"].get<double>(),
                -1.25, 1e-6);
    EXPECT_NEAR(out.signature.params["offset_mm"].get<double>(),
                -1.25, 1e-6);
}
