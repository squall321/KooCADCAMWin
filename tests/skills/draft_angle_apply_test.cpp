// @lat: [[process/test-strategy#skill round-trip]]
//
// draft_angle_apply — metadata-only marker that a vertical face needs a 1-5°
// draft taper for mould release.  Shape is INTENTIONALLY unchanged.
//
// Cases:
//   1. ApplyRemovesMaterial : here interpreted as "apply produces a new
//      workpiece" — shape is identical (no material change is correct), but
//      a new Workpiece with the feature signature must be returned.
//   2. ValidateRejectsBadInput : draft_deg outside [1.0, 5.0] → DFM-DRAFT-RANGE.
//   3. SignatureRecordsKind + face_id + draft_deg.
//   4. RecognizeMetadataReplay : recognize() replays from history.
//   5. RejectsHorizontalFace (specific) : top face (n.Z ≈ 1) → DFM-DRAFT-VERT.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/draft_angle_apply.hpp"

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

// ─── 1. Apply (no material change, but new workpiece returned) ────────────
TEST(SkillDraftAngleApply, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);
    const double stockVol = volumeOf(stock->shape());

    skill::draft_angle_apply::Input in;
    // Side face — normal is ±X (vertical for draft purposes).
    in.target_face = skill::FaceByNormal{ gp_Dir(1, 0, 0), 5.0, "largest" };
    in.draft_deg   = 2.0;

    auto out = skill::draft_angle_apply::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Metadata-only by design — volume is unchanged.
    const double newVol = volumeOf(out.workpiece->shape());
    EXPECT_NEAR(newVol, stockVol, 1e-6);
    // …but the feature record exists on the new workpiece.
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate rejects out-of-range draft ───────────────────────────────
TEST(SkillDraftAngleApply, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);

    skill::draft_angle_apply::Input in;
    in.target_face = skill::FaceByNormal{ gp_Dir(1, 0, 0) };
    in.draft_deg   = 7.0;     // > 5

    auto r = skill::draft_angle_apply::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DRAFT-RANGE") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);

    EXPECT_THROW(skill::draft_angle_apply::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillDraftAngleApply, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);

    skill::draft_angle_apply::Input in;
    in.target_face = skill::FaceByNormal{ gp_Dir(0, 1, 0) };   // +Y vertical face
    in.draft_deg   = 3.0;

    auto out = skill::draft_angle_apply::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("draft_angle_apply"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("draft_angle_apply"));
    EXPECT_NEAR(out.signature.pattern["draft_deg"].get<double>(), 3.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["applied_geometry"].get<bool>());
    // face_id must be a non-negative integer.
    EXPECT_GE(out.signature.pattern["face_id"].get<int>(), 0);
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillDraftAngleApply, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);

    skill::draft_angle_apply::Input in;
    in.target_face = skill::FaceByNormal{ gp_Dir(1, 0, 0) };
    in.draft_deg   = 2.5;

    auto out = skill::draft_angle_apply::apply(*stock, in);
    auto cands = skill::draft_angle_apply::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    const auto& c = cands.front();
    EXPECT_GT(c.confidence, 0.7);
    EXPECT_NEAR(c.recovered_params["draft_deg"].get<double>(), 2.5, 1e-9);
    EXPECT_GE(c.recovered_params["face_id"].get<int>(), 0);
}

// ─── 5. Rejects horizontal face (specific) ────────────────────────────────
TEST(SkillDraftAngleApply, RejectsHorizontalFace)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);

    skill::draft_angle_apply::Input in;
    // Top face normal = +Z → |n.Z|=1 > 0.2 → DFM-DRAFT-VERT error.
    in.target_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.draft_deg   = 2.0;

    auto r = skill::draft_angle_apply::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DRAFT-VERT" && f.severity == "error") {
            sawCode = true; break;
        }
    EXPECT_TRUE(sawCode);
}
