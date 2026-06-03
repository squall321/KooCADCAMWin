// @lat: [[process/test-strategy#skill round-trip]]
//
// reference_plane_offset — metadata-only datum plane offset from a source
// face along that face's outward normal.
//
// Cases (5):
//   1. ApplyDoesNotChangeShape  — workpiece TopoDS_Shape is unchanged.
//   2. SignatureRecordsKindAndDatumClass.
//   3. ResolvedOriginXYZ        — origin == faceCenter + n * offset_mm.
//   4. ValidateRejectsBadInputs — empty name / zero offset / non-planar face.
//   5. RecognizeMetadataReplay  — round-trips with confidence 1.0.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/reference_plane_offset.hpp"

#include <cmath>

using namespace koocadcam;

namespace {
// Find first planar face whose outward normal is closest to +Z (top of cuboid).
int findTopFaceId(const skill::Workpiece& wp)
{
    int best = -1;
    double bestZ = 0.5;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        const gp_Dir n = wp.faceNormal(i);
        if (n.Z() > bestZ) { bestZ = n.Z(); best = i; }
    }
    return best;
}
}  // namespace

// ─── 1. Shape unchanged ───────────────────────────────────────────────────
TEST(SkillReferencePlaneOffset, ApplyDoesNotChangeShape)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const int top = findTopFaceId(*stock);
    ASSERT_GE(top, 0);

    skill::reference_plane_offset::Input in;
    in.source_face_id = top;
    in.offset_mm      = 25.0;
    in.name           = "DATUM_A_OFF_25";

    auto out = skill::reference_plane_offset::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Same face count → workpiece is a clone of the original geometry.
    EXPECT_EQ(out.workpiece->faceCount(), stock->faceCount());
    EXPECT_EQ(out.workpiece->edgeCount(), stock->edgeCount());
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Signature kind + datum class ──────────────────────────────────────
TEST(SkillReferencePlaneOffset, SignatureRecordsKindAndDatumClass)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const int top = findTopFaceId(*stock);
    ASSERT_GE(top, 0);

    skill::reference_plane_offset::Input in;
    in.source_face_id = top;
    in.offset_mm      = 30.0;
    in.name           = "DATUM_TOP_OFF_30";

    auto out = skill::reference_plane_offset::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("reference_plane_offset"));
    EXPECT_TRUE(out.signature.pattern["is_reference"].get<bool>());
    EXPECT_EQ(out.signature.pattern["datum_class"].get<std::string>(),
              std::string("plane"));
    EXPECT_EQ(out.signature.pattern["named_id"].get<std::string>(),
              std::string("DATUM_TOP_OFF_30"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 3. Resolved origin = faceCenter + n * offset_mm (round-trip 1e-6) ────
TEST(SkillReferencePlaneOffset, ResolvedOriginXYZ)
{
    // 40 x 30 x 10 cuboid; top face center is at (20, 15, 10), normal +Z.
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const int top = findTopFaceId(*stock);
    ASSERT_GE(top, 0);

    const gp_Pnt c = stock->faceCenter(top);
    const gp_Dir n = stock->faceNormal(top);

    skill::reference_plane_offset::Input in;
    in.source_face_id = top;
    in.offset_mm      = 7.5;
    in.name           = "TOP_OFF_7p5";

    auto out = skill::reference_plane_offset::apply(*stock, in);
    const auto origin = out.signature.pattern["plane_origin_xyz"];
    const auto normal = out.signature.pattern["plane_normal_xyz"];

    EXPECT_NEAR(origin[0].get<double>(), c.X() + n.X() * 7.5, 1e-6);
    EXPECT_NEAR(origin[1].get<double>(), c.Y() + n.Y() * 7.5, 1e-6);
    EXPECT_NEAR(origin[2].get<double>(), c.Z() + n.Z() * 7.5, 1e-6);

    EXPECT_NEAR(normal[0].get<double>(), n.X(), 1e-6);
    EXPECT_NEAR(normal[1].get<double>(), n.Y(), 1e-6);
    EXPECT_NEAR(normal[2].get<double>(), n.Z(), 1e-6);
}

// ─── 4. DFM rejects bad inputs ────────────────────────────────────────────
TEST(SkillReferencePlaneOffset, ValidateRejectsBadInputs)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    // Empty name.
    {
        skill::reference_plane_offset::Input in;
        in.source_face_id = 0;
        in.offset_mm      = 5.0;
        in.name           = "";
        auto r = skill::reference_plane_offset::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::reference_plane_offset::apply(*stock, in),
                     skill::SkillError);
    }
    // Zero offset.
    {
        skill::reference_plane_offset::Input in;
        in.source_face_id = 0;
        in.offset_mm      = 0.0;
        in.name           = "X";
        auto r = skill::reference_plane_offset::validate(*stock, in);
        EXPECT_FALSE(r.passed);
    }
    // Face out of range.
    {
        skill::reference_plane_offset::Input in;
        in.source_face_id = stock->faceCount() + 99;
        in.offset_mm      = 5.0;
        in.name           = "X";
        auto r = skill::reference_plane_offset::validate(*stock, in);
        EXPECT_FALSE(r.passed);
    }
}

// ─── 5. Recognize replay ──────────────────────────────────────────────────
TEST(SkillReferencePlaneOffset, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const int top = findTopFaceId(*stock);
    ASSERT_GE(top, 0);

    skill::reference_plane_offset::Input in;
    in.source_face_id = top;
    in.offset_mm      = -12.5;          // negative offset is allowed
    in.name           = "BELOW_TOP_12p5";

    auto out   = skill::reference_plane_offset::apply(*stock, in);
    auto cands = skill::reference_plane_offset::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("reference_plane_offset"));
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_NEAR(cands[0].recovered_params["offset_mm"].get<double>(),
                -12.5, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["name"].get<std::string>(),
              std::string("BELOW_TOP_12p5"));
}
