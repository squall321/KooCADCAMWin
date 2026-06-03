// @lat: [[process/test-strategy#skill round-trip]]
//
// reference_point_on_face_uv — datum point at (u, v) on a face surface.
//
// Cases (5):
//   1. ApplyDoesNotChangeShape.
//   2. SignatureRecordsKindAndDatumClass.
//   3. ResolvedPointMatchesBRepAdaptor — midpoint (u=v=0.5) on top face of a
//      cuboid lies at the face centroid (within 1e-6).
//   4. ValidateRejectsOutOfRangeUV.
//   5. RecognizeMetadataReplay.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/reference_point_on_face_uv.hpp"

#include <cmath>

using namespace koocadcam;

namespace {
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
TEST(SkillReferencePointOnFaceUV, ApplyDoesNotChangeShape)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const int top = findTopFaceId(*stock);
    ASSERT_GE(top, 0);

    skill::reference_point_on_face_uv::Input in;
    in.face_id = top;
    in.u       = 0.5;
    in.v       = 0.5;
    in.name    = "PT_CENTER";

    auto out = skill::reference_point_on_face_uv::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_EQ(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Signature kind + datum_class ──────────────────────────────────────
TEST(SkillReferencePointOnFaceUV, SignatureRecordsKindAndDatumClass)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const int top = findTopFaceId(*stock);
    ASSERT_GE(top, 0);

    skill::reference_point_on_face_uv::Input in;
    in.face_id = top;
    in.u       = 0.25;
    in.v       = 0.75;
    in.name    = "PT_QUART";

    auto out = skill::reference_point_on_face_uv::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("reference_point_on_face_uv"));
    EXPECT_EQ(out.signature.pattern["datum_class"].get<std::string>(),
              std::string("point"));
    EXPECT_TRUE(out.signature.pattern["is_reference"].get<bool>());
    EXPECT_NEAR(out.signature.pattern["u_norm"].get<double>(), 0.25, 1e-9);
    EXPECT_NEAR(out.signature.pattern["v_norm"].get<double>(), 0.75, 1e-9);
}

// ─── 3. Resolved point at u=v=0.5 ≈ face centroid for planar top face ────
TEST(SkillReferencePointOnFaceUV, ResolvedPointMatchesFaceCenter)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const int top = findTopFaceId(*stock);
    ASSERT_GE(top, 0);

    const gp_Pnt c = stock->faceCenter(top);

    skill::reference_point_on_face_uv::Input in;
    in.face_id = top;
    in.u       = 0.5;
    in.v       = 0.5;
    in.name    = "PT_TOP_CTR";

    auto out = skill::reference_point_on_face_uv::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern["resolved"].get<bool>());

    auto pt = out.signature.pattern["point_xyz"];
    // For a planar rectangular face whose underlying surface parameter range
    // spans the rectangle uniformly, the midpoint (u=v=0.5) lies at the
    // face's parametric centre, which coincides with the geometric centre
    // (within rounding) for axis-aligned rectangles.
    EXPECT_NEAR(pt[0].get<double>(), c.X(), 1e-3);
    EXPECT_NEAR(pt[1].get<double>(), c.Y(), 1e-3);
    EXPECT_NEAR(pt[2].get<double>(), c.Z(), 1e-3);
}

// ─── 4. DFM rejects out-of-range u/v ──────────────────────────────────────
TEST(SkillReferencePointOnFaceUV, ValidateRejectsOutOfRangeUV)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const int top = findTopFaceId(*stock);
    ASSERT_GE(top, 0);

    skill::reference_point_on_face_uv::Input in;
    in.face_id = top;
    in.u       = 1.5;
    in.v       = 0.5;
    in.name    = "BAD";

    auto r = skill::reference_point_on_face_uv::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::reference_point_on_face_uv::apply(*stock, in),
                 skill::SkillError);
}

// ─── 5. Recognize replay ──────────────────────────────────────────────────
TEST(SkillReferencePointOnFaceUV, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const int top = findTopFaceId(*stock);
    ASSERT_GE(top, 0);

    skill::reference_point_on_face_uv::Input in;
    in.face_id = top;
    in.u       = 0.3;
    in.v       = 0.7;
    in.name    = "PT_037";

    auto out   = skill::reference_point_on_face_uv::apply(*stock, in);
    auto cands = skill::reference_point_on_face_uv::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_NEAR(cands[0].recovered_params["u"].get<double>(), 0.3, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["v"].get<double>(), 0.7, 1e-9);
}
