// @lat: [[process/test-strategy#compound macro]]
//
// o_ring_groove_face — compound: face-seal annular groove + ID + OD lead-in.
//
// Tests (5):
//   1. ApplyProducesValidShapeWithMultipleSubFeatures — face count growth.
//   2. ValidateRejectsBadInput — unknown AS568 + mean_dia too small.
//   3. SignatureRecordsCompoundKind.
//   4. RecognizeMetadataReplay.
//   5. GrooveInnerDiaLessThanOuterDia — ID < OD invariant.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/o_ring_groove_face.hpp"

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

// ─── 1. Apply produces valid shape with multiple sub-features ─────────────
TEST(SkillORingGrooveFace, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    // Cuboid block: 60×60×20, face groove on top face.
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);

    skill::o_ring_groove_face::Input in;
    in.face_id      = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.center_x_mm  = 30.0;
    in.center_y_mm  = 30.0;
    in.axis_dir     = gp_Dir(0, 0, -1);
    in.mean_dia_mm  = 30.0;
    in.o_ring_size  = "-210";    // CS = 3.53 mm

    const int facesBefore = stock->faceCount();
    auto out = skill::o_ring_groove_face::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const int facesAfter = out.workpiece->faceCount();
    EXPECT_GT(facesAfter, facesBefore);

    // Volume removed must be roughly the annular groove volume (chamfers
    // negligibly small in comparison).
    const double cs       = 3.53;
    const double depth    = 0.74 * cs;
    const double width    = 1.50 * cs;
    const double meanR    = 15.0;
    const double outerR   = meanR + width / 2.0;
    const double innerR   = meanR - width / 2.0;
    const double grooveVol = M_PI * (outerR * outerR - innerR * innerR) * depth;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    // Allow generous tolerance — chamfers + overhang add a few mm³.
    EXPECT_GT(removed, 0.8 * grooveVol);
    EXPECT_LT(removed, 1.6 * grooveVol);
}

// ─── 2. Validate rejects bad input ─────────────────────────────────────────
TEST(SkillORingGrooveFace, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);

    skill::o_ring_groove_face::Input bad;
    bad.face_id     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.center_x_mm = 30.0;
    bad.center_y_mm = 30.0;
    bad.mean_dia_mm = 30.0;
    bad.o_ring_size = "-NOPE";

    auto r1 = skill::o_ring_groove_face::validate(*stock, bad);
    EXPECT_FALSE(r1.passed);

    // mean_dia too small: AS568 width would push ID below zero.
    skill::o_ring_groove_face::Input tiny;
    tiny.face_id     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    tiny.center_x_mm = 30.0;
    tiny.center_y_mm = 30.0;
    tiny.mean_dia_mm = 1.0;            // way too small for any AS568 size
    tiny.o_ring_size = "-310";          // CS 5.33 → width ≈ 8 mm → ID negative
    auto r2 = skill::o_ring_groove_face::validate(*stock, tiny);
    EXPECT_FALSE(r2.passed);

    EXPECT_THROW(skill::o_ring_groove_face::apply(*stock, bad), skill::SkillError);
}

// ─── 3. Signature records compound kind ────────────────────────────────────
TEST(SkillORingGrooveFace, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);

    skill::o_ring_groove_face::Input in;
    in.face_id     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 30.0;
    in.center_y_mm = 30.0;
    in.axis_dir    = gp_Dir(0, 0, -1);
    in.mean_dia_mm = 30.0;
    in.o_ring_size = "-110";

    auto out = skill::o_ring_groove_face::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("o_ring_groove_face"));
    EXPECT_EQ(pat.at("is_compound").get<bool>(), true);
    EXPECT_GE(pat.at("subfeature_count").get<int>(), 2);
}

// ─── 4. Recognize via metadata replay ──────────────────────────────────────
TEST(SkillORingGrooveFace, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);

    skill::o_ring_groove_face::Input in;
    in.face_id     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 30.0;
    in.center_y_mm = 30.0;
    in.axis_dir    = gp_Dir(0, 0, -1);
    in.mean_dia_mm = 30.0;
    in.o_ring_size = "-210";

    auto out = skill::o_ring_groove_face::apply(*stock, in);
    auto cands = skill::o_ring_groove_face::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("o_ring_groove_face"));
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}

// ─── 5. Groove inner dia < outer dia + CS lookup ──────────────────────────
TEST(SkillORingGrooveFace, GrooveInnerDiaLessThanOuterDia)
{
    EXPECT_NEAR(skill::o_ring_groove_face::crossSectionFor("-110"), 2.62, 1e-6);
    EXPECT_NEAR(skill::o_ring_groove_face::recommendedFaceDepthFor(3.53),
                0.74 * 3.53, 1e-6);
    EXPECT_NEAR(skill::o_ring_groove_face::recommendedFaceWidthFor(3.53),
                1.50 * 3.53, 1e-6);

    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::o_ring_groove_face::Input in;
    in.face_id     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 30.0;
    in.center_y_mm = 30.0;
    in.axis_dir    = gp_Dir(0, 0, -1);
    in.mean_dia_mm = 30.0;
    in.o_ring_size = "-210";

    auto out = skill::o_ring_groove_face::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    const double idDia = pat.at("groove_inner_dia_mm").get<double>();
    const double odDia = pat.at("groove_outer_dia_mm").get<double>();
    EXPECT_LT(idDia, odDia);
    EXPECT_NEAR((idDia + odDia) / 2.0, 30.0, 1e-6);   // mean_dia preserved
}
