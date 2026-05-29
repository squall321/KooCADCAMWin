// @lat: [[process/test-strategy#compound macro]]
//
// lip_seal_seat — compound: shaft bore + counterbore + dust groove + nose chamfer.
//
// Tests (5):
//   1. ApplyProducesValidShapeWithMultipleSubFeatures.
//   2. ValidateRejectsBadInput — seal_od too close to shaft_id.
//   3. SignatureRecordsCompoundKind.
//   4. RecognizeMetadataReplay.
//   5. SealCounterboreLargerThanShaftBore — seal_od > shaft_id by ≥ 2 mm.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/lip_seal_seat.hpp"

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
TEST(SkillLipSealSeat, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);

    skill::lip_seal_seat::Input in;
    in.body_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.center_x_mm       = 30.0;
    in.center_y_mm       = 30.0;
    in.axis_dir          = gp_Dir(0, 0, -1);
    in.shaft_id_mm       = 12.0;
    in.seal_od_mm        = 20.0;
    in.seal_thickness_mm = 5.0;
    in.bore_depth_mm     = 25.0;

    const int facesBefore = stock->faceCount();
    auto out = skill::lip_seal_seat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // 4 sub-features → topology gain compared to a plain through bore.
    const int facesAfter = out.workpiece->faceCount();
    EXPECT_GT(facesAfter, facesBefore);

    // Volume removed lower bound: just the shaft bore + counterbore.
    const double shaftR  = 6.0;
    const double sealR   = 10.0;
    const double vBore   = M_PI * shaftR * shaftR * 25.0;
    const double vCBore  = M_PI * (sealR * sealR - shaftR * shaftR) * 5.0;
    const double minVol  = vBore + vCBore;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 0.9 * minVol);
}

// ─── 2. Validate rejects bad input ─────────────────────────────────────────
TEST(SkillLipSealSeat, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);

    // seal_od ≤ shaft_id + 2.0 mm → DFM-LIPSEAL-WALL
    skill::lip_seal_seat::Input bad;
    bad.body_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.center_x_mm       = 30.0;
    bad.center_y_mm       = 30.0;
    bad.shaft_id_mm       = 12.0;
    bad.seal_od_mm        = 13.0;    // gap = 1 mm < 2 mm required
    bad.seal_thickness_mm = 5.0;
    bad.bore_depth_mm     = 25.0;

    auto r1 = skill::lip_seal_seat::validate(*stock, bad);
    EXPECT_FALSE(r1.passed);
    bool foundWall = false;
    for (const auto& f : r1.findings)
        if (f.code == "DFM-LIPSEAL-WALL") { foundWall = true; break; }
    EXPECT_TRUE(foundWall);

    EXPECT_THROW(skill::lip_seal_seat::apply(*stock, bad), skill::SkillError);

    // shaft_id < 1.0 mm
    skill::lip_seal_seat::Input tiny;
    tiny.body_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    tiny.shaft_id_mm       = 0.5;
    tiny.seal_od_mm        = 5.0;
    tiny.seal_thickness_mm = 2.0;
    tiny.bore_depth_mm     = 10.0;
    auto r2 = skill::lip_seal_seat::validate(*stock, tiny);
    EXPECT_FALSE(r2.passed);
}

// ─── 3. Signature records compound kind ────────────────────────────────────
TEST(SkillLipSealSeat, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);

    skill::lip_seal_seat::Input in;
    in.body_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm       = 30.0;
    in.center_y_mm       = 30.0;
    in.axis_dir          = gp_Dir(0, 0, -1);
    in.shaft_id_mm       = 12.0;
    in.seal_od_mm        = 20.0;
    in.seal_thickness_mm = 5.0;
    in.bore_depth_mm     = 25.0;

    auto out = skill::lip_seal_seat::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("lip_seal_seat"));
    EXPECT_EQ(pat.at("is_compound").get<bool>(), true);
    EXPECT_GE(pat.at("subfeature_count").get<int>(), 2);
    EXPECT_NEAR(pat.at("nose_chamfer_deg").get<double>(), 15.0, 1e-6);
}

// ─── 4. Recognize metadata replay ──────────────────────────────────────────
TEST(SkillLipSealSeat, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);

    skill::lip_seal_seat::Input in;
    in.body_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm       = 30.0;
    in.center_y_mm       = 30.0;
    in.axis_dir          = gp_Dir(0, 0, -1);
    in.shaft_id_mm       = 12.0;
    in.seal_od_mm        = 20.0;
    in.seal_thickness_mm = 5.0;
    in.bore_depth_mm     = 25.0;

    auto out   = skill::lip_seal_seat::apply(*stock, in);
    auto cands = skill::lip_seal_seat::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("lip_seal_seat"));
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_NEAR(cands[0].recovered_params.at("shaft_id_mm").get<double>(), 12.0, 1e-6);
}

// ─── 5. Seal counterbore must be larger than shaft bore by ≥ 2 mm ─────────
TEST(SkillLipSealSeat, SealCounterboreLargerThanShaftBore)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);

    skill::lip_seal_seat::Input in;
    in.body_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm       = 30.0;
    in.center_y_mm       = 30.0;
    in.axis_dir          = gp_Dir(0, 0, -1);
    in.shaft_id_mm       = 12.0;
    in.seal_od_mm        = 20.0;
    in.seal_thickness_mm = 5.0;
    in.bore_depth_mm     = 25.0;

    auto out = skill::lip_seal_seat::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    const double shaftId = pat.at("shaft_id_mm").get<double>();
    const double sealOd  = pat.at("seal_od_mm").get<double>();
    EXPECT_GT(sealOd, shaftId + 2.0 - 1e-6);
    EXPECT_GT(pat.at("bore_depth_mm").get<double>(),
              pat.at("seal_thickness_mm").get<double>());
}
