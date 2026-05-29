// @lat: [[process/test-strategy#compound macro]]
//
// linear_bushing_seat — COMPOUND macro: bore + retention shoulder + 2 lub
// grooves.  Tests:
//   1. apply produces valid shape with multiple sub-features.
//   2. validate rejects bad input.
//   3. signature records compound kind, subfeature_count >= 2.
//   4. recognize metadata replay (confidence 1.0).
//   5. shoulder_id = bushing_od - 2 × shoulder_offset (geometric invariant).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/linear_bushing_seat.hpp"

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
TEST(SkillLinearBushingSeat, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 40.0);

    skill::linear_bushing_seat::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    // Bushing axis: from top center going down (-Z).
    in.axis           = gp_Ax1(gp_Pnt(30.0, 30.0, 40.0), gp_Dir(0, 0, -1));
    in.bushing_od_mm  = 20.0;
    in.bushing_l_mm   = 25.0;

    auto out = skill::linear_bushing_seat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double vBefore = volumeOf(stock->shape());
    const double vAfter  = volumeOf(out.workpiece->shape());
    EXPECT_LT(vAfter, vBefore);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());

    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 4);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillLinearBushingSeat, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 40.0);

    skill::linear_bushing_seat::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis           = gp_Ax1(gp_Pnt(30.0, 30.0, 40.0), gp_Dir(0, 0, -1));
    in.bushing_od_mm  = 9.0;   // too small → shoulder_id = 9 - 8 = 1 mm (still OK but on edge)
    in.bushing_l_mm   = 3.0;   // too short for grooves → DFM-LB-GROOVE

    auto r = skill::linear_bushing_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);

    EXPECT_THROW(skill::linear_bushing_seat::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature records compound kind ───────────────────────────────────
TEST(SkillLinearBushingSeat, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 40.0);

    skill::linear_bushing_seat::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis           = gp_Ax1(gp_Pnt(30.0, 30.0, 40.0), gp_Dir(0, 0, -1));
    in.bushing_od_mm  = 20.0;
    in.bushing_l_mm   = 25.0;

    auto out = skill::linear_bushing_seat::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("linear_bushing_seat"));
    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("linear_bushing_seat"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillLinearBushingSeat, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 40.0);

    skill::linear_bushing_seat::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis           = gp_Ax1(gp_Pnt(30.0, 30.0, 40.0), gp_Dir(0, 0, -1));
    in.bushing_od_mm  = 20.0;
    in.bushing_l_mm   = 25.0;

    auto out = skill::linear_bushing_seat::apply(*stock, in);

    auto cands = skill::linear_bushing_seat::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_DOUBLE_EQ(cands[0].recovered_params.at("bushing_od_mm").get<double>(), 20.0);
}

// ─── 5. shoulder_id = bushing_od - 2 × shoulder_offset (geometric invariant)
TEST(SkillLinearBushingSeat, ShoulderIdMatchesBushingOdMinusOffset)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 40.0);

    skill::linear_bushing_seat::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis           = gp_Ax1(gp_Pnt(30.0, 30.0, 40.0), gp_Dir(0, 0, -1));
    in.bushing_od_mm  = 24.0;
    in.bushing_l_mm   = 25.0;

    auto out = skill::linear_bushing_seat::apply(*stock, in);

    const double od     = out.signature.pattern.at("bushing_od_mm").get<double>();
    const double id     = out.signature.pattern.at("shoulder_id_mm").get<double>();
    const double offset = out.signature.pattern.at("shoulder_offset_mm").get<double>();
    EXPECT_NEAR(id, od - 2.0 * offset, 1e-9);
}
