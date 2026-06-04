// @lat: [[process/test-strategy#compound macro — retaining_ring_din5417_spiral]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/retaining_ring_din5417_spiral.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}

// ─── T1: apply cuts wide spiral groove (2-turn) ──────────────────────────
TEST(SkillRetainingRingDin5417Spiral, T1_ApplyRemovesExpectedVolume)
{
    // 20 mm shaft × 40 mm long; 2 turns × 1.0 mm = 2.10 mm wide groove
    auto stock = skill::createCylindricalStock(20.0, 40.0);
    const double v0 = volumeOf(stock->shape());

    skill::retaining_ring_din5417_spiral::Input in;
    in.face_id              = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0,0,0), gp::DZ()), 5.0 };
    in.shaft_z_position_mm  = 20.0;
    in.shaft_dia_mm         = 20.0;
    in.ring_thickness_mm    = 1.0;
    in.turn_count           = 2;

    auto out = skill::retaining_ring_din5417_spiral::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double depth   = 1.0;
    const double width   = 2 * 1.0 * 1.05;
    const double outerR  = 10.0;
    const double grooveR = outerR - depth;
    const double expected =
        M_PI * (outerR * outerR - grooveR * grooveR) * width;

    const double removed = v0 - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 0.80 * expected);
    EXPECT_LT(removed, 1.20 * expected);
}

// ─── T2: validate rejects invalid turn_count ─────────────────────────────
TEST(SkillRetainingRingDin5417Spiral, T2_ValidateRejectsInvalidTurns)
{
    auto stock = skill::createCylindricalStock(20.0, 40.0);
    skill::retaining_ring_din5417_spiral::Input bad;
    bad.face_id             = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0,0,0), gp::DZ()), 5.0 };
    bad.shaft_z_position_mm = 20.0;
    bad.shaft_dia_mm        = 20.0;
    bad.ring_thickness_mm   = 1.0;
    bad.turn_count          = 5;            // must be 2 or 3

    auto r = skill::retaining_ring_din5417_spiral::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::retaining_ring_din5417_spiral::apply(*stock, bad),
                 skill::SkillError);
}

// ─── T3: signature pattern (3-turn) ──────────────────────────────────────
TEST(SkillRetainingRingDin5417Spiral, T3_SignaturePattern)
{
    auto stock = skill::createCylindricalStock(20.0, 40.0);
    skill::retaining_ring_din5417_spiral::Input in;
    in.face_id              = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0,0,0), gp::DZ()), 5.0 };
    in.shaft_z_position_mm  = 20.0;
    in.shaft_dia_mm         = 20.0;
    in.ring_thickness_mm    = 1.0;
    in.turn_count           = 3;

    auto out = skill::retaining_ring_din5417_spiral::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("retaining_ring_din5417_spiral"));
    EXPECT_EQ(pat.at("is_compound").get<bool>(), true);
    EXPECT_EQ(pat.at("ring_standard").get<std::string>(),
              std::string("DIN 5417 / ISO 5417"));
    ASSERT_TRUE(pat.contains("derived_dims"));
    EXPECT_EQ(pat.at("derived_dims").at("turn_count").get<int>(), 3);
    EXPECT_GT(pat.at("ring_volume_mm3").get<double>(), 0.0);
}

// ─── T4: recognize replays via metadata ───────────────────────────────────
TEST(SkillRetainingRingDin5417Spiral, T4_RecognizeReplays)
{
    auto stock = skill::createCylindricalStock(20.0, 40.0);
    skill::retaining_ring_din5417_spiral::Input in;
    in.face_id              = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0,0,0), gp::DZ()), 5.0 };
    in.shaft_z_position_mm  = 20.0;
    in.shaft_dia_mm         = 20.0;
    in.ring_thickness_mm    = 1.0;
    in.turn_count           = 2;

    auto out = skill::retaining_ring_din5417_spiral::apply(*stock, in);
    auto cands = skill::retaining_ring_din5417_spiral::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id,
              std::string("retaining_ring_din5417_spiral"));
    EXPECT_EQ(cands[0].recovered_params.at("turn_count").get<int>(), 2);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}

// ─── T5: 3-turn width is 1.5× wider than 2-turn ──────────────────────────
TEST(SkillRetainingRingDin5417Spiral, T5_TurnsScaleWidth)
{
    auto stock2 = skill::createCylindricalStock(20.0, 40.0);
    skill::retaining_ring_din5417_spiral::Input in2;
    in2.face_id             = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0,0,0), gp::DZ()), 5.0 };
    in2.shaft_z_position_mm = 20.0;
    in2.shaft_dia_mm        = 20.0;
    in2.ring_thickness_mm   = 1.0;
    in2.turn_count          = 2;
    auto out2 = skill::retaining_ring_din5417_spiral::apply(*stock2, in2);

    auto stock3 = skill::createCylindricalStock(20.0, 40.0);
    skill::retaining_ring_din5417_spiral::Input in3 = in2;
    in3.turn_count          = 3;
    auto out3 = skill::retaining_ring_din5417_spiral::apply(*stock3, in3);

    const double w2 = out2.signature.pattern.at("derived_dims")
                      .at("groove_width_mm").get<double>();
    const double w3 = out3.signature.pattern.at("derived_dims")
                      .at("groove_width_mm").get<double>();
    EXPECT_NEAR(w3 / w2, 1.5, 1e-6);
}
