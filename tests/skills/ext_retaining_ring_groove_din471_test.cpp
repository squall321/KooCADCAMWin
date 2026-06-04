// @lat: [[process/test-strategy#compound macro — ext_retaining_ring_groove_din471]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/_retaining_rings.hpp"
#include "skills/ext_retaining_ring_groove_din471.hpp"

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

// ─── T1: apply cuts ~ groove volume on a shaft ─────────────────────────────
TEST(SkillExtRetainingRingGrooveDin471, T1_ApplyRemovesExpectedVolume)
{
    // Use a 16 mm Ø shaft × 30 mm long matching DIN 471 "16mm"
    auto stock = skill::createCylindricalStock(16.0, 30.0);
    const double v0 = volumeOf(stock->shape());

    skill::ext_retaining_ring_groove_din471::Input in;
    in.face_id = skill::FaceCylinderByAxis{ gp_Ax1(gp_Pnt(0,0,0), gp::DZ()), 5.0 };
    in.shaft_z_position_mm = 15.0;
    in.ring_size_key       = "16mm";

    auto out = skill::ext_retaining_ring_groove_din471::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected volume math: outerR=8, depth=0.4, grooveR=7.6, width=1.1
    //   groove_vol = pi * (8^2 - 7.6^2) * 1.1 = pi * 6.24 * 1.1 ≈ 21.56 mm^3
    const auto* spec = skill::retaining_rings::findDin471("16mm");
    ASSERT_NE(spec, nullptr);
    const double outerR  = 8.0;
    const double grooveR = outerR - spec->groove_depth_mm;
    const double expected = M_PI * (outerR * outerR - grooveR * grooveR)
                          * spec->groove_width_mm;

    const double removed = v0 - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 0.80 * expected);
    EXPECT_LT(removed, 1.20 * expected);
}

// ─── T2: validate rejects unknown ring_size_key ────────────────────────────
TEST(SkillExtRetainingRingGrooveDin471, T2_ValidateRejectsUnknownSpec)
{
    auto stock = skill::createCylindricalStock(16.0, 30.0);
    skill::ext_retaining_ring_groove_din471::Input bad;
    bad.face_id            = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0,0,0), gp::DZ()), 5.0 };
    bad.shaft_z_position_mm = 15.0;
    bad.ring_size_key       = "11mm";       // not in DIN 471 table

    auto r = skill::ext_retaining_ring_groove_din471::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DIN471") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::ext_retaining_ring_groove_din471::apply(*stock, bad),
                 skill::SkillError);
}

// ─── T3: signature carries ring_standard + is_compound = true ─────────────
TEST(SkillExtRetainingRingGrooveDin471, T3_SignaturePattern)
{
    auto stock = skill::createCylindricalStock(20.0, 30.0);
    skill::ext_retaining_ring_groove_din471::Input in;
    in.face_id = skill::FaceCylinderByAxis{ gp_Ax1(gp_Pnt(0,0,0), gp::DZ()), 5.0 };
    in.shaft_z_position_mm = 15.0;
    in.ring_size_key       = "20mm";

    auto out = skill::ext_retaining_ring_groove_din471::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("ext_retaining_ring_groove_din471"));
    EXPECT_EQ(pat.at("is_compound").get<bool>(), true);
    EXPECT_EQ(pat.at("ring_standard").get<std::string>(), std::string("DIN 471"));
    EXPECT_EQ(pat.at("ring_size_key").get<std::string>(), std::string("20mm"));
    EXPECT_GT(pat.at("ring_volume_mm3").get<double>(), 0.0);
    ASSERT_TRUE(pat.contains("derived_dims"));
    EXPECT_NEAR(pat.at("derived_dims").at("shaft_dia_mm").get<double>(),
                20.0, 1e-6);
}

// ─── T4: recognize replays via metadata ──────────────────────────────────
TEST(SkillExtRetainingRingGrooveDin471, T4_RecognizeReplays)
{
    auto stock = skill::createCylindricalStock(25.0, 30.0);
    skill::ext_retaining_ring_groove_din471::Input in;
    in.face_id = skill::FaceCylinderByAxis{ gp_Ax1(gp_Pnt(0,0,0), gp::DZ()), 5.0 };
    in.shaft_z_position_mm = 15.0;
    in.ring_size_key       = "25mm";

    auto out = skill::ext_retaining_ring_groove_din471::apply(*stock, in);
    auto cands = skill::ext_retaining_ring_groove_din471::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id,
              std::string("ext_retaining_ring_groove_din471"));
    EXPECT_EQ(cands[0].recovered_params.at("ring_size_key").get<std::string>(),
              std::string("25mm"));
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}

// ─── T5: central table values match DIN 471 ───────────────────────────────
TEST(SkillExtRetainingRingGrooveDin471, T5_CentralTable)
{
    EXPECT_EQ(skill::retaining_rings::kDin471.size(), 14u);
    const auto* s16 = skill::retaining_rings::findDin471("16mm");
    ASSERT_NE(s16, nullptr);
    EXPECT_NEAR(s16->shaft_dia_mm,    16.0, 1e-6);
    EXPECT_NEAR(s16->groove_dia_mm,   15.2, 1e-6);
    EXPECT_NEAR(s16->groove_width_mm,  1.1, 1e-6);
    EXPECT_NEAR(s16->groove_depth_mm,  0.4, 1e-6);
    EXPECT_EQ(skill::retaining_rings::findDin471("99mm"), nullptr);
}
