// @lat: [[process/test-strategy#compound macro — e_clip_groove_din6799]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/_retaining_rings.hpp"
#include "skills/e_clip_groove_din6799.hpp"

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

// ─── T1: apply cuts an E-clip groove ─────────────────────────────────────
TEST(SkillEClipGrooveDin6799, T1_ApplyRemovesExpectedVolume)
{
    // 5 mm shaft × 20 mm long for DIN 6799 "4mm" (shaft=5, groove=4, w=0.7)
    auto stock = skill::createCylindricalStock(5.0, 20.0);
    const double v0 = volumeOf(stock->shape());

    skill::e_clip_groove_din6799::Input in;
    in.face_id            = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0,0,0), gp::DZ()), 5.0 };
    in.shaft_z_position_mm = 10.0;
    in.ring_size_key       = "4mm";

    auto out = skill::e_clip_groove_din6799::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const auto* spec = skill::retaining_rings::findDin6799("4mm");
    ASSERT_NE(spec, nullptr);
    const double outerR   = 2.5;
    const double depth    = (spec->shaft_dia_mm - spec->groove_dia_mm) / 2.0;
    const double grooveR  = outerR - depth;
    const double expected = M_PI * (outerR * outerR - grooveR * grooveR)
                          * spec->groove_width_mm;

    const double removed = v0 - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 0.80 * expected);
    EXPECT_LT(removed, 1.20 * expected);
}

// ─── T2: validate rejects unknown size ────────────────────────────────────
TEST(SkillEClipGrooveDin6799, T2_ValidateRejectsUnknownSpec)
{
    auto stock = skill::createCylindricalStock(5.0, 20.0);
    skill::e_clip_groove_din6799::Input bad;
    bad.face_id            = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0,0,0), gp::DZ()), 5.0 };
    bad.shaft_z_position_mm = 10.0;
    bad.ring_size_key       = "100mm";    // not in table

    auto r = skill::e_clip_groove_din6799::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DIN6799") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::e_clip_groove_din6799::apply(*stock, bad),
                 skill::SkillError);
}

// ─── T3: signature pattern ────────────────────────────────────────────────
TEST(SkillEClipGrooveDin6799, T3_SignaturePattern)
{
    auto stock = skill::createCylindricalStock(6.0, 20.0);
    skill::e_clip_groove_din6799::Input in;
    in.face_id            = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0,0,0), gp::DZ()), 5.0 };
    in.shaft_z_position_mm = 10.0;
    in.ring_size_key       = "5mm";

    auto out = skill::e_clip_groove_din6799::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("e_clip_groove_din6799"));
    EXPECT_EQ(pat.at("is_compound").get<bool>(), true);
    EXPECT_EQ(pat.at("ring_standard").get<std::string>(),
              std::string("DIN 6799"));
    EXPECT_EQ(pat.at("ring_size_key").get<std::string>(), std::string("5mm"));
    EXPECT_GT(pat.at("ring_volume_mm3").get<double>(), 0.0);
}

// ─── T4: recognize replays via metadata ───────────────────────────────────
TEST(SkillEClipGrooveDin6799, T4_RecognizeReplays)
{
    auto stock = skill::createCylindricalStock(5.0, 20.0);
    skill::e_clip_groove_din6799::Input in;
    in.face_id            = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0,0,0), gp::DZ()), 5.0 };
    in.shaft_z_position_mm = 10.0;
    in.ring_size_key       = "4mm";

    auto out = skill::e_clip_groove_din6799::apply(*stock, in);
    auto cands = skill::e_clip_groove_din6799::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("e_clip_groove_din6799"));
    EXPECT_EQ(cands[0].recovered_params.at("ring_size_key").get<std::string>(),
              std::string("4mm"));
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}

// ─── T5: central table values match DIN 6799 ─────────────────────────────
TEST(SkillEClipGrooveDin6799, T5_CentralTable)
{
    EXPECT_EQ(skill::retaining_rings::kDin6799.size(), 10u);
    const auto* s = skill::retaining_rings::findDin6799("4mm");
    ASSERT_NE(s, nullptr);
    EXPECT_NEAR(s->shaft_dia_mm,             5.0, 1e-6);
    EXPECT_NEAR(s->groove_dia_mm,            4.0, 1e-6);
    EXPECT_NEAR(s->groove_width_mm,          0.7, 1e-6);
    EXPECT_NEAR(s->groove_axial_position_mm, 2.5, 1e-6);
    EXPECT_EQ(skill::retaining_rings::findDin6799("99mm"), nullptr);
}
