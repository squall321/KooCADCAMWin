// @lat: [[process/test-strategy#compound macro — int_retaining_ring_groove_din472]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/_retaining_rings.hpp"
#include "skills/bore_cylindrical.hpp"
#include "skills/int_retaining_ring_groove_din472.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

// Helper: cylindrical stock with a Ø boreDia bore through the Z axis.
std::shared_ptr<skill::Workpiece> stockWithBore(double shaftDia, double thk,
                                                double boreDia)
{
    auto stock = skill::createCylindricalStock(shaftDia, thk);
    skill::bore_cylindrical::Input bin;
    bin.entry_face   = skill::FaceByNormal{ gp_Dir(0,0,1), 5.0, "largest" };
    bin.position_x_mm = 0.0;
    bin.position_y_mm = 0.0;
    bin.axis_dir      = gp_Dir(0,0,-1);
    bin.diameter_mm   = boreDia;
    bin.depth_mm      = thk;
    auto out = skill::bore_cylindrical::apply(*stock, bin);
    return out.workpiece;
}
}

// ─── T1: apply cuts ~ groove volume in a bore ─────────────────────────────
TEST(SkillIntRetainingRingGrooveDin472, T1_ApplyRemovesExpectedVolume)
{
    // 30 mm OD stock, 12 mm thk, 16 mm bore -> DIN 472 "16mm"
    auto wp = stockWithBore(30.0, 12.0, 16.0);
    const double v0 = volumeOf(wp->shape());

    skill::int_retaining_ring_groove_din472::Input in;
    in.bore_face_id        = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0,0,0), gp::DZ()), 5.0 };
    in.bore_z_position_mm  = 6.0;
    in.ring_size_key       = "16mm";

    auto out = skill::int_retaining_ring_groove_din472::apply(*wp, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const auto* spec = skill::retaining_rings::findDin472("16mm");
    ASSERT_NE(spec, nullptr);
    const double boreR    = 8.0;
    const double grooveR  = boreR + spec->groove_depth_mm;
    const double expected = M_PI * (grooveR * grooveR - boreR * boreR)
                          * spec->groove_width_mm;

    const double removed = v0 - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 0.70 * expected);
    EXPECT_LT(removed, 1.30 * expected);
}

// ─── T2: validate rejects unknown size + missing bore ─────────────────────
TEST(SkillIntRetainingRingGrooveDin472, T2_ValidateRejectsUnknownSpec)
{
    auto wp = stockWithBore(30.0, 12.0, 16.0);
    skill::int_retaining_ring_groove_din472::Input bad;
    bad.bore_face_id       = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0,0,0), gp::DZ()), 5.0 };
    bad.bore_z_position_mm = 6.0;
    bad.ring_size_key      = "17mm";

    auto r = skill::int_retaining_ring_groove_din472::validate(*wp, bad);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DIN472") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::int_retaining_ring_groove_din472::apply(*wp, bad),
                 skill::SkillError);
}

// ─── T3: signature pattern ────────────────────────────────────────────────
TEST(SkillIntRetainingRingGrooveDin472, T3_SignaturePattern)
{
    auto wp = stockWithBore(30.0, 12.0, 20.0);
    skill::int_retaining_ring_groove_din472::Input in;
    in.bore_face_id       = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0,0,0), gp::DZ()), 5.0 };
    in.bore_z_position_mm = 6.0;
    in.ring_size_key      = "20mm";

    auto out = skill::int_retaining_ring_groove_din472::apply(*wp, in);
    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("int_retaining_ring_groove_din472"));
    EXPECT_EQ(pat.at("is_compound").get<bool>(), true);
    EXPECT_EQ(pat.at("ring_standard").get<std::string>(), std::string("DIN 472"));
    EXPECT_EQ(pat.at("ring_size_key").get<std::string>(), std::string("20mm"));
    EXPECT_GT(pat.at("ring_volume_mm3").get<double>(), 0.0);
}

// ─── T4: recognize replays via metadata ───────────────────────────────────
TEST(SkillIntRetainingRingGrooveDin472, T4_RecognizeReplays)
{
    auto wp = stockWithBore(30.0, 12.0, 16.0);
    skill::int_retaining_ring_groove_din472::Input in;
    in.bore_face_id       = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0,0,0), gp::DZ()), 5.0 };
    in.bore_z_position_mm = 6.0;
    in.ring_size_key      = "16mm";

    auto out = skill::int_retaining_ring_groove_din472::apply(*wp, in);
    auto cands = skill::int_retaining_ring_groove_din472::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id,
              std::string("int_retaining_ring_groove_din472"));
    EXPECT_EQ(cands[0].recovered_params.at("ring_size_key").get<std::string>(),
              std::string("16mm"));
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}

// ─── T5: central table values match DIN 472 ──────────────────────────────
TEST(SkillIntRetainingRingGrooveDin472, T5_CentralTable)
{
    EXPECT_EQ(skill::retaining_rings::kDin472.size(), 13u);
    const auto* s16 = skill::retaining_rings::findDin472("16mm");
    ASSERT_NE(s16, nullptr);
    EXPECT_NEAR(s16->bore_dia_mm,    16.0, 1e-6);
    EXPECT_NEAR(s16->groove_dia_mm,  16.8, 1e-6);
    EXPECT_NEAR(s16->groove_width_mm, 1.1, 1e-6);
    EXPECT_NEAR(s16->groove_depth_mm, 0.4, 1e-6);
    EXPECT_EQ(skill::retaining_rings::findDin472("99mm"), nullptr);
}
