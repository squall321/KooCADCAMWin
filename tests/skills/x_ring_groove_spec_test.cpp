// @lat: [[process/test-strategy#compound macro]]
//
// x_ring_groove_spec — 5 tests verifying REAL volume + spec.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/x_ring_groove_spec.hpp"

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

// ─── T1: apply removes ~ quad-ring groove volume (±20 %) ──────────────────
TEST(SkillXRingGrooveSpec, T1_ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    const double v0 = volumeOf(stock->shape());

    skill::x_ring_groove_spec::Input in;
    in.face_id     = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.center_x_mm = 40.0;
    in.center_y_mm = 40.0;
    in.axis_dir    = gp_Dir(0, 0, -1);
    in.mean_dia_mm = 40.0;
    in.dash_size   = "-212";

    auto out = skill::x_ring_groove_spec::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // T1 expected math: CS = 3.53, depth = 0.78*3.53 = 2.7534,
    // width = 1.40*3.53 = 4.942, meanR = 20, outerR = 22.471, innerR = 17.529.
    // grooveVol = pi*(outerR^2 - innerR^2)*depth
    //           = pi*197.684*2.7534 ≈ 1709.8 mm³
    const double cs = 3.53;
    const double depth = 0.78 * cs;
    const double width = 1.40 * cs;
    const double meanR = 20.0;
    const double outerR = meanR + width / 2.0;
    const double innerR = meanR - width / 2.0;
    const double grooveVol = M_PI * (outerR * outerR - innerR * innerR) * depth;

    const double removed = v0 - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 0.80 * grooveVol);
    EXPECT_LT(removed, 1.20 * grooveVol);
}

// ─── T2: validate rejects unknown dash + apply throws ─────────────────────
TEST(SkillXRingGrooveSpec, T2_ValidateRejectsUnknownSpec)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    skill::x_ring_groove_spec::Input bad;
    bad.face_id     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.center_x_mm = 40.0;
    bad.center_y_mm = 40.0;
    bad.mean_dia_mm = 40.0;
    bad.dash_size   = "-WHAT";

    auto r = skill::x_ring_groove_spec::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-XRING-SPEC") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::x_ring_groove_spec::apply(*stock, bad),
                 skill::SkillError);
}

// ─── T3: signature has spec_key + seal_profile = quad_ring ────────────────
TEST(SkillXRingGrooveSpec, T3_SignatureCarriesSpecKeyAndProfile)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    skill::x_ring_groove_spec::Input in;
    in.face_id     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 40.0;
    in.center_y_mm = 40.0;
    in.axis_dir    = gp_Dir(0, 0, -1);
    in.mean_dia_mm = 40.0;
    in.dash_size   = "-325";

    auto out = skill::x_ring_groove_spec::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("x_ring_groove_spec"));
    EXPECT_EQ(pat.at("is_compound").get<bool>(), true);
    EXPECT_EQ(pat.at("subfeature_count").get<int>(), 3);
    EXPECT_EQ(pat.at("spec_key").get<std::string>(), std::string("-325"));
    EXPECT_EQ(pat.at("seal_profile").get<std::string>(),
              std::string("quad_ring"));
}

// ─── T4: recognize returns ≥1 candidate with right dash_size ─────────────
TEST(SkillXRingGrooveSpec, T4_RecognizeReplaysSpec)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    skill::x_ring_groove_spec::Input in;
    in.face_id     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 40.0;
    in.center_y_mm = 40.0;
    in.axis_dir    = gp_Dir(0, 0, -1);
    in.mean_dia_mm = 40.0;
    in.dash_size   = "-116";

    auto out = skill::x_ring_groove_spec::apply(*stock, in);
    auto cands = skill::x_ring_groove_spec::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("x_ring_groove_spec"));
    EXPECT_EQ(cands[0].recovered_params.at("dash_size").get<std::string>(),
              std::string("-116"));
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}

// ─── T5: X-ring depth = 0.78*CS — deeper than O-ring (0.75*CS) ────────────
TEST(SkillXRingGrooveSpec, T5_XRingDepthIsDeeperThanORing)
{
    // X-ring spec formula: G = 0.78 * CS.
    EXPECT_NEAR(skill::x_ring_groove_spec::recommendedDepthFor(3.53),
                0.78 * 3.53, 1e-6);
    EXPECT_NEAR(skill::x_ring_groove_spec::recommendedWidthFor(3.53),
                1.40 * 3.53, 1e-6);
    // X-ring depth > O-ring depth at same CS (0.78 vs 0.75).
    EXPECT_GT(skill::x_ring_groove_spec::recommendedDepthFor(3.53),
              0.75 * 3.53);
    // Spec coverage.
    EXPECT_NEAR(skill::x_ring_groove_spec::crossSectionFor("-006"), 1.78, 1e-6);
    EXPECT_NEAR(skill::x_ring_groove_spec::crossSectionFor("-425"), 6.99, 1e-6);
    EXPECT_NEAR(skill::x_ring_groove_spec::crossSectionFor("-908"), 1.78, 1e-6);
    EXPECT_EQ(skill::x_ring_groove_spec::crossSectionFor("-NONE"), 0.0);
}
