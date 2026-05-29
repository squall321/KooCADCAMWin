// @lat: [[process/test-strategy#compound macro]]
//
// o_ring_groove_as568_spec — 5 tests verifying REAL volume/spec behaviour.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/o_ring_groove_as568_spec.hpp"

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

// ─── T1: apply removes ~ groove volume (±20 %) ────────────────────────────
TEST(SkillORingGrooveAS568Spec, T1_ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    const double v0 = volumeOf(stock->shape());

    skill::o_ring_groove_as568_spec::Input in;
    in.face_id     = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.center_x_mm = 40.0;
    in.center_y_mm = 40.0;
    in.axis_dir    = gp_Dir(0, 0, -1);
    in.mean_dia_mm = 40.0;
    in.dash_size   = "-212";    // CS = 3.53 mm

    auto out = skill::o_ring_groove_as568_spec::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected volume math (T1 expected math, per task report):
    //   CS = 3.53, depth = 0.75 * 3.53 = 2.6475, width = 1.40 * 3.53 = 4.942
    //   meanR = 20, outerR = 22.471, innerR = 17.529
    //   grooveVol = pi * (outerR^2 - innerR^2) * depth
    //             = pi * (504.946 - 307.262) * 2.6475
    //             = pi * 197.684 * 2.6475 ≈ 1644.4 mm³
    const double cs = 3.53;
    const double depth = 0.75 * cs;
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
TEST(SkillORingGrooveAS568Spec, T2_ValidateRejectsUnknownSpec)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);

    skill::o_ring_groove_as568_spec::Input bad;
    bad.face_id     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.center_x_mm = 40.0;
    bad.center_y_mm = 40.0;
    bad.mean_dia_mm = 40.0;
    bad.dash_size   = "-XYZ";          // not in spec table

    auto r = skill::o_ring_groove_as568_spec::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-AS568-SPEC") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::o_ring_groove_as568_spec::apply(*stock, bad),
                 skill::SkillError);
}

// ─── T3: signature carries spec_key + is_compound = true ──────────────────
TEST(SkillORingGrooveAS568Spec, T3_SignatureCarriesSpecKey)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    skill::o_ring_groove_as568_spec::Input in;
    in.face_id     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 40.0;
    in.center_y_mm = 40.0;
    in.axis_dir    = gp_Dir(0, 0, -1);
    in.mean_dia_mm = 40.0;
    in.dash_size   = "-325";

    auto out = skill::o_ring_groove_as568_spec::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("o_ring_groove_as568_spec"));
    EXPECT_EQ(pat.at("is_compound").get<bool>(), true);
    EXPECT_EQ(pat.at("subfeature_count").get<int>(), 3);
    EXPECT_EQ(pat.at("spec_key").get<std::string>(), std::string("-325"));
}

// ─── T4: recognize returns ≥1 candidate with correct spec key ─────────────
TEST(SkillORingGrooveAS568Spec, T4_RecognizeReplaysSpec)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 20.0);
    skill::o_ring_groove_as568_spec::Input in;
    in.face_id     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm = 40.0;
    in.center_y_mm = 40.0;
    in.axis_dir    = gp_Dir(0, 0, -1);
    in.mean_dia_mm = 40.0;
    in.dash_size   = "-111";

    auto out = skill::o_ring_groove_as568_spec::apply(*stock, in);
    auto cands = skill::o_ring_groove_as568_spec::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id,
              std::string("o_ring_groove_as568_spec"));
    EXPECT_EQ(cands[0].recovered_params.at("dash_size").get<std::string>(),
              std::string("-111"));
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}

// ─── T5: spec-table values match AS568D / Parker §4-2 constants ──────────
TEST(SkillORingGrooveAS568Spec, T5_SpecTableAndFormulas)
{
    // 10 dash sizes from the embedded AS568D table.
    EXPECT_NEAR(skill::o_ring_groove_as568_spec::crossSectionFor("-006"), 1.78, 1e-6);
    EXPECT_NEAR(skill::o_ring_groove_as568_spec::crossSectionFor("-011"), 1.78, 1e-6);
    EXPECT_NEAR(skill::o_ring_groove_as568_spec::crossSectionFor("-016"), 1.78, 1e-6);
    EXPECT_NEAR(skill::o_ring_groove_as568_spec::crossSectionFor("-111"), 2.62, 1e-6);
    EXPECT_NEAR(skill::o_ring_groove_as568_spec::crossSectionFor("-116"), 2.62, 1e-6);
    EXPECT_NEAR(skill::o_ring_groove_as568_spec::crossSectionFor("-212"), 3.53, 1e-6);
    EXPECT_NEAR(skill::o_ring_groove_as568_spec::crossSectionFor("-224"), 3.53, 1e-6);
    EXPECT_NEAR(skill::o_ring_groove_as568_spec::crossSectionFor("-325"), 5.33, 1e-6);
    EXPECT_NEAR(skill::o_ring_groove_as568_spec::crossSectionFor("-425"), 6.99, 1e-6);
    EXPECT_NEAR(skill::o_ring_groove_as568_spec::crossSectionFor("-908"), 1.78, 1e-6);
    EXPECT_EQ(skill::o_ring_groove_as568_spec::crossSectionFor("-NOPE"), 0.0);

    // Parker §4-2 formulas:  G = 0.75 × CS, W = 1.40 × CS.
    EXPECT_NEAR(skill::o_ring_groove_as568_spec::recommendedDepthFor(3.53),
                0.75 * 3.53, 1e-6);
    EXPECT_NEAR(skill::o_ring_groove_as568_spec::recommendedWidthFor(3.53),
                1.40 * 3.53, 1e-6);
}
