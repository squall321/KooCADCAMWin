// @lat: [[process/test-strategy#compound macro]]
//
// face_seal_compound_compression — 5 tests verifying REAL volume + spec.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/face_seal_compound_compression.hpp"

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

// ─── T1: apply removes ~ primary + spring grooves ±20 % ──────────────────
TEST(SkillFaceSealCompoundCompression, T1_ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    const double v0 = volumeOf(stock->shape());

    skill::face_seal_compound_compression::Input in;
    in.face_id              = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.center_x_mm          = 50.0;
    in.center_y_mm          = 50.0;
    in.axis_dir             = gp_Dir(0, 0, -1);
    in.primary_radius_mm    = 30.0;
    in.spring_radius_mm     = 15.0;     // well separated
    in.spring_extra_depth_mm = 0.5;
    in.dash_size            = "-212";   // CS = 3.53

    auto out = skill::face_seal_compound_compression::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // T1 expected math: CS = 3.53
    //   primary depth = 0.75*3.53 = 2.6475, width = 1.40*3.53 = 4.942
    //   spring depth  = 2.6475 + 0.5 = 3.1475, width = 0.80*3.53 = 2.824
    //   primary outer/inner R = 30 ± 2.471 = 32.471 / 27.529
    //   primary vol = pi*(32.471^2 - 27.529^2)*2.6475 = pi*296.526*2.6475 ≈ 2466.4
    //   spring outer/inner R = 15 ± 1.412 = 16.412 / 13.588
    //   spring vol = pi*(16.412^2 - 13.588^2)*3.1475 = pi*84.728*3.1475 ≈ 837.7
    //   Total ≈ 3304.1 mm³
    const double cs = 3.53;
    const double primG = 0.75 * cs;
    const double primW = 1.40 * cs;
    const double sprG  = primG + 0.5;
    const double sprW  = 0.80 * cs;
    const double primOR = 30.0 + primW / 2.0;
    const double primIR = 30.0 - primW / 2.0;
    const double sprOR  = 15.0 + sprW / 2.0;
    const double sprIR  = 15.0 - sprW / 2.0;
    const double expected =
        M_PI * (primOR*primOR - primIR*primIR) * primG +
        M_PI * (sprOR*sprOR   - sprIR*sprIR)   * sprG;

    const double removed = v0 - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 0.80 * expected);
    EXPECT_LT(removed, 1.20 * expected);
}

// ─── T2: validate rejects unknown dash + apply throws ────────────────────
TEST(SkillFaceSealCompoundCompression, T2_ValidateRejectsUnknownSpec)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    skill::face_seal_compound_compression::Input bad;
    bad.face_id              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.primary_radius_mm    = 30.0;
    bad.spring_radius_mm     = 15.0;
    bad.spring_extra_depth_mm = 0.5;
    bad.dash_size            = "-NONE";

    auto r = skill::face_seal_compound_compression::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-FACESEAL-SPEC") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::face_seal_compound_compression::apply(*stock, bad),
                 skill::SkillError);
}

// ─── T3: signature has spec_key + is_compound=true + 3 subfeatures ───────
TEST(SkillFaceSealCompoundCompression, T3_SignatureCarriesSpecKey)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    skill::face_seal_compound_compression::Input in;
    in.face_id              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm          = 50.0;
    in.center_y_mm          = 50.0;
    in.axis_dir             = gp_Dir(0, 0, -1);
    in.primary_radius_mm    = 30.0;
    in.spring_radius_mm     = 15.0;
    in.spring_extra_depth_mm = 0.5;
    in.dash_size            = "-325";

    auto out = skill::face_seal_compound_compression::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("face_seal_compound_compression"));
    EXPECT_EQ(pat.at("is_compound").get<bool>(), true);
    EXPECT_EQ(pat.at("subfeature_count").get<int>(), 3);
    EXPECT_EQ(pat.at("spec_key").get<std::string>(), std::string("-325"));
    EXPECT_EQ(pat.at("groove_count").get<int>(), 2);
}

// ─── T4: recognize replays + matches spec_key ─────────────────────────────
TEST(SkillFaceSealCompoundCompression, T4_RecognizeReplaysSpec)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    skill::face_seal_compound_compression::Input in;
    in.face_id              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm          = 50.0;
    in.center_y_mm          = 50.0;
    in.axis_dir             = gp_Dir(0, 0, -1);
    in.primary_radius_mm    = 30.0;
    in.spring_radius_mm     = 15.0;
    in.spring_extra_depth_mm = 0.5;
    in.dash_size            = "-111";

    auto out = skill::face_seal_compound_compression::apply(*stock, in);
    auto cands = skill::face_seal_compound_compression::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id,
              std::string("face_seal_compound_compression"));
    EXPECT_EQ(cands[0].recovered_params.at("dash_size").get<std::string>(),
              std::string("-111"));
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}

// ─── T5: spring groove is deeper and narrower than primary (Bal Seal §3.2) ─
TEST(SkillFaceSealCompoundCompression, T5_SpringGrooveDeeperAndNarrower)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    skill::face_seal_compound_compression::Input in;
    in.face_id              = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm          = 50.0;
    in.center_y_mm          = 50.0;
    in.axis_dir             = gp_Dir(0, 0, -1);
    in.primary_radius_mm    = 30.0;
    in.spring_radius_mm     = 15.0;
    in.spring_extra_depth_mm = 1.0;
    in.dash_size            = "-212";

    auto out = skill::face_seal_compound_compression::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    // Spring deeper than primary (by spring_extra_depth_mm).
    EXPECT_GT(pat.at("spring_depth_mm").get<double>(),
              pat.at("primary_depth_mm").get<double>());
    // Spring narrower than primary (0.80 × CS vs 1.40 × CS).
    EXPECT_LT(pat.at("spring_width_mm").get<double>(),
              pat.at("primary_width_mm").get<double>());

    // Embedded constants follow standards.
    EXPECT_NEAR(skill::face_seal_compound_compression::primaryDepthFor(3.53),
                0.75 * 3.53, 1e-6);
    EXPECT_NEAR(skill::face_seal_compound_compression::primaryWidthFor(3.53),
                1.40 * 3.53, 1e-6);
    EXPECT_NEAR(skill::face_seal_compound_compression::springWidthFor(3.53),
                0.80 * 3.53, 1e-6);

    // Overlap DFM gate: bring grooves close together → must fail.
    skill::face_seal_compound_compression::Input clash = in;
    clash.spring_radius_mm = 28.5;   // very close to primary 30.0
    auto r = skill::face_seal_compound_compression::validate(*stock, clash);
    EXPECT_FALSE(r.passed);
    bool ovl = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-FACESEAL-OVERLAP") { ovl = true; break; }
    EXPECT_TRUE(ovl);
}
