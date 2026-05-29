// @lat: [[process/test-strategy#perforated_grille_pattern]]
//
// perforated_grille_pattern — COMPOUND, SPEC-DRIVEN feature (CUTS hex holes).
//
// Test cases:
//   1. apply removes (hex hole count) × π × (d/2)² × wall_thickness mm³.
//   2. validate rejects unknown spec + sub-50 % free area + apply throws.
//   3. signature carries spec key + subfeature_count (hole_count).
//   4. recognize returns the candidate with the right spec key (> 50 % conf).
//   5. spec table & open-area formula: HEX-3 → π/(2√3) × (3/5)² = 0.3265
//      < 50 % so HEX-3 cannot satisfy DFM-GRILLE-FA on its own → fails.
//      A custom (d=6 mm, pitch=8 mm) hex pattern yields 0.51 > 0.5 → passes.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/perforated_grille_pattern.hpp"

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

// ─── 1. apply removes hex-hole volume ───────────────────────────────────
TEST(SkillPerforatedGrille, ApplyRemovesHexHoleVolume)
{
    // 80 × 40 × 3 mm plate.  Use a custom (d=6, pitch=8) to satisfy
    // DFM-GRILLE-FA (open-area = 0.51 > 0.5).
    auto stock = skill::createCuboidStock(80.0, 40.0, 3.0);
    const double v0 = volumeOf(stock->shape());

    skill::perforated_grille_pattern::Input in;
    in.face                = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.spec_key            = "";        // override
    in.hole_dia_mm         = 6.0;
    in.hex_pitch_mm        = 8.0;
    in.footprint_length_mm = 60.0;
    in.footprint_width_mm  = 30.0;
    in.center_x_mm         = 40.0;
    in.center_y_mm         = 20.0;
    in.grid_axis           = gp_Dir(1, 0, 0);
    in.wall_thickness_mm   = 3.0;

    auto out = skill::perforated_grille_pattern::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    const int holeCount =
        out.signature.pattern.at("subfeature_count").get<int>();
    ASSERT_GE(holeCount, 4);
    const double approx = holeCount *
        M_PI * std::pow(in.hole_dia_mm / 2.0, 2.0) * in.wall_thickness_mm;
    const double removed = v0 - v1;
    EXPECT_NEAR(removed, approx, approx * 0.05)
        << "expected ≈ " << approx << " mm³, got " << removed;
    EXPECT_EQ(out.signature.skill_id,
              std::string("perforated_grille_pattern"));
}

// ─── 2. validate rejects bad inputs ─────────────────────────────────────
TEST(SkillPerforatedGrille, ValidateRejectsBadInputs)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 3.0);

    // (a) Unknown spec
    {
        skill::perforated_grille_pattern::Input in;
        in.face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.spec_key = "HEX-99";
        in.footprint_length_mm = 60.0;
        in.footprint_width_mm  = 30.0;
        auto r = skill::perforated_grille_pattern::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool sp = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-GRILLE-SP") { sp = true; break; }
        EXPECT_TRUE(sp);
        EXPECT_THROW(skill::perforated_grille_pattern::apply(*stock, in),
                     skill::SkillError);
    }

    // (b) Free-area gate: HEX-3 fails it (open area 0.3265 < 0.5).
    {
        skill::perforated_grille_pattern::Input in;
        in.face                = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.spec_key            = "HEX-3";
        in.footprint_length_mm = 60.0;
        in.footprint_width_mm  = 30.0;
        auto r = skill::perforated_grille_pattern::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool fa = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-GRILLE-FA") { fa = true; break; }
        EXPECT_TRUE(fa);
        EXPECT_THROW(skill::perforated_grille_pattern::apply(*stock, in),
                     skill::SkillError);
    }
}

// ─── 3. signature carries spec key + subfeature_count ───────────────────
TEST(SkillPerforatedGrille, SignatureCarriesSpec)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 3.0);

    skill::perforated_grille_pattern::Input in;
    in.face                = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.spec_key            = "";
    in.hole_dia_mm         = 6.0;
    in.hex_pitch_mm        = 8.0;
    in.footprint_length_mm = 60.0;
    in.footprint_width_mm  = 30.0;
    in.center_x_mm         = 40.0;
    in.center_y_mm         = 20.0;
    in.wall_thickness_mm   = 3.0;
    auto out = skill::perforated_grille_pattern::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("perforated_grille_pattern"));
    EXPECT_EQ(out.signature.pattern.at("is_compound").get<bool>(), true);
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 4);
    EXPECT_NEAR(out.signature.pattern.at("hole_dia_mm").get<double>(),
                6.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("hex_pitch_mm").get<double>(),
                8.0, 1e-6);
}

// ─── 4. recognize returns candidate ─────────────────────────────────────
TEST(SkillPerforatedGrille, RecognizeRecoversSpec)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 3.0);

    skill::perforated_grille_pattern::Input in;
    in.face                = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.spec_key            = "";
    in.hole_dia_mm         = 6.0;
    in.hex_pitch_mm        = 8.0;
    in.footprint_length_mm = 60.0;
    in.footprint_width_mm  = 30.0;
    in.center_x_mm         = 40.0;
    in.center_y_mm         = 20.0;
    in.wall_thickness_mm   = 3.0;
    auto out = skill::perforated_grille_pattern::apply(*stock, in);

    auto cands = skill::perforated_grille_pattern::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_GT(cands.front().confidence, 0.5);
    EXPECT_NEAR(cands.front().recovered_params["hole_dia_mm"].get<double>(),
                6.0, 1e-6);
    EXPECT_NEAR(cands.front().recovered_params["hex_pitch_mm"].get<double>(),
                8.0, 1e-6);
}

// ─── 5. spec table + hex grid count + open-area formula ─────────────────
TEST(SkillPerforatedGrille, SpecTableAndHexGridMath)
{
    auto s3 = skill::perforated_grille_pattern::grilleSpecFor("HEX-3");
    auto s5 = skill::perforated_grille_pattern::grilleSpecFor("HEX-5");
    auto s8 = skill::perforated_grille_pattern::grilleSpecFor("HEX-8");
    EXPECT_NEAR(s3.hole_dia_mm,  3.0, 1e-9);
    EXPECT_NEAR(s3.hex_pitch_mm, 5.0, 1e-9);
    EXPECT_NEAR(s5.hole_dia_mm,  5.0, 1e-9);
    EXPECT_NEAR(s5.hex_pitch_mm, 8.0, 1e-9);
    EXPECT_NEAR(s8.hex_pitch_mm, 12.0, 1e-9);

    auto bad = skill::perforated_grille_pattern::grilleSpecFor("HEX-99");
    EXPECT_NEAR(bad.hole_dia_mm, 0.0, 1e-9);

    // hex grid count for a 60×30 mm footprint, pitch 8:
    //   rowStep = 8 × √3/2 ≈ 6.928, nRows ≈ 30/6.928 + 1 = 5 rows.
    //   nCols   = 60/8 + 1 = 8 cols.
    // Hex pattern alternates offset so the count is ~5×8 = 40-ish; just
    // assert it returns at least 4 (the DFM threshold).
    const int n = skill::perforated_grille_pattern::hexHoleCount(60.0, 30.0, 8.0);
    EXPECT_GE(n, 4);

    // For 0 length: zero holes.
    EXPECT_EQ(skill::perforated_grille_pattern::hexHoleCount(0.0, 30.0, 8.0), 0);
}
