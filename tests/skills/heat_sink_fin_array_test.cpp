// @lat: [[process/test-strategy#heat_sink_fin_array]]
//
// heat_sink_fin_array — COMPOUND, SPEC-DRIVEN feature (ADDS material).
//
// Test cases:
//   1. apply ADDS fins of the right volume (spec'd dims × fin_count).
//   2. validate rejects unknown spec key + fin_count < 2 + no-spec / no-override.
//   3. signature carries spec_key + derived params (subfeature_count == fin_count).
//   4. recognize returns the candidate with the right spec_key (> 50 % conf).
//   5. spec-table lookup returns correct numbers for every supported key.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/heat_sink_fin_array.hpp"

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

// ─── 1. apply ADDS fin volume ───────────────────────────────────────────
TEST(SkillHeatSinkFinArray, ApplyAddsFinVolume)
{
    // 200 × 100 × 10 mm base (large enough for 6 × T-50 fins).
    auto stock = skill::createCuboidStock(200.0, 100.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());
    const double v0 = volumeOf(stock->shape());

    skill::heat_sink_fin_array::Input in;
    in.base_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.spec_key         = "T-50";        // pitch 5, thk 1.5, height 15
    in.fin_count        = 6;
    in.airflow_dir      = gp_Dir(1, 0, 0);
    in.origin_x_mm      = 50.0;
    in.origin_y_mm      = 50.0;
    in.fin_length_mm    = 30.0;

    auto out = skill::heat_sink_fin_array::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());

    // Expected added volume = N × fin_length × fin_thickness × fin_height.
    const double approx = 6.0 * 30.0 * 1.5 * 15.0;       // = 4050 mm³
    const double added = v1 - v0;
    EXPECT_NEAR(added, approx, approx * 0.05)
        << "expected added volume ≈ " << approx << ", got " << added;

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("heat_sink_fin_array"));
}

// ─── 2. validate rejects bad inputs ──────────────────────────────────────
TEST(SkillHeatSinkFinArray, ValidateRejectsBadInputs)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 10.0);

    // (a) Unknown spec key
    {
        skill::heat_sink_fin_array::Input in;
        in.base_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.spec_key  = "T-99";    // unknown
        in.fin_count = 4;
        auto r = skill::heat_sink_fin_array::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-FIN-SP") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::heat_sink_fin_array::apply(*stock, in),
                     skill::SkillError);
    }

    // (b) fin_count < 2
    {
        skill::heat_sink_fin_array::Input in;
        in.base_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.spec_key  = "T-50";
        in.fin_count = 1;          // too few
        auto r = skill::heat_sink_fin_array::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-FIN-N") { found = true; break; }
        EXPECT_TRUE(found);
    }

    // (c) No spec_key AND no override → DFM-INPUT
    {
        skill::heat_sink_fin_array::Input in;
        in.base_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.spec_key  = "";
        in.fin_count = 4;
        auto r = skill::heat_sink_fin_array::validate(*stock, in);
        EXPECT_FALSE(r.passed);
    }
}

// ─── 3. signature carries spec key + derived params ─────────────────────
TEST(SkillHeatSinkFinArray, SignatureCarriesSpec)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 10.0);

    skill::heat_sink_fin_array::Input in;
    in.base_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.spec_key    = "T-80";        // pitch 8, thk 2, height 20
    in.fin_count   = 5;
    in.origin_x_mm = 30.0;
    in.origin_y_mm = 40.0;
    in.fin_length_mm = 50.0;

    auto out = skill::heat_sink_fin_array::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("heat_sink_fin_array"));
    EXPECT_EQ(out.signature.pattern.at("is_compound").get<bool>(), true);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 5);
    EXPECT_EQ(out.signature.pattern.at("spec_key").get<std::string>(),
              std::string("T-80"));
    EXPECT_NEAR(out.signature.pattern.at("fin_thickness_mm").get<double>(), 2.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("fin_height_mm").get<double>(),   20.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("fin_pitch_mm").get<double>(),     8.0, 1e-6);
}

// ─── 4. recognize finds the candidate ────────────────────────────────────
TEST(SkillHeatSinkFinArray, RecognizeRecoversSpec)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 10.0);

    skill::heat_sink_fin_array::Input in;
    in.base_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.spec_key    = "T-50";
    in.fin_count   = 4;
    in.origin_x_mm = 50.0;
    in.origin_y_mm = 40.0;
    in.fin_length_mm = 40.0;
    auto out = skill::heat_sink_fin_array::apply(*stock, in);

    auto cands = skill::heat_sink_fin_array::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_GT(cands.front().confidence, 0.5);
    EXPECT_EQ(cands.front().recovered_params["spec_key"].get<std::string>(),
              std::string("T-50"));
    EXPECT_EQ(cands.front().recovered_params["fin_count"].get<int>(), 4);
}

// ─── 5. spec table lookup correctness ────────────────────────────────────
TEST(SkillHeatSinkFinArray, SpecTableCorrectness)
{
    auto s30  = skill::heat_sink_fin_array::finSpecFor("T-30");
    auto s50  = skill::heat_sink_fin_array::finSpecFor("T-50");
    auto s80  = skill::heat_sink_fin_array::finSpecFor("T-80");
    auto s120 = skill::heat_sink_fin_array::finSpecFor("T-120");
    auto s99  = skill::heat_sink_fin_array::finSpecFor("T-99");

    EXPECT_NEAR(s30.pitch_mm,         3.0, 1e-9);
    EXPECT_NEAR(s30.fin_thickness_mm, 1.0, 1e-9);
    EXPECT_NEAR(s30.fin_height_mm,    8.0, 1e-9);

    EXPECT_NEAR(s50.pitch_mm,         5.0, 1e-9);
    EXPECT_NEAR(s50.fin_thickness_mm, 1.5, 1e-9);

    EXPECT_NEAR(s80.fin_height_mm,   20.0, 1e-9);
    EXPECT_NEAR(s120.fin_height_mm,  30.0, 1e-9);

    // All spec table entries satisfy DFM-FIN-AR (height / thickness ≤ 10).
    EXPECT_LE(s30.fin_height_mm  / s30.fin_thickness_mm,  10.0);
    EXPECT_LE(s50.fin_height_mm  / s50.fin_thickness_mm,  10.0);
    EXPECT_LE(s80.fin_height_mm  / s80.fin_thickness_mm,  10.0);
    EXPECT_LE(s120.fin_height_mm / s120.fin_thickness_mm, 10.0);

    // Override path with AR = 20 (height 40 / thickness 2) MUST fail
    // DFM-FIN-AR.
    auto stock = skill::createCuboidStock(200.0, 100.0, 10.0);
    skill::heat_sink_fin_array::Input in;
    in.base_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.spec_key         = "";
    in.fin_thickness_mm = 2.0;
    in.fin_height_mm    = 40.0;      // AR = 20 → reject
    in.fin_pitch_mm     = 5.0;
    in.fin_count        = 3;
    auto r = skill::heat_sink_fin_array::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool ar = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-FIN-AR") { ar = true; break; }
    EXPECT_TRUE(ar);

    // Unknown key → zeroed.
    EXPECT_NEAR(s99.pitch_mm, 0.0, 1e-9);
}
