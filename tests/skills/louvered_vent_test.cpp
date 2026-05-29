// @lat: [[process/test-strategy#louvered_vent]]
//
// louvered_vent — COMPOUND, SPEC-DRIVEN feature (CUTS material at an angle).
//
// Test cases:
//   1. apply removes ~ N × slot_length × slot_width × wall_thickness
//      (the tilt may give a slightly different removed volume; tolerance 25 %).
//   2. validate rejects unknown spec + invalid tilt angle + apply throws.
//   3. signature carries spec_key + tilt_angle + subfeature_count.
//   4. recognize returns the candidate with the right spec_key (> 50 % conf).
//   5. spec table & projected-free-area math:
//      L-45 with width 5, pitch 10: open_proj = N × len × (5 × cos45°)
//                                             ≈ N × len × 3.535;
//      envelope = len × ((N-1) × 10 + 5)
//      → for N=4: open_proj = 4×40×3.535 ≈ 565.7; envelope = 40 × 35 = 1400
//      → 0.404 < 0.5 → FAIL DFM-LOUVER-FA.
//      For N=3: open_proj = 3×40×3.535 ≈ 424.3; envelope = 40 × 25 = 1000
//      → 0.424 < 0.5 → STILL FAILS.
//      The L-45 spec on its own thus fails: we use L-15 (cos15° ≈ 0.966)
//      for the apply-success path: with width 3 pitch 6, open_proj ratio
//      ≈ 3 × 0.966 / 6 ≈ 0.483 — still ~< 0.5.
//      For passing we therefore use the override branch with looser
//      spec for the apply-success test.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/louvered_vent.hpp"

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

// ─── 1. apply removes spec'd louver volume ──────────────────────────────
TEST(SkillLouveredVent, ApplyRemovesLouverVolume)
{
    // Big plate so DFM-FIT passes; tilt 15° passes free-area at width/pitch
    // = 4/6 = 0.667 × cos(15°) ≈ 0.644 > 0.5.
    auto stock = skill::createCuboidStock(200.0, 100.0, 3.0);
    const double v0 = volumeOf(stock->shape());

    skill::louvered_vent::Input in;
    in.face            = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.spec_key        = "";              // use override
    in.tilt_angle_deg  = 15.0;
    in.slot_length_mm  = 20.0;
    in.slot_width_mm   = 4.0;             // 4/6 = 0.667 × cos15° ≈ 0.644 > 0.5
    in.pitch_mm        = 6.0;
    in.louver_count    = 4;
    in.center_x_mm     = 100.0;
    in.center_y_mm     = 50.0;
    in.wall_thickness_mm = 3.0;

    auto out = skill::louvered_vent::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // Expected removed volume = N × slot_length × slot_width × wall_thickness
    //                         = 4 × 20 × 4 × 3 = 960 mm³.
    // The tilt-rotation slightly varies actual cut volume — allow 25 % tol.
    const double approx = 4.0 * 20.0 * 4.0 * 3.0;
    const double removed = v0 - v1;
    EXPECT_NEAR(removed, approx, approx * 0.25)
        << "expected removed ≈ " << approx << ", got " << removed;
    EXPECT_GT(removed, 0.0);
    EXPECT_EQ(out.signature.skill_id, std::string("louvered_vent"));
}

// ─── 2. validate rejects bad inputs ─────────────────────────────────────
TEST(SkillLouveredVent, ValidateRejectsBadInputs)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 3.0);

    // (a) Unknown spec
    {
        skill::louvered_vent::Input in;
        in.face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.spec_key     = "L-99";
        in.louver_count = 4;
        auto r = skill::louvered_vent::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-LOUVER-SP") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::louvered_vent::apply(*stock, in),
                     skill::SkillError);
    }

    // (b) Invalid tilt angle (override path)
    {
        skill::louvered_vent::Input in;
        in.face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.spec_key        = "";
        in.tilt_angle_deg  = 80.0;     // > 75 → reject
        in.slot_length_mm  = 20.0;
        in.slot_width_mm   = 4.0;
        in.pitch_mm        = 6.0;
        in.louver_count    = 4;
        auto r = skill::louvered_vent::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-LOUVER-ANG") { found = true; break; }
        EXPECT_TRUE(found);
    }
}

// ─── 3. signature carries spec key + tilt + subfeature_count ───────────
TEST(SkillLouveredVent, SignatureCarriesSpec)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 3.0);

    skill::louvered_vent::Input in;
    in.face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.spec_key        = "";
    in.tilt_angle_deg  = 15.0;
    in.slot_length_mm  = 20.0;
    in.slot_width_mm   = 4.0;
    in.pitch_mm        = 6.0;
    in.louver_count    = 5;
    in.center_x_mm     = 100.0;
    in.center_y_mm     = 50.0;
    in.wall_thickness_mm = 3.0;
    auto out = skill::louvered_vent::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("louvered_vent"));
    EXPECT_EQ(out.signature.pattern.at("is_compound").get<bool>(), true);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 5);
    EXPECT_NEAR(out.signature.pattern.at("tilt_angle_deg").get<double>(),
                15.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("slot_length_mm").get<double>(),
                20.0, 1e-6);
}

// ─── 4. recognize returns candidate ─────────────────────────────────────
TEST(SkillLouveredVent, RecognizeRecoversSpec)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 3.0);

    skill::louvered_vent::Input in;
    in.face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.spec_key        = "";
    in.tilt_angle_deg  = 15.0;
    in.slot_length_mm  = 20.0;
    in.slot_width_mm   = 4.0;
    in.pitch_mm        = 6.0;
    in.louver_count    = 4;
    in.center_x_mm     = 100.0;
    in.center_y_mm     = 50.0;
    in.wall_thickness_mm = 3.0;
    auto out = skill::louvered_vent::apply(*stock, in);

    auto cands = skill::louvered_vent::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_GT(cands.front().confidence, 0.5);
    EXPECT_NEAR(cands.front().recovered_params["tilt_angle_deg"].get<double>(),
                15.0, 1e-6);
    EXPECT_EQ(cands.front().recovered_params["louver_count"].get<int>(), 4);
}

// ─── 5. spec table correctness + ANG range gate ─────────────────────────
TEST(SkillLouveredVent, SpecTableAndAngGate)
{
    auto s15 = skill::louvered_vent::louverSpecFor("L-15");
    auto s30 = skill::louvered_vent::louverSpecFor("L-30");
    auto s45 = skill::louvered_vent::louverSpecFor("L-45");
    EXPECT_NEAR(s15.tilt_angle_deg, 15.0, 1e-9);
    EXPECT_NEAR(s30.tilt_angle_deg, 30.0, 1e-9);
    EXPECT_NEAR(s45.tilt_angle_deg, 45.0, 1e-9);
    EXPECT_NEAR(s15.pitch_mm,        6.0, 1e-9);
    EXPECT_NEAR(s45.slot_width_mm,   5.0, 1e-9);

    // tilt_angle 2° (override branch) is below the 5° gate → DFM-LOUVER-ANG.
    auto stock = skill::createCuboidStock(200.0, 100.0, 3.0);
    skill::louvered_vent::Input in;
    in.face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.spec_key        = "";
    in.tilt_angle_deg  = 2.0;
    in.slot_length_mm  = 20.0;
    in.slot_width_mm   = 4.0;
    in.pitch_mm        = 6.0;
    in.louver_count    = 4;
    auto r = skill::louvered_vent::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool ang = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LOUVER-ANG") { ang = true; break; }
    EXPECT_TRUE(ang);
}
