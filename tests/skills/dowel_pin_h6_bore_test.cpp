// @lat: [[process/test-strategy#skill round-trip]]
//
// dowel_pin_h6_bore — 5 tests.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/dowel_pin_h6_bore.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── T1: apply removes ≈ final-dia cylinder ──────────────────────────────
TEST(SkillDowelPinH6Bore, ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::dowel_pin_h6_bore::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.axis_dir       = gp_Dir(0, 0, -1);
    in.nominal_dia_mm = 6.0;       // H6 row 6–10 → +9 µm  (also standard dowel size)
    in.depth_mm       = 10.0;
    in.chamfer_mm     = 0.2;

    auto out = skill::dowel_pin_h6_bore::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double upperDev = 9.0;   // µm
    const double finalDia = in.nominal_dia_mm + (upperDev * 1e-3) / 2.0;
    const double rF = finalDia / 2.0;
    const double approx = M_PI * rF * rF * in.depth_mm;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.20);
    EXPECT_EQ(out.signature.skill_id, std::string("dowel_pin_h6_bore"));
}

// ─── T2: validate rejects unknown spec_key + apply throws ────────────────
TEST(SkillDowelPinH6Bore, ValidateRejectsUnknownSpecKey)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::dowel_pin_h6_bore::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.nominal_dia_mm = 6.0;
    in.depth_mm       = 8.0;
    in.spec_key       = "H7";    // wrong

    auto r = skill::dowel_pin_h6_bore::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SPEC") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::dowel_pin_h6_bore::apply(*stock, in), skill::SkillError);
}

// ─── T3: signature carries H6 + is_compound=true ─────────────────────────
TEST(SkillDowelPinH6Bore, SignatureCarriesSpecAndCompoundFlag)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::dowel_pin_h6_bore::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.nominal_dia_mm = 6.0;
    in.depth_mm       = 8.0;

    auto out = skill::dowel_pin_h6_bore::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("dowel_pin_h6_bore"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["spec_key"].get<std::string>(), std::string("H6"));
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 3);
}

// ─── T4: recognize returns ≥1 H6 candidate ───────────────────────────────
TEST(SkillDowelPinH6Bore, RecognizeFindsH6Candidate)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::dowel_pin_h6_bore::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.nominal_dia_mm = 6.0;
    in.depth_mm       = 10.0;
    in.chamfer_mm     = 0.0;

    auto out = skill::dowel_pin_h6_bore::apply(*stock, in);
    auto cands = skill::dowel_pin_h6_bore::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    bool foundH6 = false;
    for (const auto& c : cands) {
        if (c.skill_id != "dowel_pin_h6_bore") continue;
        if (c.recovered_params.value("spec_key", std::string()) == "H6") {
            foundH6 = true;
            EXPECT_GT(c.confidence, 0.7);
        }
    }
    EXPECT_TRUE(foundH6);
}

// ─── T5: standard dowel-size detection + H6 table values ─────────────────
TEST(SkillDowelPinH6Bore, StandardDowelAndSpecTable)
{
    // Standard DIN 7 sizes accepted.
    EXPECT_TRUE(skill::dowel_pin_h6_bore::isStandardDowelSize(3.0));
    EXPECT_TRUE(skill::dowel_pin_h6_bore::isStandardDowelSize(6.0));
    EXPECT_TRUE(skill::dowel_pin_h6_bore::isStandardDowelSize(10.0));
    // Non-standard size rejected (warning, but flagged here).
    EXPECT_FALSE(skill::dowel_pin_h6_bore::isStandardDowelSize(7.5));
    // H6 table: Ø 6 → 9, Ø 18 → 11, Ø 50 → 16 µm.
    EXPECT_NEAR(skill::dowel_pin_h6_bore::upperDeviationMicronsH6(6.0),   9.0, 0.01);
    EXPECT_NEAR(skill::dowel_pin_h6_bore::upperDeviationMicronsH6(18.0), 11.0, 0.01);
    EXPECT_NEAR(skill::dowel_pin_h6_bore::upperDeviationMicronsH6(50.0), 16.0, 0.01);
}
