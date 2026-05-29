// @lat: [[process/test-strategy#skill round-trip]]
//
// locating_g6_bore — 5 tests.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/locating_g6_bore.hpp"

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
TEST(SkillLocatingG6Bore, ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::locating_g6_bore::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 30.0;
    in.axis_dir       = gp_Dir(0, 0, -1);
    in.nominal_dia_mm = 18.0;     // G6 row 10–18: upper +17, lower +6 µm
    in.depth_mm       = 10.0;
    in.chamfer_mm     = 0.3;

    auto out = skill::locating_g6_bore::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // finalDia = 18 + (17+6)/2 µm = 18 + 0.01150 mm.
    const double midDevMm = ((17.0 + 6.0) / 2.0) * 1e-3;
    const double finalDia = in.nominal_dia_mm + midDevMm;
    const double rF = finalDia / 2.0;
    const double approx = M_PI * rF * rF * in.depth_mm;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.20);
    EXPECT_EQ(out.signature.skill_id, std::string("locating_g6_bore"));
}

// ─── T2: validate rejects unknown spec_key + apply throws ────────────────
TEST(SkillLocatingG6Bore, ValidateRejectsUnknownSpecKey)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::locating_g6_bore::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.nominal_dia_mm = 10.0;
    in.depth_mm       = 5.0;
    in.spec_key       = "P7";   // wrong

    auto r = skill::locating_g6_bore::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SPEC") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::locating_g6_bore::apply(*stock, in), skill::SkillError);
}

// ─── T3: signature carries G6 + is_compound=true + context info ──────────
TEST(SkillLocatingG6Bore, SignatureCarriesSpecAndContext)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::locating_g6_bore::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.nominal_dia_mm = 10.0;
    in.depth_mm       = 5.0;

    auto out = skill::locating_g6_bore::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("locating_g6_bore"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["spec_key"].get<std::string>(), std::string("G6"));
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 3);
    // Context detection: cuboid stock has 6 planar faces.
    ASSERT_TRUE(out.signature.params.contains("ctx_planar_faces"));
    EXPECT_GE(out.signature.params["ctx_planar_faces"].get<int>(), 6);
}

// ─── T4: recognize returns ≥1 G6 candidate ───────────────────────────────
TEST(SkillLocatingG6Bore, RecognizeFindsG6Candidate)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::locating_g6_bore::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 30.0;
    in.nominal_dia_mm = 18.0;
    in.depth_mm       = 10.0;
    in.chamfer_mm     = 0.0;

    auto out = skill::locating_g6_bore::apply(*stock, in);
    auto cands = skill::locating_g6_bore::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    bool foundG6 = false;
    for (const auto& c : cands) {
        if (c.skill_id != "locating_g6_bore") continue;
        if (c.recovered_params.value("spec_key", std::string()) == "G6") {
            foundG6 = true;
            EXPECT_GT(c.confidence, 0.65);
        }
    }
    EXPECT_TRUE(foundG6);
}

// ─── T5: G6 table values match ISO 286-1 ─────────────────────────────────
TEST(SkillLocatingG6Bore, SpecTableMatchesIsoTable)
{
    // G6: Ø 10 → upper +14, lower +5 µm.
    auto d10 = skill::locating_g6_bore::deviationFor(10.0);
    EXPECT_TRUE(d10.valid);
    EXPECT_NEAR(d10.upper_um, 14.0, 0.01);
    EXPECT_NEAR(d10.lower_um,  5.0, 0.01);
    // G6: Ø 18 → upper +17, lower +6 µm (row 10–18).
    auto d18 = skill::locating_g6_bore::deviationFor(18.0);
    EXPECT_TRUE(d18.valid);
    EXPECT_NEAR(d18.upper_um, 17.0, 0.01);
    EXPECT_NEAR(d18.lower_um,  6.0, 0.01);
    // Out of range.
    auto d999 = skill::locating_g6_bore::deviationFor(999.0);
    EXPECT_FALSE(d999.valid);
}
