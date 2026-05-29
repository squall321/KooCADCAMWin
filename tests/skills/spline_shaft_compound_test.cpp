// @lat: [[process/test-strategy#real compound feature]]
//
// spline_shaft_compound — REAL compound feature on cylindrical shaft:
// chamfer + N axial spline grooves per DIN 5480.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/spline_shaft_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::spline_shaft_compound::Input goodInput()
{
    skill::spline_shaft_compound::Input in;
    in.shaft_dia_mm      = 30.0;       // > major_dia = 25 + 0.9*1 = 25.9
    in.shaft_length_mm   = 80.0;
    in.module_mm         = 1.0;
    in.num_splines       = 25;          // major_dia = 25.9
    in.spline_length_mm  = 30.0;
    in.end_chamfer_mm    = 1.0;
    return in;
}

}  // namespace

// ─── 1. apply() — REAL volume delta + face count growth ──────────────────
TEST(SkillSplineShaftCompound, ApplyRemovesRealGeometry)
{
    auto stock = skill::createCylindricalStock(30.0, 80.0);
    const double vStock = volumeOf(stock->shape());
    const int    fStock = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::spline_shaft_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double vShaft = volumeOf(out.workpiece->shape());
    EXPECT_LT(vShaft, vStock);

    // Expected delta = N splines * tooth_w * depth * spline_length (chamfer
    // contribution is small).
    const int N = in.num_splines;
    const double m = in.module_mm;
    const double major_r = (m * N + 0.9 * m) / 2.0;
    const double minor_r = (m * N - 1.1 * m) / 2.0;
    const double depth = major_r - minor_r;
    const double tooth_w = 0.5 * M_PI * m / 2.0;
    const double expected_splines = N * tooth_w * depth * in.spline_length_mm;
    const double delta = vStock - vShaft;
    EXPECT_GT(delta, expected_splines * 0.5);
    // chamfer volume bound: ~2 * pi * shaft_r * chamfer^2
    EXPECT_LT(delta, expected_splines * 4.0 + 100.0);

    EXPECT_GT(out.workpiece->faceCount(), fStock + N);
}

// ─── 2. validate() rejects bad input ─────────────────────────────────────
TEST(SkillSplineShaftCompound, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(30.0, 80.0);

    auto in = goodInput();
    in.num_splines = 100;             // > 82 DIN max
    auto rep = skill::spline_shaft_compound::validate(*stock, in);
    EXPECT_FALSE(rep.passed);
    bool found = false;
    for (const auto& f : rep.findings) if (f.code == "DFM-SPLINE-N") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::spline_shaft_compound::apply(*stock, in),
                 skill::SkillError);

    // Bad module
    in = goodInput();
    in.module_mm = 0.97;            // not in DIN 5480 set
    rep = skill::spline_shaft_compound::validate(*stock, in);
    EXPECT_FALSE(rep.passed);
}

// ─── 3. signature is_compound ────────────────────────────────────────────
TEST(SkillSplineShaftCompound, SignatureIsCompound)
{
    auto stock = skill::createCylindricalStock(30.0, 80.0);
    auto in    = goodInput();
    auto out   = skill::spline_shaft_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("spline_shaft_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("num_splines").get<int>(), 25);
    EXPECT_EQ(out.signature.pattern.at("standard").get<std::string>(),
              std::string("DIN 5480"));
}

// ─── 4. recognize() returns ≥1 candidate ─────────────────────────────────
TEST(SkillSplineShaftCompound, RecognizeFindsCandidate)
{
    auto stock = skill::createCylindricalStock(30.0, 80.0);
    auto in    = goodInput();
    auto out   = skill::spline_shaft_compound::apply(*stock, in);

    auto cands = skill::spline_shaft_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());

    bool found_replay = false;
    for (const auto& c : cands) {
        if (std::abs(c.confidence - 1.0) < 1e-9) { found_replay = true; break; }
    }
    EXPECT_TRUE(found_replay);
}

// ─── 5. Feature-specific: major_dia = ref_dia + 0.9 m, minor_dia = ref_dia - 1.1 m ───
TEST(SkillSplineShaftCompound, DimensionsMatchDIN5480)
{
    auto stock = skill::createCylindricalStock(30.0, 80.0);
    auto in    = goodInput();
    auto out   = skill::spline_shaft_compound::apply(*stock, in);

    const double refDia = in.module_mm * static_cast<double>(in.num_splines);
    EXPECT_NEAR(out.signature.pattern.at("reference_dia_mm").get<double>(),
                refDia, 1e-9);
    EXPECT_NEAR(out.signature.pattern.at("major_dia_mm").get<double>(),
                refDia + 0.9 * in.module_mm, 1e-9);
    EXPECT_NEAR(out.signature.pattern.at("minor_dia_mm").get<double>(),
                refDia - 1.1 * in.module_mm, 1e-9);
}
