// @lat: [[process/test-strategy#skill round-trip]]
//
// parametric_rib_array — REAL-behavior tests for the spec-driven rib array.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/parametric_rib_array.hpp"

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

skill::parametric_rib_array::Input goodInput()
{
    skill::parametric_rib_array::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.spec_class   = "mold_standard";        // thickness 1.2, max H/T 3
    in.rib_count    = 4;
    in.pitch_mm     = 6.0;                    // 5× thickness — well above 2T
    in.length_mm    = 20.0;
    in.height_mm    = 3.0;                    // H/T = 2.5 < 3
    in.thickness_mm = 0.0;                    // use spec default 1.2
    in.anchor_x_mm  = 25.0;
    in.anchor_y_mm  = 25.0;
    in.array_axis   = gp_Dir(0, 1, 0);        // pitch along Y
    in.extrude_dir  = gp_Dir(0, 0, 1);
    return in;
}

}  // namespace

// ─── Test 1: apply produces a real volume delta matching the spec ────────
TEST(SkillParametricRibArray, ApplyAddsRealVolumeMatchingSpec)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());

    auto in = goodInput();
    const double V0 = volumeOf(stock->shape());

    auto out = skill::parametric_rib_array::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Spec thickness for "mold_standard" = 1.2 mm.
    const double specThick = skill::parametric_rib_array::specThicknessFor("mold_standard");
    EXPECT_NEAR(specThick, 1.2, 1e-9);

    // Expected ADDED volume = N × L × T × H = 4 × 20 × 1.2 × 3 = 288 mm³.
    const double expected = in.rib_count * in.length_mm * specThick * in.height_mm;
    const double delta = volumeOf(out.workpiece->shape()) - V0;
    EXPECT_NEAR(delta, expected, expected * 0.05)
        << "rib array should add N × L × T × H volume";

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("parametric_rib_array"));
}

// ─── Test 2: validate rejects unknown spec key + auto-detect failure ─────
TEST(SkillParametricRibArray, ValidateRejectsUnknownSpecAndAutoDetectFailure)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // (a) Unknown spec_class
    {
        auto in = goodInput();
        in.spec_class = "M99";   // not in {mold_thin, mold_standard, …}
        auto r = skill::parametric_rib_array::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-RIB-UNKNOWN") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::parametric_rib_array::apply(*stock, in), skill::SkillError);
    }

    // (b) Auto-detect failure on rib spacing — pitch < 2 × thickness
    {
        auto in = goodInput();
        in.pitch_mm = 1.0;       // 1.0 < 2 × 1.2 = 2.4
        auto r = skill::parametric_rib_array::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-RIB-SPACING") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::parametric_rib_array::apply(*stock, in), skill::SkillError);
    }
}

// ─── Test 3: signature carries spec_class + derived params ───────────────
TEST(SkillParametricRibArray, SignatureCarriesSpecKeyAndParams)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto out = skill::parametric_rib_array::apply(*stock, goodInput());

    const auto& sig = out.signature;
    EXPECT_EQ(sig.params.value("spec_class", std::string()), "mold_standard");
    EXPECT_EQ(sig.pattern.value("kind", std::string()), "parametric_rib_array");
    EXPECT_TRUE(sig.pattern.value("is_compound", false));
    EXPECT_EQ(sig.pattern.value("subfeature_count", 0), 4);
    EXPECT_EQ(sig.pattern.value("rib_count",        0), 4);
    EXPECT_NEAR(sig.pattern.value("thickness_mm",   0.0), 1.2, 1e-6);
    EXPECT_NEAR(sig.pattern.value("pitch_mm",       0.0), 6.0, 1e-6);
    EXPECT_NEAR(sig.pattern.value("spec_thickness_mm", 0.0), 1.2, 1e-6);

    // ribs[] array has 4 entries
    ASSERT_TRUE(sig.pattern.contains("ribs"));
    EXPECT_EQ(sig.pattern["ribs"].size(), 4u);
}

// ─── Test 4: recognize returns the candidate with the right spec ──────────
TEST(SkillParametricRibArray, RecognizeReturnsCandidateWithSpecKey)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto out = skill::parametric_rib_array::apply(*stock, goodInput());

    auto cands = skill::parametric_rib_array::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("parametric_rib_array"));
    EXPECT_GT(c.confidence, 0.5);
    EXPECT_EQ(c.recovered_params.value("spec_class", std::string()), "mold_standard");
    EXPECT_EQ(c.recovered_params.value("rib_count",  0), 4);
}

// ─── Test 5: spec table lookup correctness ────────────────────────────────
TEST(SkillParametricRibArray, SpecTableLookupCorrectness)
{
    // All five canonical spec classes resolve to expected thickness:
    EXPECT_NEAR(skill::parametric_rib_array::specThicknessFor("mold_thin"),     0.8, 1e-9);
    EXPECT_NEAR(skill::parametric_rib_array::specThicknessFor("mold_standard"), 1.2, 1e-9);
    EXPECT_NEAR(skill::parametric_rib_array::specThicknessFor("mold_heavy"),    2.0, 1e-9);
    EXPECT_NEAR(skill::parametric_rib_array::specThicknessFor("structural"),    3.0, 1e-9);
    EXPECT_NEAR(skill::parametric_rib_array::specThicknessFor("air_flow_fin"),  0.6, 1e-9);
    EXPECT_NEAR(skill::parametric_rib_array::specThicknessFor("UNKNOWN"),       0.0, 1e-9);

    // Max H/T ratios:
    EXPECT_NEAR(skill::parametric_rib_array::specMaxHTRatioFor("mold_standard"), 3.0, 1e-9);
    EXPECT_NEAR(skill::parametric_rib_array::specMaxHTRatioFor("air_flow_fin"),  8.0, 1e-9);
}
