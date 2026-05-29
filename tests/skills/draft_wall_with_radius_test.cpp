// @lat: [[process/test-strategy#skill round-trip]]
//
// draft_wall_with_radius — REAL-behavior tests for the drafted wall macro.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/draft_wall_with_radius.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

skill::draft_wall_with_radius::Input goodInput()
{
    skill::draft_wall_with_radius::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.spec_class        = "standard_draft";   // 3° draft, R = 0.5
    in.center_x_mm       = 25.0;
    in.center_y_mm       = 25.0;
    in.length_mm         = 20.0;
    in.base_thickness_mm = 3.0;                // > 0.4 mm
    in.height_mm         = 5.0;
    in.length_dir        = gp_Dir(1, 0, 0);
    return in;
}

}  // namespace

// ─── Test 1: apply adds a real volume matching trapezoidal prism math ────
TEST(SkillDraftWallWithRadius, ApplyAddsTrapezoidalPrismVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());

    auto in = goodInput();
    const double V0 = volumeOf(stock->shape());

    auto out = skill::draft_wall_with_radius::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double draftDeg = skill::draft_wall_with_radius::specDraftDegFor("standard_draft");
    const double rFillet  = skill::draft_wall_with_radius::specFilletRMmFor("standard_draft");
    EXPECT_NEAR(draftDeg, 3.0, 1e-9);
    EXPECT_NEAR(rFillet,  0.5, 1e-9);

    const double tan = std::tan(draftDeg * M_PI / 180.0);
    const double baseL = in.length_mm;
    const double baseT = in.base_thickness_mm;
    const double topL  = baseL + 2.0 * in.height_mm * tan;
    const double topT  = baseT + 2.0 * in.height_mm * tan;
    // Trapezoidal prism: V = H × ((baseL+topL)/2) × ((baseT+topT)/2)  (approx)
    const double expected = in.height_mm * (baseL + topL) / 2.0 * (baseT + topT) / 2.0;

    const double delta = volumeOf(out.workpiece->shape()) - V0;
    // Fillet eats a small sliver; accept 8 % tolerance.
    EXPECT_NEAR(delta, expected, expected * 0.08)
        << "drafted wall should add a trapezoidal prism volume (with fillet sliver)";
}

// ─── Test 2: validate rejects unknown spec + auto-detect failure ─────────
TEST(SkillDraftWallWithRadius, ValidateRejectsUnknownSpecAndFilletOverflow)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    // (a) Unknown spec_class
    {
        auto in = goodInput();
        in.spec_class = "M99_draft";
        auto r = skill::draft_wall_with_radius::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-DRAFT-UNKNOWN") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::draft_wall_with_radius::apply(*stock, in), skill::SkillError);
    }

    // (b) Fillet overflow auto-detected — choose aggressive_draft (R=1.5)
    //     with a thin base (1.0 mm) — 1.5 > 0.4 × 1.0 = 0.4 → DFM-FILLET-LIMIT
    {
        auto in = goodInput();
        in.spec_class        = "aggressive_draft";
        in.base_thickness_mm = 1.0;
        auto r = skill::draft_wall_with_radius::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-FILLET-LIMIT") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::draft_wall_with_radius::apply(*stock, in), skill::SkillError);
    }
}

// ─── Test 3: signature carries spec key + derived params ─────────────────
TEST(SkillDraftWallWithRadius, SignatureCarriesSpecKeyAndDerivedParams)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    auto out = skill::draft_wall_with_radius::apply(*stock, goodInput());

    const auto& sig = out.signature;
    EXPECT_EQ(sig.params.value("spec_class", std::string()), "standard_draft");
    EXPECT_EQ(sig.pattern.value("kind", std::string()), "draft_wall_with_radius");
    EXPECT_TRUE(sig.pattern.value("is_compound", false));
    EXPECT_EQ(sig.pattern.value("subfeature_count", 0), 5);
    EXPECT_NEAR(sig.pattern.value("spec_draft_deg",  0.0), 3.0, 1e-9);
    EXPECT_NEAR(sig.pattern.value("spec_fillet_r_mm", 0.0), 0.5, 1e-9);

    // Derived top width must equal base + 2H tan(3°).
    const double tan3 = std::tan(3.0 * M_PI / 180.0);
    EXPECT_NEAR(sig.pattern.value("top_thickness_mm", 0.0),
                3.0 + 2.0 * 5.0 * tan3, 1e-6);
}

// ─── Test 4: recognize returns the candidate with the right spec ──────────
TEST(SkillDraftWallWithRadius, RecognizeReturnsCandidateWithSpecKey)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    auto out = skill::draft_wall_with_radius::apply(*stock, goodInput());
    auto cands = skill::draft_wall_with_radius::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("draft_wall_with_radius"));
    EXPECT_GT(c.confidence, 0.5);
    EXPECT_EQ(c.recovered_params.value("spec_class", std::string()), "standard_draft");
}

// ─── Test 5: spec lookup correctness for ALL 4 entries ───────────────────
TEST(SkillDraftWallWithRadius, SpecTableLookupCorrectness)
{
    EXPECT_NEAR(skill::draft_wall_with_radius::specDraftDegFor("low_draft"),        1.0, 1e-9);
    EXPECT_NEAR(skill::draft_wall_with_radius::specDraftDegFor("standard_draft"),   3.0, 1e-9);
    EXPECT_NEAR(skill::draft_wall_with_radius::specDraftDegFor("high_draft"),       5.0, 1e-9);
    EXPECT_NEAR(skill::draft_wall_with_radius::specDraftDegFor("aggressive_draft"), 7.0, 1e-9);
    EXPECT_NEAR(skill::draft_wall_with_radius::specDraftDegFor("UNKNOWN"),          0.0, 1e-9);

    EXPECT_NEAR(skill::draft_wall_with_radius::specFilletRMmFor("low_draft"),        0.3, 1e-9);
    EXPECT_NEAR(skill::draft_wall_with_radius::specFilletRMmFor("standard_draft"),   0.5, 1e-9);
    EXPECT_NEAR(skill::draft_wall_with_radius::specFilletRMmFor("high_draft"),       1.0, 1e-9);
    EXPECT_NEAR(skill::draft_wall_with_radius::specFilletRMmFor("aggressive_draft"), 1.5, 1e-9);
}
