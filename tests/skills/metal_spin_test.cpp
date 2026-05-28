// @lat: [[process/test-strategy#skill round-trip]]
//
// metal_spin — disc-into-shell spinning over a polyline (z, r) profile.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/metal_spin.hpp"

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

// ── 1. ApplyHandlesInput — produces a non-null shell from a 3-point profile ─
TEST(SkillMetalSpin, ApplyHandlesInput)
{
    auto stock = skill::createCylindricalStock(60.0, 2.0);  // disc Ø60 × 2 mm

    skill::metal_spin::Input in;
    in.blank_thickness_mm = 2.0;
    in.profile = {
        { /*z*/ 0.0,  /*r*/ 25.0 },
        { /*z*/ 5.0,  /*r*/ 20.0 },
        { /*z*/ 12.0, /*r*/ 15.0 },
    };

    auto out = skill::metal_spin::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_GT(volumeOf(out.workpiece->shape()), 0.0);
    EXPECT_EQ(out.signature.skill_id, std::string("metal_spin"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ── 2. ValidateRejectsBadInput — too few points & z non-monotonic ───────────
TEST(SkillMetalSpin, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(60.0, 2.0);

    // Case A: only 1 point.
    {
        skill::metal_spin::Input in;
        in.blank_thickness_mm = 1.0;
        in.profile = { { 0.0, 20.0 } };
        auto r = skill::metal_spin::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::metal_spin::apply(*stock, in), skill::SkillError);
    }

    // Case B: z not strictly increasing.
    {
        skill::metal_spin::Input in;
        in.blank_thickness_mm = 1.0;
        in.profile = {
            { 0.0,  25.0 },
            { 5.0,  20.0 },
            { 5.0,  15.0 },        // duplicate z — invalid
        };
        auto r = skill::metal_spin::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-INPUT") { found = true; break; }
        EXPECT_TRUE(found);
    }
}

// ── 3. SignatureRecordsKind + key params (wall_thinning_pct present) ───────
TEST(SkillMetalSpin, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCylindricalStock(60.0, 2.0);

    skill::metal_spin::Input in;
    in.blank_thickness_mm = 2.0;
    in.profile = {
        { 0.0,  25.0 },
        { 10.0, 18.0 },
        { 20.0, 12.0 },
    };

    auto out = skill::metal_spin::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("metal_spin"));
    EXPECT_EQ(out.signature.pattern["point_count"].get<int>(), 3);
    EXPECT_NEAR(out.signature.pattern["blank_thickness_mm"].get<double>(),
                2.0, 1e-9);
    EXPECT_TRUE(out.signature.pattern.contains("wall_thinning_pct"));
    EXPECT_GT(out.signature.pattern["wall_thinning_pct"].get<double>(), 0.0);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("spin_roller"));
}

// ── 4. RecognizeMetadataReplay — exact polyline recovery ───────────────────
TEST(SkillMetalSpin, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(60.0, 2.0);

    skill::metal_spin::Input in;
    in.blank_thickness_mm = 1.5;
    in.profile = {
        { 0.0,  20.0 },
        { 8.0,  16.0 },
        { 18.0, 12.0 },
    };

    auto out = skill::metal_spin::apply(*stock, in);
    auto cands = skill::metal_spin::recognize(*out.workpiece);

    ASSERT_GE(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["profile"].size(), 3u);
    EXPECT_NEAR(cands[0].recovered_params["profile"][0]["r_mm"].get<double>(),
                20.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["blank_thickness_mm"].get<double>(),
                1.5, 1e-9);
}

// ── 5. SPECIFIC: thinning detection on a steep profile ─────────────────────
TEST(SkillMetalSpin, ThinningDetectedOnSteepProfile)
{
    auto stock = skill::createCylindricalStock(60.0, 2.0);

    // Steep radial swing (mostly Δr, small Δz) → high implied thinning %.
    skill::metal_spin::Input in;
    in.blank_thickness_mm = 1.0;
    in.profile = {
        { 0.0,  28.0 },
        { 0.5,  20.0 },
        { 1.0,  10.0 },        // wall mostly radial → high thinning
    };

    auto r = skill::metal_spin::validate(*stock, in);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SPIN-THIN") { found = true; break; }
    EXPECT_TRUE(found);
}
