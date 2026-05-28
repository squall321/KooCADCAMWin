// @lat: [[process/test-strategy#skill round-trip]]
//
// ceramic_sinter skill — metadata-only firing of green ceramic body.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/ceramic_sinter.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. Apply handles input + geometry unchanged ──────────────────────────
TEST(SkillCeramicSinter, ApplyHandlesInput)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 5.0, "alumina_99");
    const double volBefore = volumeOf(stock->shape());

    skill::ceramic_sinter::Input in;
    in.temp_peak_c   = 1500.0;
    in.dwell_min     = 120.0;
    in.atmosphere    = "air";
    in.shrinkage_pct = 18.0;

    auto out = skill::ceramic_sinter::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.signature.skill_id, std::string("ceramic_sinter"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate rejects bad input (bad atmosphere) ───────────────────────
TEST(SkillCeramicSinter, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 5.0, "alumina_99");

    skill::ceramic_sinter::Input in;
    in.temp_peak_c   = 1500.0;
    in.dwell_min     = 120.0;
    in.atmosphere    = "plasma";       // not in {air, argon, nitrogen, vacuum, hydrogen}
    in.shrinkage_pct = 18.0;

    auto r = skill::ceramic_sinter::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SINTER-ATMOS") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::ceramic_sinter::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillCeramicSinter, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 5.0, "zirconia");

    skill::ceramic_sinter::Input in;
    in.temp_peak_c   = 1450.0;
    in.dwell_min     = 90.0;
    in.atmosphere    = "argon";
    in.shrinkage_pct = 20.0;

    auto out = skill::ceramic_sinter::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("ceramic_sinter"));
    EXPECT_NEAR(out.signature.pattern["temp_peak_c"].get<double>(), 1450.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["dwell_min"].get<double>(), 90.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["atmosphere"].get<std::string>(), "argon");
    EXPECT_NEAR(out.signature.pattern["shrinkage_pct"].get<double>(), 20.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("kiln"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillCeramicSinter, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 5.0, "silicon_carbide");

    skill::ceramic_sinter::Input in;
    in.temp_peak_c   = 1900.0;
    in.dwell_min     = 60.0;
    in.atmosphere    = "argon";
    in.shrinkage_pct = 10.0;

    auto out = skill::ceramic_sinter::apply(*stock, in);
    auto cands = skill::ceramic_sinter::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["temp_peak_c"].get<double>(), 1900.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::ceramic_sinter::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: green-to-fired scale factor = 1 / (1 - shrink) ──────────
TEST(SkillCeramicSinter, GreenToFiredScaleFactorComputed)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 4.0, "alumina_99");

    skill::ceramic_sinter::Input in;
    in.temp_peak_c   = 1600.0;
    in.dwell_min     = 120.0;
    in.atmosphere    = "air";
    in.shrinkage_pct = 20.0;        // → green = fired × 1.25

    auto out = skill::ceramic_sinter::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["green_to_fired_scale_factor"].get<double>(),
                1.0 / 0.80, 1e-6);
}
