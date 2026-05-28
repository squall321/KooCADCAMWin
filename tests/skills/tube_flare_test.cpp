// @lat: [[process/test-strategy#skill round-trip]]
//
// tube_flare — expand one tube end's OD via rebuild as larger cylinder.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/tube_flare.hpp"

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

// ─── 1. ApplyHandlesInput — flaring adds volume in the flare zone ────────
TEST(SkillTubeFlare, ApplyHandlesInput)
{
    auto stock = skill::createCylindricalStock(20.0, 30.0);   // Ø20, length 30
    ASSERT_FALSE(stock->shape().IsNull());

    const double volBefore = volumeOf(stock->shape());

    skill::tube_flare::Input in;
    in.start_z_mm   = 20.0;
    in.end_z_mm     = 30.0;
    in.target_od_mm = 26.0;       // Ø20 → Ø26 over the last 10 mm

    auto out = skill::tube_flare::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());

    // Flaring (rebuild) increases material volume.
    EXPECT_GT(volAfter, volBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("tube_flare"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. ValidateRejectsBadInput — target_od <= current OD is invalid ─────
TEST(SkillTubeFlare, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(20.0, 30.0);

    skill::tube_flare::Input in;
    in.start_z_mm   = 20.0;
    in.end_z_mm     = 30.0;
    in.target_od_mm = 18.0;       // smaller — invalid for flaring

    auto r = skill::tube_flare::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-TUBE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::tube_flare::apply(*stock, in), skill::SkillError);
}

// ─── 3. SignatureRecordsKind + key params ────────────────────────────────
TEST(SkillTubeFlare, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCylindricalStock(20.0, 30.0);

    skill::tube_flare::Input in;
    in.start_z_mm   = 22.0;
    in.end_z_mm     = 30.0;
    in.target_od_mm = 25.0;
    auto out = skill::tube_flare::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("tube_flare"));
    EXPECT_NEAR(out.signature.pattern["target_od_mm"].get<double>(),
                25.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["start_z_mm"].get<double>(),
                22.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["end_z_mm"].get<double>(),
                30.0, 1e-6);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("flare_mandrel"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 4. RecognizeMetadataReplay — recognise via feature history ──────────
TEST(SkillTubeFlare, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(20.0, 30.0);

    skill::tube_flare::Input in;
    in.start_z_mm   = 22.0;
    in.end_z_mm     = 30.0;
    in.target_od_mm = 26.0;
    auto out = skill::tube_flare::apply(*stock, in);

    auto cands = skill::tube_flare::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    bool matched = false;
    for (const auto& c : cands) {
        const double od = c.recovered_params["target_od_mm"].get<double>();
        if (std::abs(od - in.target_od_mm) < 0.3) {
            EXPECT_GT(c.confidence, 0.7);
            matched = true;
            break;
        }
    }
    EXPECT_TRUE(matched);
}

// ─── 5. Specific: expansion ratio captured in signature tooling extra ────
TEST(SkillTubeFlare, ExpansionRatioInToolingExtra)
{
    auto stock = skill::createCylindricalStock(20.0, 30.0);   // R_init = 10

    skill::tube_flare::Input in;
    in.start_z_mm   = 22.0;
    in.end_z_mm     = 30.0;
    in.target_od_mm = 26.0;                     // R_target = 13 → ratio 0.30
    auto out = skill::tube_flare::apply(*stock, in);

    ASSERT_TRUE(out.signature.tooling.extra.contains("expansion_ratio"));
    const double ratio = out.signature.tooling.extra["expansion_ratio"].get<double>();
    EXPECT_NEAR(ratio, 0.30, 0.02);
}
