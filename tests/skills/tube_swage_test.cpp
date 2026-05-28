// @lat: [[process/test-strategy#skill round-trip]]
//
// tube_swage — reduce one tube end's OD via rebuild as smaller cylinder.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/tube_swage.hpp"

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

// ─── 1. ApplyHandlesInput — swaging reduces volume in the swage zone ─────
TEST(SkillTubeSwage, ApplyHandlesInput)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);  // Ø40, length 30
    ASSERT_FALSE(stock->shape().IsNull());

    const double volBefore = volumeOf(stock->shape());

    skill::tube_swage::Input in;
    in.start_z_mm   = 20.0;     // swage upper portion
    in.end_z_mm     = 30.0;
    in.target_od_mm = 28.0;     // Ø40 → Ø28 over the last 10 mm

    auto out = skill::tube_swage::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());

    // Swaged section has less material than the original 10 mm slab.
    EXPECT_LT(volAfter, volBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("tube_swage"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. ValidateRejectsBadInput — target_od >= current OD is invalid ─────
TEST(SkillTubeSwage, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::tube_swage::Input in;
    in.start_z_mm   = 20.0;
    in.end_z_mm     = 30.0;
    in.target_od_mm = 50.0;     // larger than current OD — invalid for swage

    auto r = skill::tube_swage::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-TUBE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::tube_swage::apply(*stock, in), skill::SkillError);
}

// ─── 3. SignatureRecordsKind + key params ────────────────────────────────
TEST(SkillTubeSwage, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::tube_swage::Input in;
    in.start_z_mm   = 18.0;
    in.end_z_mm     = 30.0;
    in.target_od_mm = 30.0;
    auto out = skill::tube_swage::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("tube_swage"));
    EXPECT_NEAR(out.signature.pattern["target_od_mm"].get<double>(),
                30.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["start_z_mm"].get<double>(),
                18.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["end_z_mm"].get<double>(),
                30.0, 1e-6);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("rotary_swage_die"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 4. RecognizeMetadataReplay — recognise via feature history ──────────
TEST(SkillTubeSwage, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::tube_swage::Input in;
    in.start_z_mm   = 20.0;
    in.end_z_mm     = 30.0;
    in.target_od_mm = 28.0;
    auto out = skill::tube_swage::apply(*stock, in);

    auto cands = skill::tube_swage::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    bool matched = false;
    for (const auto& c : cands) {
        const double od = c.recovered_params["target_od_mm"].get<double>();
        if (std::abs(od - in.target_od_mm) < 0.2) {
            EXPECT_GT(c.confidence, 0.7);
            matched = true;
            break;
        }
    }
    EXPECT_TRUE(matched);
}

// ─── 5. Specific: recognized swage Z range touches the bbox top end ──────
TEST(SkillTubeSwage, RecognizedZRangeTouchesBboxEnd)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);  // zMax = 30

    skill::tube_swage::Input in;
    in.start_z_mm   = 20.0;
    in.end_z_mm     = 30.0;
    in.target_od_mm = 28.0;
    auto out = skill::tube_swage::apply(*stock, in);

    auto cands = skill::tube_swage::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    bool endTouches = false;
    for (const auto& c : cands) {
        const double od = c.recovered_params["target_od_mm"].get<double>();
        if (std::abs(od - in.target_od_mm) >= 0.2) continue;
        const double endZ = c.recovered_params["end_z_mm"].get<double>();
        if (std::abs(endZ - 30.0) < 0.3) { endTouches = true; break; }
    }
    EXPECT_TRUE(endTouches)
        << "swage recogniser must locate the swaged section at one bbox end";
}
