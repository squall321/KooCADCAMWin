// @lat: [[process/test-strategy#skill round-trip]]
//
// hard_turning skill — 5-case sweep covering:
//   1. apply() removes the annular volume (geometry delegated to turn_external)
//   2. validate() rejects HRC < 50 (DFM-HARDTURN-HRC)
//   3. signature carries strategy=hard_turning + CBN tooling
//   4. recognize() metadata replay via feature history
//   5. apply() with HRC > 65 throws

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/hard_turning.hpp"

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

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

}  // namespace

// ─── 1. Apply removes the annular volume (delegates geometry to turn_external)
TEST(SkillHardTurning, ApplyChangesGeometry)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0, "alloy_steel_4140");

    skill::hard_turning::Input in;
    in.start_z_mm      = 5.0;
    in.end_z_mm        = 15.0;
    in.final_dia_mm    = 32.0;
    in.roughing_passes = 2;
    in.workpiece_hrc   = 58.0;     // hardened
    in.feed_override_mm = 0.0;     // default 0.05

    auto out = skill::hard_turning::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double approx = M_PI * (20.0 * 20.0 - 16.0 * 16.0) * 10.0;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.08);

    // turn_external stamps its own feature first, then hard_turning re-stamps.
    EXPECT_EQ(out.workpiece->features().size(), 2u);
    EXPECT_EQ(out.signature.skill_id, std::string("hard_turning"));
}

// ─── 2. Validate rejects soft material (HRC < 50) ─────────────────────────
TEST(SkillHardTurning, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);

    skill::hard_turning::Input in;
    in.start_z_mm      = 5.0;
    in.end_z_mm        = 15.0;
    in.final_dia_mm    = 32.0;
    in.workpiece_hrc   = 30.0;       // too soft for hard turning
    in.roughing_passes = 1;

    auto r = skill::hard_turning::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-HARDTURN-HRC"));
    EXPECT_THROW(skill::hard_turning::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records strategy + CBN tooling ──────────────────────────
TEST(SkillHardTurning, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);

    skill::hard_turning::Input in;
    in.start_z_mm      = 4.0;
    in.end_z_mm        = 16.0;
    in.final_dia_mm    = 30.0;
    in.workpiece_hrc   = 60.0;
    in.feed_override_mm = 0.06;
    in.roughing_passes = 1;

    auto out = skill::hard_turning::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("hard_turning"));
    EXPECT_EQ(out.signature.pattern["strategy"].get<std::string>(),
              std::string("hard_turning"));
    EXPECT_NEAR(out.signature.pattern["workpiece_hrc"].get<double>(), 60.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["final_dia_mm"].get<double>(), 30.0, 1e-9);

    EXPECT_EQ(out.signature.tooling.tool_type, std::string("lathe_cbn_insert"));
    EXPECT_EQ(out.signature.tooling.tool_material, std::string("cbn"));
    EXPECT_LT(out.signature.tooling.cutting_speed_sfm, 250.0)
        << "hard turning runs at much lower SFM than carbide turn";
    EXPECT_NEAR(out.signature.tooling.feed_per_tooth_mm, 0.06, 1e-9);
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillHardTurning, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);

    skill::hard_turning::Input in;
    in.start_z_mm      = 5.0;
    in.end_z_mm        = 15.0;
    in.final_dia_mm    = 32.0;
    in.workpiece_hrc   = 58.0;
    in.roughing_passes = 1;

    auto out   = skill::hard_turning::apply(*stock, in);
    auto cands = skill::hard_turning::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("hard_turning"));
    EXPECT_GE(cands[0].confidence, 0.85);
    EXPECT_NEAR(cands[0].recovered_params["workpiece_hrc"].get<double>(),
                58.0, 1e-9);

    // On a metadata-free workpiece, hard_turning recognize returns empty
    // (turn_external would own the topology).
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::hard_turning::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. HRC > 65 (above CBN envelope) is rejected ─────────────────────────
TEST(SkillHardTurning, ValidateRejectsExcessiveHardness)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0);

    skill::hard_turning::Input in;
    in.start_z_mm      = 5.0;
    in.end_z_mm        = 15.0;
    in.final_dia_mm    = 32.0;
    in.workpiece_hrc   = 70.0;       // above CBN limit
    in.roughing_passes = 1;

    auto r = skill::hard_turning::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-HARDTURN-HRC"));
    EXPECT_THROW(skill::hard_turning::apply(*stock, in), skill::SkillError);
}
