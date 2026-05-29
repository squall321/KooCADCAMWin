// @lat: [[process/test-strategy#skill round-trip]]
//
// battery_recycle skill — 5-case sweep covering identity-geometry synthesis,
// chemistry+mass+recovery DFM gates, metadata signature replay, and the
// recovered_mass_kg = mass_kg * recovery_pct / 100 specific assertion.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/battery_recycle.hpp"

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

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

}  // namespace

// ─── 1. Apply: geometry is COMPLETELY unchanged ──────────────────────────
TEST(SkillBatteryRecycle, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::battery_recycle::Input in;
    in.battery_chemistry = "Li-ion";
    in.mass_kg           = 12.0;
    in.recovery_pct      = 85.0;

    auto out = skill::battery_recycle::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("battery_recycle"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. DFM: empty chemistry / zero mass / out-of-range recovery rejected
TEST(SkillBatteryRecycle, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::battery_recycle::Input bad;
    bad.battery_chemistry = "";              // invalid
    bad.mass_kg           = 10.0;
    bad.recovery_pct      = 80.0;
    auto r = skill::battery_recycle::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::battery_recycle::apply(*stock, bad), skill::SkillError);

    bad.battery_chemistry = "Li-ion";
    bad.mass_kg           = 0.0;             // invalid
    EXPECT_FALSE(skill::battery_recycle::validate(*stock, bad).passed);

    bad.mass_kg      = 10.0;
    bad.recovery_pct = 0.0;                  // invalid (must be > 0)
    EXPECT_FALSE(skill::battery_recycle::validate(*stock, bad).passed);

    bad.recovery_pct = 150.0;                // invalid (> 100)
    EXPECT_FALSE(skill::battery_recycle::validate(*stock, bad).passed);
}

// ─── 3. Signature records kind + chemistry + mass + recovery ─────────────
TEST(SkillBatteryRecycle, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::battery_recycle::Input in;
    in.battery_chemistry = "NiMH";
    in.mass_kg           = 7.5;
    in.recovery_pct      = 78.0;

    auto out = skill::battery_recycle::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("battery_recycle"));
    EXPECT_EQ(out.signature.pattern["battery_chemistry"].get<std::string>(),
              std::string("NiMH"));
    EXPECT_NEAR(out.signature.pattern["mass_kg"].get<double>(),      7.5,  1e-9);
    EXPECT_NEAR(out.signature.pattern["recovery_pct"].get<double>(), 78.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("battery_recycle_line"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillBatteryRecycle, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::battery_recycle::Input in;
    in.battery_chemistry = "Pb-acid";
    in.mass_kg           = 15.0;
    in.recovery_pct      = 92.0;

    auto out = skill::battery_recycle::apply(*stock, in);
    auto cands = skill::battery_recycle::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["battery_chemistry"].get<std::string>(),
              std::string("Pb-acid"));
    EXPECT_NEAR(cands[0].recovered_params["mass_kg"].get<double>(), 15.0, 1e-9);

    // Same shape without metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::battery_recycle::recognize(raw).empty())
        << "battery_recycle cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: recovered_mass_kg = mass_kg * recovery_pct / 100 ───────
TEST(SkillBatteryRecycle, RecoveredMassEqualsMassTimesRecoveryPct)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::battery_recycle::Input in;
    in.battery_chemistry = "Li-ion";
    in.mass_kg           = 20.0;
    in.recovery_pct      = 90.0;

    auto out = skill::battery_recycle::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["recovered_mass_kg"].get<double>(),
                18.0, 1e-9);
    EXPECT_NEAR(out.signature.tooling.extra["recovered_mass_kg"].get<double>(),
                18.0, 1e-9);

    // High-recovery edge: 95 % is the info threshold; > 95 % triggers info.
    skill::battery_recycle::Input edge;
    edge.battery_chemistry = "Li-ion";
    edge.mass_kg           = 5.0;
    edge.recovery_pct      = 98.0;
    auto r = skill::battery_recycle::validate(*stock, edge);
    EXPECT_TRUE(r.passed);  // info only
    EXPECT_TRUE(hasFinding(r, "DFM-BATT-RECOVERY"));
}
