// @lat: [[process/test-strategy#skill round-trip]]
//
// separator_install — 5-case sweep covering identity-geometry synthesis,
// type / capacity / pressure DFM gates, metadata signature replay, and
// the capacity → cycle-time scaling assertion.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/separator_install.hpp"

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

// ── 1. Apply: geometry is COMPLETELY unchanged ───────────────────────────
TEST(SkillSeparatorInstall, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::separator_install::Input in;
    in.separator_type       = "3-phase";
    in.capacity_bpd         = 10000.0;
    in.working_pressure_kpa = 5000.0;

    auto out = skill::separator_install::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("separator_install"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: zero capacity / pressure rejected; unknown type info ─────────
TEST(SkillSeparatorInstall, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::separator_install::Input bad;
    bad.separator_type       = "3-phase";
    bad.capacity_bpd         = 0.0;        // invalid
    bad.working_pressure_kpa = 5000.0;
    auto r = skill::separator_install::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::separator_install::apply(*stock, bad),
                 skill::SkillError);

    bad.capacity_bpd         = 10000.0;
    bad.working_pressure_kpa = 0.0;        // invalid
    EXPECT_FALSE(skill::separator_install::validate(*stock, bad).passed);

    // Unknown type → info only, still passes
    bad.separator_type       = "4-phase";
    bad.capacity_bpd         = 10000.0;
    bad.working_pressure_kpa = 5000.0;
    auto r2 = skill::separator_install::validate(*stock, bad);
    EXPECT_TRUE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-SI-TYPE"));
}

// ── 3. Signature records kind + type/capacity/pressure ───────────────────
TEST(SkillSeparatorInstall, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::separator_install::Input in;
    in.separator_type       = "2-phase";
    in.capacity_bpd         = 25000.0;
    in.working_pressure_kpa = 8000.0;

    auto out = skill::separator_install::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("separator_install"));
    EXPECT_EQ(out.signature.pattern["separator_type"].get<std::string>(),
              std::string("2-phase"));
    EXPECT_NEAR(out.signature.pattern["capacity_bpd"].get<double>(),
                25000.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["working_pressure_kpa"].get<double>(),
                8000.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("crane_rigging_spread"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillSeparatorInstall, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::separator_install::Input in;
    in.separator_type       = "3-phase";
    in.capacity_bpd         = 15000.0;
    in.working_pressure_kpa = 6500.0;

    auto out = skill::separator_install::apply(*stock, in);
    auto cands = skill::separator_install::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["separator_type"].get<std::string>(),
              std::string("3-phase"));
    EXPECT_NEAR(cands[0].recovered_params["capacity_bpd"].get<double>(),
                15000.0, 1e-9);

    // Same shape without metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::separator_install::recognize(raw).empty());
}

// ── 5. Cycle time scales with capacity (specific assertion) ──────────────
TEST(SkillSeparatorInstall, CycleTimeScalesWithCapacity)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::separator_install::Input small;
    small.separator_type       = "3-phase";
    small.capacity_bpd         = 5000.0;
    small.working_pressure_kpa = 4000.0;
    auto outSmall = skill::separator_install::apply(*stock, small);

    skill::separator_install::Input big = small;
    big.capacity_bpd = 50000.0;
    auto outBig = skill::separator_install::apply(*stock, big);

    EXPECT_GT(outBig.signature.tooling.est_cycle_time_s,
              outSmall.signature.tooling.est_cycle_time_s);

    // Exact small: 28800 + 720 * (5000/1000) = 28800 + 3600 = 32400 s
    EXPECT_NEAR(outSmall.signature.tooling.est_cycle_time_s, 32400.0, 1e-6);

    // HP-class warning fires at > 20000 kPa.
    skill::separator_install::Input hp = small;
    hp.working_pressure_kpa = 25000.0;
    auto r = skill::separator_install::validate(*stock, hp);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-SI-PRESS"));
}
