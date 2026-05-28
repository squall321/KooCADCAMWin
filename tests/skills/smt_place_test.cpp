// @lat: [[process/test-strategy#skill round-trip]]
//
// smt_place — surface-mount pick-and-place placement skill, 5-case sweep.
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndKeyParams
//   4. RecognizeMetadataReplay
//   5. CycleTimeMatchesRateAndCount   (specific assertion)

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/smt_place.hpp"

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

// ─── 1. Apply: geometry is COMPLETELY unchanged ──────────────────────────
TEST(SkillSmtPlace, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 1.6, "FR4");  // PCB-ish
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::smt_place::Input in;
    in.component_count    = 250;
    in.placement_rate_cph = 30000.0;
    in.accuracy_um        = 40.0;

    auto out = skill::smt_place::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("smt_place"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. DFM: invalid inputs are rejected ─────────────────────────────────
TEST(SkillSmtPlace, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 1.6, "FR4");

    skill::smt_place::Input zeroCount;
    zeroCount.component_count    = 0;            // invalid
    zeroCount.placement_rate_cph = 30000.0;
    zeroCount.accuracy_um        = 40.0;
    auto r1 = skill::smt_place::validate(*stock, zeroCount);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::smt_place::apply(*stock, zeroCount), skill::SkillError);

    skill::smt_place::Input coarse;
    coarse.component_count    = 100;
    coarse.placement_rate_cph = 30000.0;
    coarse.accuracy_um        = 250.0;           // > 100 μm → DFM-SMT-ACCURACY
    auto r2 = skill::smt_place::validate(*stock, coarse);
    EXPECT_FALSE(r2.passed);

    bool foundAcc = false;
    for (const auto& f : r2.findings)
        if (f.code == "DFM-SMT-ACCURACY") { foundAcc = true; break; }
    EXPECT_TRUE(foundAcc);
}

// ─── 3. Signature carries kind + key parameters ──────────────────────────
TEST(SkillSmtPlace, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 1.6, "FR4");

    skill::smt_place::Input in;
    in.component_count    = 1024;
    in.placement_rate_cph = 50000.0;
    in.accuracy_um        = 25.0;

    auto out = skill::smt_place::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("smt_place"));
    EXPECT_EQ(out.signature.pattern["component_count"].get<int>(), 1024);
    EXPECT_NEAR(out.signature.pattern["placement_rate_cph"].get<double>(),
                50000.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["accuracy_um"].get<double>(),
                25.0, 1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("pick_and_place"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillSmtPlace, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 1.6, "FR4");

    skill::smt_place::Input in;
    in.component_count    = 320;
    in.placement_rate_cph = 25000.0;
    in.accuracy_um        = 50.0;

    auto out = skill::smt_place::apply(*stock, in);

    auto cands = skill::smt_place::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["component_count"].get<int>(), 320);

    // Strip features → empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::smt_place::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty())
        << "smt_place is not geometrically recognizable";
}

// ─── 5. Cycle time matches count / rate (specific assertion) ─────────────
TEST(SkillSmtPlace, CycleTimeMatchesRateAndCount)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 1.6, "FR4");

    skill::smt_place::Input in;
    in.component_count    = 18000;            // 18000 / 36000 cph = 0.5 h = 1800 s
    in.placement_rate_cph = 36000.0;
    in.accuracy_um        = 30.0;

    auto out = skill::smt_place::apply(*stock, in);

    EXPECT_NEAR(out.signature.tooling.est_cycle_time_s, 1800.0, 1e-3);
}
