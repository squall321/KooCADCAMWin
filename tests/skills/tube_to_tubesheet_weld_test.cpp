// @lat: [[process/test-strategy#skill round-trip]]
//
// tube_to_tubesheet_weld — 5-case sweep for the heat-exchanger tube-to-tube-
// sheet TIG roll-and-weld operation.  All cases assume the metadata-only
// pattern: geometry is left untouched; the signature captures tube_dia_mm,
// sheet_thick_mm, joint_count + computed fillet_leg_mm.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/tube_to_tubesheet_weld.hpp"

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

// ─── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillTubeToTubesheetWeld, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 25.0, "carbon_steel_1045");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::tube_to_tubesheet_weld::Input in;
    in.tube_dia_mm    = 19.05;
    in.sheet_thick_mm = 25.0;
    in.joint_count    = 120;

    auto out = skill::tube_to_tubesheet_weld::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillTubeToTubesheetWeld, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 25.0);

    skill::tube_to_tubesheet_weld::Input bad;
    bad.tube_dia_mm    = -1.0;     // invalid
    bad.sheet_thick_mm = 25.0;
    bad.joint_count    = 10;
    auto r1 = skill::tube_to_tubesheet_weld::validate(*stock, bad);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::tube_to_tubesheet_weld::apply(*stock, bad),
                 skill::SkillError);

    skill::tube_to_tubesheet_weld::Input bad2;
    bad2.tube_dia_mm    = 19.05;
    bad2.sheet_thick_mm = 25.0;
    bad2.joint_count    = 0;       // invalid
    auto r2 = skill::tube_to_tubesheet_weld::validate(*stock, bad2);
    EXPECT_FALSE(r2.passed);
}

// ─── 3. Signature records kind + params ───────────────────────────────────
TEST(SkillTubeToTubesheetWeld, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 25.0);

    skill::tube_to_tubesheet_weld::Input in;
    in.tube_dia_mm    = 25.4;
    in.sheet_thick_mm = 30.0;
    in.joint_count    = 240;

    auto out = skill::tube_to_tubesheet_weld::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("tube_to_tubesheet_weld"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("tube_to_tubesheet_weld"));
    EXPECT_NEAR(out.signature.pattern["tube_dia_mm"].get<double>(),    25.4, 1e-6);
    EXPECT_NEAR(out.signature.pattern["sheet_thick_mm"].get<double>(), 30.0, 1e-6);
    EXPECT_EQ  (out.signature.pattern["joint_count"].get<int>(),       240);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("tig_torch_orbital"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillTubeToTubesheetWeld, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 25.0);

    skill::tube_to_tubesheet_weld::Input in;
    in.tube_dia_mm    = 19.05;
    in.sheet_thick_mm = 25.0;
    in.joint_count    = 80;

    auto out = skill::tube_to_tubesheet_weld::apply(*stock, in);
    auto cands = skill::tube_to_tubesheet_weld::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["tube_dia_mm"].get<double>(),
                19.05, 1e-6);
    EXPECT_EQ  (cands[0].recovered_params["joint_count"].get<int>(), 80);

    // Strip features — pure geometry cannot recover this skill.
    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::tube_to_tubesheet_weld::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty());
}

// ─── 5. Specific: cycle-time scales with joint_count ──────────────────────
// Orbital TIG ≈ 60 s / joint, so 100 joints ⇒ ≥ 6000 s and far exceeds the
// per-joint cycle of a single weld — verifies fleet-level planning data.
TEST(SkillTubeToTubesheetWeld, CycleTimeScalesWithJointCount)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 25.0);

    skill::tube_to_tubesheet_weld::Input one;
    one.tube_dia_mm    = 19.05;
    one.sheet_thick_mm = 25.0;
    one.joint_count    = 1;
    const double tOne =
        skill::tube_to_tubesheet_weld::apply(*stock, one).signature
        .tooling.est_cycle_time_s;

    skill::tube_to_tubesheet_weld::Input many = one;
    many.joint_count = 100;
    const double tMany =
        skill::tube_to_tubesheet_weld::apply(*stock, many).signature
        .tooling.est_cycle_time_s;

    EXPECT_GT(tMany, tOne * 50.0)
        << "100 joints should cost much more than 1 joint";
    EXPECT_NEAR(tMany / tOne, 100.0, 1e-6)
        << "cycle time must scale linearly with joint_count";
}
