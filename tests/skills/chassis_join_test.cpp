// @lat: [[process/test-strategy#skill round-trip]]
//
// chassis_join — 5-case sweep covering identity-geometry synthesis,
// bad-input rejection, signature, metadata replay, and the cycle-time
// formula (welds + sealer + bolts).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/chassis_join.hpp"

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

// ── 1. Apply: geometry COMPLETELY unchanged ──────────────────────────────
TEST(SkillChassisJoin, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::chassis_join::Input in;
    in.weld_count    = 80;
    in.sealer_meters = 4.5;
    in.bolt_count    = 16;

    auto out = skill::chassis_join::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("chassis_join"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: negative counts and empty-join rejected ──────────────────────
TEST(SkillChassisJoin, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::chassis_join::Input bad;
    bad.weld_count    = -1;          // invalid
    bad.sealer_meters = 1.0;
    bad.bolt_count    = 1;
    EXPECT_FALSE(skill::chassis_join::validate(*stock, bad).passed);
    EXPECT_THROW(skill::chassis_join::apply(*stock, bad), skill::SkillError);

    bad.weld_count    = 1;
    bad.sealer_meters = -0.5;        // invalid
    EXPECT_FALSE(skill::chassis_join::validate(*stock, bad).passed);

    bad.sealer_meters = 1.0;
    bad.bolt_count    = -1;          // invalid
    EXPECT_FALSE(skill::chassis_join::validate(*stock, bad).passed);

    bad.weld_count    = 0;           // empty join
    bad.bolt_count    = 0;
    bad.sealer_meters = 1.0;
    auto r = skill::chassis_join::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    bool sawEmpty = false;
    for (const auto& f : r.findings) {
        if (f.code == "DFM-CHASSIS-EMPTY" && f.severity == "error") {
            sawEmpty = true; break;
        }
    }
    EXPECT_TRUE(sawEmpty);
}

// ── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillChassisJoin, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::chassis_join::Input in;
    in.weld_count    = 96;
    in.sealer_meters = 5.25;
    in.bolt_count    = 22;

    auto out = skill::chassis_join::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("chassis_join"));
    EXPECT_EQ(out.signature.pattern["weld_count"].get<int>(), 96);
    EXPECT_NEAR(out.signature.pattern["sealer_meters"].get<double>(),
                5.25, 1e-9);
    EXPECT_EQ(out.signature.pattern["bolt_count"].get<int>(), 22);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("chassis_join_station"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillChassisJoin, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::chassis_join::Input in;
    in.weld_count    = 50;
    in.sealer_meters = 2.0;
    in.bolt_count    = 8;

    auto out = skill::chassis_join::apply(*stock, in);
    auto cands = skill::chassis_join::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["weld_count"].get<int>(), 50);
    EXPECT_EQ(cands[0].recovered_params["bolt_count"].get<int>(), 8);

    // Same shape without metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::chassis_join::recognize(raw).empty());
}

// ── 5. Cycle-time formula: 20 + 2*welds + 4*sealer + 6*bolts ─────────────
TEST(SkillChassisJoin, CycleTimeMatchesFormula)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::chassis_join::Input in;
    in.weld_count    = 60;
    in.sealer_meters = 3.0;
    in.bolt_count    = 12;

    auto out = skill::chassis_join::apply(*stock, in);
    // 20 + 2*60 + 4*3 + 6*12 = 20 + 120 + 12 + 72 = 224.
    EXPECT_NEAR(out.signature.tooling.est_cycle_time_s, 224.0, 1e-9);
}
