// @lat: [[process/test-strategy#skill round-trip]]
//
// motor_winding_ev skill — 5-case sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/motor_winding_ev.hpp"

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

skill::motor_winding_ev::Input goodInput()
{
    skill::motor_winding_ev::Input in;
    in.stator_id       = "STATOR-EV-001";
    in.slot_count      = 48;
    in.copper_fill_pct = 55.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillMotorWindingEv, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 80.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::motor_winding_ev::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("motor_winding_ev"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillMotorWindingEv, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 80.0);

    auto in = goodInput();
    in.copper_fill_pct = 0.0;             // must be > 0
    auto r = skill::motor_winding_ev::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-WIND-FILL"));
    EXPECT_THROW(skill::motor_winding_ev::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.copper_fill_pct = 90.0;           // > 75% physical ceiling
    auto r2 = skill::motor_winding_ev::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-WIND-FILL"));

    auto in3 = goodInput();
    in3.stator_id = "";                   // empty stator_id
    auto r3 = skill::motor_winding_ev::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillMotorWindingEv, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 80.0);

    auto in = goodInput();
    in.stator_id       = "STATOR-EV-042";
    in.slot_count      = 72;
    in.copper_fill_pct = 62.5;

    auto out = skill::motor_winding_ev::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("motor_winding_ev"));
    EXPECT_EQ(out.signature.pattern["stator_id"].get<std::string>(),
              std::string("STATOR-EV-042"));
    EXPECT_EQ(out.signature.pattern["slot_count"].get<int>(), 72);
    EXPECT_NEAR(out.signature.pattern["copper_fill_pct"].get<double>(),
                62.5, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillMotorWindingEv, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 80.0);
    auto out = skill::motor_winding_ev::apply(*stock, goodInput());

    auto cands = skill::motor_winding_ev::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["slot_count"].get<int>(), 48);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::motor_winding_ev::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. SPECIFIC: cycle time scales with slot count ───────────────────────
TEST(SkillMotorWindingEv, CycleTimeScalesWithSlotCount)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 80.0);

    auto inSmall = goodInput();
    inSmall.slot_count = 24;
    auto outSmall = skill::motor_winding_ev::apply(*stock, inSmall);

    auto inLarge = goodInput();
    inLarge.slot_count = 96;
    auto outLarge = skill::motor_winding_ev::apply(*stock, inLarge);

    EXPECT_LT(outSmall.signature.tooling.est_cycle_time_s,
              outLarge.signature.tooling.est_cycle_time_s);
    EXPECT_EQ(outSmall.signature.tooling.tool_type,
              std::string("hairpin_insertion_station"));
}
