// @lat: [[process/test-strategy#skill round-trip]]
//
// cnc_load_unload — 5-case sweep covering identity-geometry synthesis,
// payload + cycle-time DFM gates, metadata signature replay.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cnc_load_unload.hpp"

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

// ── 1. Apply: geometry is COMPLETELY unchanged ───────────────────────────
TEST(SkillCncLoadUnload, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::cnc_load_unload::Input in;
    in.load_method  = "robot";
    in.payload_kg   = 4.0;
    in.cycle_time_s = 10.0;

    auto out = skill::cnc_load_unload::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("cnc_load_unload"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: zero / negative payload and cycle-time rejected ──────────────
TEST(SkillCncLoadUnload, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::cnc_load_unload::Input bad;
    bad.payload_kg   = 0.0;            // invalid
    bad.cycle_time_s = 5.0;
    auto r = skill::cnc_load_unload::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::cnc_load_unload::apply(*stock, bad), skill::SkillError);

    bad.payload_kg   = -3.0;
    EXPECT_FALSE(skill::cnc_load_unload::validate(*stock, bad).passed);

    bad.payload_kg   = 5.0;
    bad.cycle_time_s = 0.0;            // invalid
    EXPECT_FALSE(skill::cnc_load_unload::validate(*stock, bad).passed);
}

// ── 3. Signature records kind + load_method/payload/cycle_time ───────────
TEST(SkillCncLoadUnload, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::cnc_load_unload::Input in;
    in.load_method  = "gantry";
    in.payload_kg   = 18.5;
    in.cycle_time_s = 25.0;

    auto out = skill::cnc_load_unload::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("cnc_load_unload"));
    EXPECT_EQ(out.signature.pattern["load_method"].get<std::string>(),
              std::string("gantry"));
    EXPECT_NEAR(out.signature.pattern["payload_kg"].get<double>(), 18.5, 1e-9);
    EXPECT_NEAR(out.signature.pattern["cycle_time_s"].get<double>(), 25.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("cell_loader"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillCncLoadUnload, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::cnc_load_unload::Input in;
    in.load_method  = "manual";
    in.payload_kg   = 2.0;
    in.cycle_time_s = 12.0;

    auto out = skill::cnc_load_unload::apply(*stock, in);
    auto cands = skill::cnc_load_unload::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["load_method"].get<std::string>(),
              std::string("manual"));
    EXPECT_NEAR(cands[0].recovered_params["cycle_time_s"].get<double>(), 12.0, 1e-9);

    // Same shape without metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::cnc_load_unload::recognize(raw).empty());
}

// ── 5. est_cycle_time_s mirrors input cycle_time_s ───────────────────────
TEST(SkillCncLoadUnload, EstCycleTimeMirrorsInputCycleTime)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::cnc_load_unload::Input in;
    in.load_method  = "robot";
    in.payload_kg   = 6.0;
    in.cycle_time_s = 14.5;

    auto out = skill::cnc_load_unload::apply(*stock, in);
    EXPECT_NEAR(out.signature.tooling.est_cycle_time_s, 14.5, 1e-9);

    // Excess cycle time → info finding, still passes.
    skill::cnc_load_unload::Input slow;
    slow.load_method  = "manual";
    slow.payload_kg   = 6.0;
    slow.cycle_time_s = 90.0;
    auto r = skill::cnc_load_unload::validate(*stock, slow);
    EXPECT_TRUE(r.passed);
    bool informed = false;
    for (const auto& f : r.findings) {
        if (f.code == "DFM-CNC-CYCLE" && f.severity == "info") {
            informed = true; break;
        }
    }
    EXPECT_TRUE(informed);
}
