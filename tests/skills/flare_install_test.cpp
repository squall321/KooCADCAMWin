// @lat: [[process/test-strategy#skill round-trip]]
//
// flare_install — 5-case sweep covering identity-geometry synthesis,
// stack-height / flow / ignition DFM gates, metadata signature replay,
// and the stack-height → erection cycle-time linear scaling.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/flare_install.hpp"

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
TEST(SkillFlareInstall, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::flare_install::Input in;
    in.stack_height_m  = 40.0;
    in.max_flow_nm3_h  = 50000.0;
    in.ignition_type   = "pilot";

    auto out = skill::flare_install::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("flare_install"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: zero height / flow rejected; empty ignition warns ────────────
TEST(SkillFlareInstall, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::flare_install::Input bad;
    bad.stack_height_m = 0.0;        // invalid
    bad.max_flow_nm3_h = 50000.0;
    bad.ignition_type  = "pilot";
    auto r = skill::flare_install::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::flare_install::apply(*stock, bad),
                 skill::SkillError);

    bad.stack_height_m = 40.0;
    bad.max_flow_nm3_h = 0.0;        // invalid
    EXPECT_FALSE(skill::flare_install::validate(*stock, bad).passed);

    // Empty ignition → warning, but still passes
    bad.max_flow_nm3_h = 50000.0;
    bad.ignition_type  = "";
    auto r2 = skill::flare_install::validate(*stock, bad);
    EXPECT_TRUE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ── 3. Signature records kind + height/flow/ignition ─────────────────────
TEST(SkillFlareInstall, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::flare_install::Input in;
    in.stack_height_m = 60.0;
    in.max_flow_nm3_h = 120000.0;
    in.ignition_type  = "flame_front_generator";

    auto out = skill::flare_install::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("flare_install"));
    EXPECT_NEAR(out.signature.pattern["stack_height_m"].get<double>(),
                60.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["max_flow_nm3_h"].get<double>(),
                120000.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["ignition_type"].get<std::string>(),
              std::string("flame_front_generator"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("crawler_crane_erection"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillFlareInstall, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::flare_install::Input in;
    in.stack_height_m = 75.0;
    in.max_flow_nm3_h = 80000.0;
    in.ignition_type  = "ballistic";

    auto out = skill::flare_install::apply(*stock, in);
    auto cands = skill::flare_install::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["ignition_type"].get<std::string>(),
              std::string("ballistic"));
    EXPECT_NEAR(cands[0].recovered_params["stack_height_m"].get<double>(),
                75.0, 1e-9);

    // Same shape without metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::flare_install::recognize(raw).empty());
}

// ── 5. Cycle time = 28800 + 3600 × stack_height_m (specific assertion) ───
TEST(SkillFlareInstall, CycleTimeLinearInStackHeight)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::flare_install::Input in;
    in.stack_height_m = 40.0;
    in.max_flow_nm3_h = 50000.0;
    in.ignition_type  = "pilot";

    auto out = skill::flare_install::apply(*stock, in);
    // 28800 + 3600 * 40 = 172800 s
    EXPECT_NEAR(out.signature.tooling.est_cycle_time_s, 172800.0, 1e-6);

    // Tall-stack triggers info finding, still passes.
    skill::flare_install::Input tall = in;
    tall.stack_height_m = 150.0;
    auto r = skill::flare_install::validate(*stock, tall);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-FI-HEIGHT"));

    // tool_length_mm equals stack_height in mm.
    auto outTall = skill::flare_install::apply(*stock, tall);
    EXPECT_NEAR(outTall.signature.tooling.tool_length_mm, 150000.0, 1e-6);
}
