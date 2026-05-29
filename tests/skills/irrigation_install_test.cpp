// @lat: [[process/test-strategy#skill round-trip]]
//
// irrigation_install — 5-case sweep covering identity-geometry synthesis,
// system-type/area/flow DFM gates, metadata signature replay.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/irrigation_install.hpp"

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

skill::irrigation_install::Input goodInput()
{
    skill::irrigation_install::Input in;
    in.system_type = "drip";
    in.area_ha     = 5.0;
    in.flow_l_h    = 2.0;
    return in;
}

}  // namespace

// ── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillIrrigationInstall, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::irrigation_install::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("irrigation_install"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM rejects unknown type, zero area, zero flow ────────────────────
TEST(SkillIrrigationInstall, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto bad = goodInput();
    bad.system_type = "mystery";          // invalid
    auto r = skill::irrigation_install::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-IRRIG-TYPE"));
    EXPECT_THROW(skill::irrigation_install::apply(*stock, bad), skill::SkillError);

    bad = goodInput();
    bad.area_ha = 0.0;                    // invalid
    EXPECT_FALSE(skill::irrigation_install::validate(*stock, bad).passed);

    bad = goodInput();
    bad.flow_l_h = -1.0;                  // invalid
    EXPECT_FALSE(skill::irrigation_install::validate(*stock, bad).passed);
}

// ── 3. Signature records kind + type/area/flow ───────────────────────────
TEST(SkillIrrigationInstall, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.system_type = "sprinkler";
    in.area_ha     = 12.0;
    in.flow_l_h    = 1500.0;

    auto out = skill::irrigation_install::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("irrigation_install"));
    EXPECT_EQ(out.signature.pattern["system_type"].get<std::string>(),
              std::string("sprinkler"));
    EXPECT_NEAR(out.signature.pattern["area_ha"].get<double>(),  12.0,   1e-9);
    EXPECT_NEAR(out.signature.pattern["flow_l_h"].get<double>(), 1500.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("sprinkler_head_kit"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillIrrigationInstall, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::irrigation_install::apply(*stock, goodInput());

    auto cands = skill::irrigation_install::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["system_type"].get<std::string>(),
              std::string("drip"));
    EXPECT_NEAR(cands[0].recovered_params["area_ha"].get<double>(), 5.0, 1e-9);

    // Strip metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::irrigation_install::recognize(raw).empty());
}

// ── 5. SPECIFIC: drip ⇒ drip_emitter_kit, flood ⇒ flood_inlet_kit ────────
TEST(SkillIrrigationInstall, ToolTypeMatchesSystemType)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto dripIn = goodInput();
    dripIn.system_type = "drip";
    auto dripOut = skill::irrigation_install::apply(*stock, dripIn);
    EXPECT_EQ(dripOut.signature.tooling.tool_type,
              std::string("drip_emitter_kit"));

    auto floodIn = goodInput();
    floodIn.system_type = "flood";
    auto floodOut = skill::irrigation_install::apply(*stock, floodIn);
    EXPECT_EQ(floodOut.signature.tooling.tool_type,
              std::string("flood_inlet_kit"));

    // Flood emits an info finding but still passes.
    auto rFlood = skill::irrigation_install::validate(*stock, floodIn);
    EXPECT_TRUE(rFlood.passed);
    EXPECT_TRUE(hasFinding(rFlood, "DFM-IRRIG-FLOOD"));
}
