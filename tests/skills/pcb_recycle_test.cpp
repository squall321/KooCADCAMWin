// @lat: [[process/test-strategy#skill round-trip]]
//
// pcb_recycle skill — 5-case sweep covering identity-geometry synthesis,
// mass+yield DFM gates, metadata signature replay, and the
// recovered_pm_g = pcb_mass_kg * pm_yield_g_t / 1000 specific assertion.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pcb_recycle.hpp"

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
TEST(SkillPcbRecycle, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::pcb_recycle::Input in;
    in.pcb_mass_kg              = 4.0;
    in.copper_yield_pct         = 22.0;
    in.precious_metal_yield_g_t = 300.0;

    auto out = skill::pcb_recycle::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("pcb_recycle"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. DFM: zero mass / bad Cu% / negative PM yield rejected ────────────
TEST(SkillPcbRecycle, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::pcb_recycle::Input bad;
    bad.pcb_mass_kg              = 0.0;          // invalid
    bad.copper_yield_pct         = 20.0;
    bad.precious_metal_yield_g_t = 200.0;
    auto r = skill::pcb_recycle::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::pcb_recycle::apply(*stock, bad), skill::SkillError);

    bad.pcb_mass_kg      = 5.0;
    bad.copper_yield_pct = 0.0;                  // invalid (must be > 0)
    EXPECT_FALSE(skill::pcb_recycle::validate(*stock, bad).passed);

    bad.copper_yield_pct = 150.0;                // invalid (> 100)
    EXPECT_FALSE(skill::pcb_recycle::validate(*stock, bad).passed);

    bad.copper_yield_pct         = 20.0;
    bad.precious_metal_yield_g_t = -1.0;         // invalid (< 0)
    EXPECT_FALSE(skill::pcb_recycle::validate(*stock, bad).passed);
}

// ─── 3. Signature records kind + mass + yields ───────────────────────────
TEST(SkillPcbRecycle, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::pcb_recycle::Input in;
    in.pcb_mass_kg              = 8.0;
    in.copper_yield_pct         = 25.0;
    in.precious_metal_yield_g_t = 450.0;

    auto out = skill::pcb_recycle::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("pcb_recycle"));
    EXPECT_NEAR(out.signature.pattern["pcb_mass_kg"].get<double>(),      8.0,   1e-9);
    EXPECT_NEAR(out.signature.pattern["copper_yield_pct"].get<double>(), 25.0,  1e-9);
    EXPECT_NEAR(out.signature.pattern["precious_metal_yield_g_t"].get<double>(),
                450.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("pcb_recycle_line"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillPcbRecycle, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::pcb_recycle::Input in;
    in.pcb_mass_kg              = 6.0;
    in.copper_yield_pct         = 18.0;
    in.precious_metal_yield_g_t = 200.0;

    auto out = skill::pcb_recycle::apply(*stock, in);
    auto cands = skill::pcb_recycle::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["pcb_mass_kg"].get<double>(), 6.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["copper_yield_pct"].get<double>(),
                18.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::pcb_recycle::recognize(raw).empty())
        << "pcb_recycle cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: recovered_pm_g = mass_kg * pm_yield_g_t / 1000 ─────────
TEST(SkillPcbRecycle, RecoveredPmGramsMatchesFormula)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::pcb_recycle::Input in;
    in.pcb_mass_kg              = 10.0;
    in.copper_yield_pct         = 20.0;
    in.precious_metal_yield_g_t = 500.0;

    auto out = skill::pcb_recycle::apply(*stock, in);
    // 10 kg × 500 g/tonne ÷ 1000 kg/tonne = 5 g recovered precious metal
    EXPECT_NEAR(out.signature.pattern["recovered_pm_g"].get<double>(),
                5.0, 1e-9);
    // 10 kg × 20 % = 2 kg recovered copper
    EXPECT_NEAR(out.signature.pattern["recovered_copper_kg"].get<double>(),
                2.0, 1e-9);

    // High-grade feedstock crosses info threshold but still passes.
    skill::pcb_recycle::Input rich;
    rich.pcb_mass_kg              = 3.0;
    rich.copper_yield_pct         = 22.0;
    rich.precious_metal_yield_g_t = 1500.0;  // > 1000
    auto r = skill::pcb_recycle::validate(*stock, rich);
    EXPECT_TRUE(r.passed);  // info only
    EXPECT_TRUE(hasFinding(r, "DFM-PCB-PM"));
}
