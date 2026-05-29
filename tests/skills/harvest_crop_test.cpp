// @lat: [[process/test-strategy#skill round-trip]]
//
// harvest_crop — 5-case sweep covering identity-geometry synthesis,
// DFM rejection, metadata signature replay, and total_yield_kg derivation.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/harvest_crop.hpp"

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

skill::harvest_crop::Input goodInput()
{
    skill::harvest_crop::Input in;
    in.crop_type      = "wheat";
    in.harvest_method = "combine";
    in.yield_kg_ha    = 3500.0;
    in.area_ha        = 10.0;
    return in;
}

}  // namespace

// ── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillHarvestCrop, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::harvest_crop::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("harvest_crop"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM rejects empty crop / zero yield / zero area ───────────────────
TEST(SkillHarvestCrop, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto bad = goodInput();
    bad.crop_type = "";          // invalid
    auto r = skill::harvest_crop::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::harvest_crop::apply(*stock, bad), skill::SkillError);

    bad = goodInput();
    bad.yield_kg_ha = 0.0;       // invalid
    EXPECT_FALSE(skill::harvest_crop::validate(*stock, bad).passed);

    bad = goodInput();
    bad.area_ha = -1.0;          // invalid
    EXPECT_FALSE(skill::harvest_crop::validate(*stock, bad).passed);
}

// ── 3. Signature records kind + crop/method/yield/area ───────────────────
TEST(SkillHarvestCrop, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.crop_type      = "rice";
    in.harvest_method = "manual";
    in.yield_kg_ha    = 5000.0;
    in.area_ha        = 2.5;

    auto out = skill::harvest_crop::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("harvest_crop"));
    EXPECT_EQ(out.signature.pattern["crop_type"].get<std::string>(),
              std::string("rice"));
    EXPECT_EQ(out.signature.pattern["harvest_method"].get<std::string>(),
              std::string("manual"));
    EXPECT_NEAR(out.signature.pattern["yield_kg_ha"].get<double>(), 5000.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["area_ha"].get<double>(),     2.5,    1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillHarvestCrop, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::harvest_crop::apply(*stock, goodInput());

    auto cands = skill::harvest_crop::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["crop_type"].get<std::string>(),
              std::string("wheat"));
    EXPECT_NEAR(cands[0].recovered_params["yield_kg_ha"].get<double>(),
                3500.0, 1e-9);

    // Strip metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::harvest_crop::recognize(raw).empty());

    // Sanity: extreme yield info, still passes.
    auto big = goodInput();
    big.yield_kg_ha = 20000.0;
    auto rb = skill::harvest_crop::validate(*stock, big);
    EXPECT_TRUE(rb.passed);
    EXPECT_TRUE(hasFinding(rb, "DFM-HARVEST-YIELD"));
}

// ── 5. SPECIFIC: total_yield_kg = yield_kg_ha × area_ha ──────────────────
TEST(SkillHarvestCrop, TotalYieldKgEqualsYieldPerHaTimesArea)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.yield_kg_ha = 4000.0;
    in.area_ha     = 7.5;
    auto out = skill::harvest_crop::apply(*stock, in);

    EXPECT_NEAR(out.signature.pattern["total_yield_kg"].get<double>(),
                30000.0, 1e-6);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("combine_harvester"));

    // Manual method maps to "manual_harvest" tool_type.
    auto manualIn = goodInput();
    manualIn.harvest_method = "manual";
    auto manualOut = skill::harvest_crop::apply(*stock, manualIn);
    EXPECT_EQ(manualOut.signature.tooling.tool_type,
              std::string("manual_harvest"));
}
