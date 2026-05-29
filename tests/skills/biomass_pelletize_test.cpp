// @lat: [[process/test-strategy#skill round-trip]]
//
// biomass_pelletize — 5-case sweep: identity-geometry synthesis, DFM gates
// on biomass / pellet_dia / throughput, metadata replay, tool_dia matches
// pellet_dia.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/biomass_pelletize.hpp"

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

skill::biomass_pelletize::Input goodInput()
{
    skill::biomass_pelletize::Input in;
    in.biomass_type    = "wood_sawdust";
    in.pellet_dia_mm   = 6.0;
    in.throughput_kg_h = 1500.0;
    return in;
}

}  // namespace

// ── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillBiomassPelletize, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::biomass_pelletize::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("biomass_pelletize"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillBiomassPelletize, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto bad = goodInput();
    bad.pellet_dia_mm = 0.0;
    EXPECT_FALSE(skill::biomass_pelletize::validate(*stock, bad).passed);
    EXPECT_THROW(skill::biomass_pelletize::apply(*stock, bad),
                 skill::SkillError);

    bad = goodInput();
    bad.throughput_kg_h = 0.0;
    EXPECT_FALSE(skill::biomass_pelletize::validate(*stock, bad).passed);
    EXPECT_THROW(skill::biomass_pelletize::apply(*stock, bad),
                 skill::SkillError);

    bad = goodInput();
    bad.throughput_kg_h = -100.0;
    EXPECT_FALSE(skill::biomass_pelletize::validate(*stock, bad).passed);
}

// ── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillBiomassPelletize, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::biomass_pelletize::Input in;
    in.biomass_type    = "rice_husk";
    in.pellet_dia_mm   = 8.0;
    in.throughput_kg_h = 2500.0;

    auto out = skill::biomass_pelletize::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("biomass_pelletize"));
    EXPECT_EQ(out.signature.pattern["biomass_type"].get<std::string>(),
              std::string("rice_husk"));
    EXPECT_NEAR(out.signature.pattern["pellet_dia_mm"].get<double>(),
                8.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["throughput_kg_h"].get<double>(),
                2500.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("ring_die_pellet_mill"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillBiomassPelletize, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::biomass_pelletize::apply(*stock, goodInput());

    auto cands = skill::biomass_pelletize::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["biomass_type"].get<std::string>(),
              std::string("wood_sawdust"));
    EXPECT_NEAR(cands[0].recovered_params["pellet_dia_mm"].get<double>(),
                6.0, 1e-9);

    // Strip metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::biomass_pelletize::recognize(raw).empty());
}

// ── 5. SPECIFIC: tooling.tool_dia_mm == pellet_dia_mm ────────────────────
TEST(SkillBiomassPelletize, ToolDiaMatchesPelletDia)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::biomass_pelletize::Input small = goodInput();
    small.pellet_dia_mm = 6.0;
    auto outSmall = skill::biomass_pelletize::apply(*stock, small);
    EXPECT_NEAR(outSmall.signature.tooling.tool_dia_mm, 6.0, 1e-9);

    skill::biomass_pelletize::Input big = goodInput();
    big.pellet_dia_mm = 12.0;
    auto outBig = skill::biomass_pelletize::apply(*stock, big);
    EXPECT_NEAR(outBig.signature.tooling.tool_dia_mm, 12.0, 1e-9);

    // Out-of-band dia → INFO only.
    skill::biomass_pelletize::Input oddDia = goodInput();
    oddDia.pellet_dia_mm = 2.0;     // below ENplus min
    auto r = skill::biomass_pelletize::validate(*stock, oddDia);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-PELLET-DIA"));
    EXPECT_NO_THROW(skill::biomass_pelletize::apply(*stock, oddDia));
}
