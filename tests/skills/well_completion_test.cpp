// @lat: [[process/test-strategy#skill round-trip]]
//
// well_completion — 5-case sweep covering identity-geometry synthesis,
// well-id / depth / casing / perforation DFM gates, metadata signature
// replay, and the perforation-rate cycle-time assertion.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/well_completion.hpp"

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
TEST(SkillWellCompletion, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::well_completion::Input in;
    in.well_id           = "K-15-A1";
    in.depth_m           = 2500.0;
    in.casing_dia_mm     = 177.8;
    in.perforation_count = 24;

    auto out = skill::well_completion::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("well_completion"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: empty well_id, zero depth/casing/perfs rejected ──────────────
TEST(SkillWellCompletion, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::well_completion::Input bad;
    bad.well_id           = "";              // invalid
    bad.depth_m           = 1000.0;
    bad.casing_dia_mm     = 177.8;
    bad.perforation_count = 12;
    auto r = skill::well_completion::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::well_completion::apply(*stock, bad), skill::SkillError);

    bad.well_id = "W-1";
    bad.depth_m = 0.0;                        // invalid
    EXPECT_FALSE(skill::well_completion::validate(*stock, bad).passed);

    bad.depth_m       = 1500.0;
    bad.casing_dia_mm = 0.0;                  // invalid
    EXPECT_FALSE(skill::well_completion::validate(*stock, bad).passed);

    bad.casing_dia_mm     = 177.8;
    bad.perforation_count = 0;                // invalid
    EXPECT_FALSE(skill::well_completion::validate(*stock, bad).passed);
}

// ── 3. Signature records kind + well_id/depth/casing/perfs ───────────────
TEST(SkillWellCompletion, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::well_completion::Input in;
    in.well_id           = "K-15-A2";
    in.depth_m           = 3120.0;
    in.casing_dia_mm     = 244.5;       // 9 5/8"
    in.perforation_count = 48;

    auto out = skill::well_completion::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("well_completion"));
    EXPECT_EQ(out.signature.pattern["well_id"].get<std::string>(),
              std::string("K-15-A2"));
    EXPECT_NEAR(out.signature.pattern["depth_m"].get<double>(),       3120.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["casing_dia_mm"].get<double>(),  244.5, 1e-9);
    EXPECT_EQ(out.signature.pattern["perforation_count"].get<int>(),   48);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("perforating_gun"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillWellCompletion, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::well_completion::Input in;
    in.well_id           = "K-15-B7";
    in.depth_m           = 2800.0;
    in.casing_dia_mm     = 177.8;
    in.perforation_count = 36;

    auto out = skill::well_completion::apply(*stock, in);
    auto cands = skill::well_completion::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["well_id"].get<std::string>(),
              std::string("K-15-B7"));
    EXPECT_EQ(cands[0].recovered_params["perforation_count"].get<int>(), 36);

    // Same shape without metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::well_completion::recognize(raw).empty());
}

// ── 5. cycle_time = 600 + 0.5 × perforation_count (specific assertion) ───
TEST(SkillWellCompletion, CycleTimeScalesWithPerforationCount)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::well_completion::Input in;
    in.well_id           = "K-15-C9";
    in.depth_m           = 2200.0;
    in.casing_dia_mm     = 177.8;
    in.perforation_count = 40;

    auto out = skill::well_completion::apply(*stock, in);
    // 600 + 0.5 * 40 = 620 s
    EXPECT_NEAR(out.signature.tooling.est_cycle_time_s, 620.0, 1e-9);

    // Heavy shot density → info finding, still passes.
    skill::well_completion::Input heavy;
    heavy.well_id           = "K-15-D1";
    heavy.depth_m           = 3500.0;
    heavy.casing_dia_mm     = 177.8;
    heavy.perforation_count = 240;
    auto r = skill::well_completion::validate(*stock, heavy);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-WC-PERF"));
}
