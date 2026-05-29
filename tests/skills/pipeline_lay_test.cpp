// @lat: [[process/test-strategy#skill round-trip]]
//
// pipeline_lay — 5-case sweep covering identity-geometry synthesis,
// pipe / wall / length DFM gates, metadata signature replay, and the
// terrain-class → duration cycle-time assertion.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pipeline_lay.hpp"

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
TEST(SkillPipelineLay, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::pipeline_lay::Input in;
    in.pipe_dia_mm   = 323.9;
    in.wall_thick_mm = 9.5;
    in.length_km     = 50.0;
    in.terrain_class = "rolling";

    auto out = skill::pipeline_lay::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("pipeline_lay"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: zero pipe/wall/length and wall ≥ radius rejected ─────────────
TEST(SkillPipelineLay, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::pipeline_lay::Input bad;
    bad.pipe_dia_mm   = 0.0;             // invalid
    bad.wall_thick_mm = 9.5;
    bad.length_km     = 50.0;
    bad.terrain_class = "rolling";
    auto r = skill::pipeline_lay::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::pipeline_lay::apply(*stock, bad), skill::SkillError);

    bad.pipe_dia_mm   = 323.9;
    bad.wall_thick_mm = 0.0;             // invalid
    EXPECT_FALSE(skill::pipeline_lay::validate(*stock, bad).passed);

    bad.wall_thick_mm = 200.0;           // wall ≥ radius (162 mm)
    EXPECT_FALSE(skill::pipeline_lay::validate(*stock, bad).passed);
    EXPECT_TRUE(hasFinding(skill::pipeline_lay::validate(*stock, bad),
                           "DFM-PL-WALL"));

    bad.wall_thick_mm = 9.5;
    bad.length_km     = 0.0;             // invalid
    EXPECT_FALSE(skill::pipeline_lay::validate(*stock, bad).passed);
}

// ── 3. Signature records kind + dia/wall/length/terrain ──────────────────
TEST(SkillPipelineLay, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::pipeline_lay::Input in;
    in.pipe_dia_mm   = 609.6;        // 24"
    in.wall_thick_mm = 12.7;
    in.length_km     = 120.0;
    in.terrain_class = "subsea";

    auto out = skill::pipeline_lay::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("pipeline_lay"));
    EXPECT_NEAR(out.signature.pattern["pipe_dia_mm"].get<double>(),   609.6, 1e-9);
    EXPECT_NEAR(out.signature.pattern["wall_thick_mm"].get<double>(),  12.7, 1e-9);
    EXPECT_NEAR(out.signature.pattern["length_km"].get<double>(),     120.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["terrain_class"].get<std::string>(),
              std::string("subsea"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("pipelayer_spread"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillPipelineLay, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::pipeline_lay::Input in;
    in.pipe_dia_mm   = 219.1;        // 8 5/8"
    in.wall_thick_mm = 8.2;
    in.length_km     = 25.5;
    in.terrain_class = "flat";

    auto out = skill::pipeline_lay::apply(*stock, in);
    auto cands = skill::pipeline_lay::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["terrain_class"].get<std::string>(),
              std::string("flat"));
    EXPECT_NEAR(cands[0].recovered_params["length_km"].get<double>(), 25.5, 1e-9);

    // Same shape without metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::pipeline_lay::recognize(raw).empty());
}

// ── 5. Cycle time differs by terrain (flat > rolling > mountain) ─────────
TEST(SkillPipelineLay, CycleTimeReflectsTerrainLayRate)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::pipeline_lay::Input flat;
    flat.pipe_dia_mm   = 323.9;
    flat.wall_thick_mm = 9.5;
    flat.length_km     = 30.0;
    flat.terrain_class = "flat";        // 3 km/day → 10 d
    auto outFlat = skill::pipeline_lay::apply(*stock, flat);

    skill::pipeline_lay::Input mtn = flat;
    mtn.terrain_class = "mountain";     // 0.8 km/day → 37.5 d
    auto outMtn = skill::pipeline_lay::apply(*stock, mtn);

    EXPECT_GT(outMtn.signature.tooling.est_cycle_time_s,
              outFlat.signature.tooling.est_cycle_time_s);
    // Flat exact: 30 / 3 days × 86400 = 864000 s
    EXPECT_NEAR(outFlat.signature.tooling.est_cycle_time_s, 864000.0, 1e-6);
    // duration_days field is consistent.
    EXPECT_NEAR(outFlat.signature.tooling.extra["duration_days"].get<double>(),
                10.0, 1e-9);
}
