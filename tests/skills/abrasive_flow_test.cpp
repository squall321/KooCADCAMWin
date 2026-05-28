// @lat: [[process/test-strategy#skill round-trip]]
//
// abrasive_flow — AFM polishing (metadata-only) round-trip tests.
//
// Pattern follows tests/skills/anneal_test.cpp — no Boolean op verified;
// instead we verify that:
//   - apply() returns the SAME shape (geometry_changed == false),
//   - the signature records the Ra reduction,
//   - validate() rejects ra_after >= ra_before,
//   - recognize() replays the signature with confidence 1.0.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/abrasive_flow.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. Apply: metadata-only (no geometric change) ────────────────────────
TEST(SkillAbrasiveFlow, ApplyIsMetadataOnly)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::abrasive_flow::Input in;
    in.channel_label = "manifold_bore_3mm";
    in.ra_before_um  = 1.6;
    in.ra_after_um   = 0.4;
    in.cycles        = 50;

    auto out = skill::abrasive_flow::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume MUST be identical (within numerical noise).
    EXPECT_NEAR(volumeOf(stock->shape()),
                volumeOf(out.workpiece->shape()),
                1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ── 2. Validate rejects bad input (ra_after >= ra_before) ────────────────
TEST(SkillAbrasiveFlow, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::abrasive_flow::Input in;
    in.ra_before_um = 0.8;
    in.ra_after_um  = 1.2;       // worse than before
    in.cycles       = 50;

    auto r = skill::abrasive_flow::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-AFM-RA" && f.severity == "error") {
            found = true; break;
        }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::abrasive_flow::apply(*stock, in), skill::SkillError);
}

// ── 3. Signature records kind + tool_type ────────────────────────────────
TEST(SkillAbrasiveFlow, SignatureRecordsKindAndToolType)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::abrasive_flow::Input in;
    in.ra_before_um = 1.6;
    in.ra_after_um  = 0.4;
    in.cycles       = 60;

    auto out = skill::abrasive_flow::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("abrasive_flow"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("abrasive_flow"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("afm_putty_extruder"));
    EXPECT_TRUE(out.signature.tooling.extra["geometric_no_op"].get<bool>());
}

// ── 4. Recognize replays metadata at confidence 1.0 ──────────────────────
TEST(SkillAbrasiveFlow, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::abrasive_flow::Input in;
    in.channel_label = "fuel_injector";
    in.ra_before_um  = 0.8;
    in.ra_after_um   = 0.15;
    in.cycles        = 80;

    auto out = skill::abrasive_flow::apply(*stock, in);
    auto cands = skill::abrasive_flow::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params["channel_label"].get<std::string>(),
              std::string("fuel_injector"));
    EXPECT_NEAR(cands[0].recovered_params["ra_after_um"].get<double>(), 0.15, 1e-9);
}

// ── 5. Specific: roughness reduction recorded in the pattern ─────────────
TEST(SkillAbrasiveFlow, SurfaceRoughnessReductionRecorded)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::abrasive_flow::Input in;
    in.ra_before_um = 1.6;
    in.ra_after_um  = 0.2;
    in.cycles       = 100;

    auto out = skill::abrasive_flow::apply(*stock, in);

    EXPECT_NEAR(out.signature.pattern["ra_reduction_um"].get<double>(),
                1.4, 1e-9);
    EXPECT_NEAR(out.signature.pattern["ra_reduction_pct"].get<double>(),
                87.5, 1e-3);          // (1.4 / 1.6) × 100
    EXPECT_NEAR(out.signature.tooling.extra["ra_reduction_um"].get<double>(),
                1.4, 1e-9);
}
