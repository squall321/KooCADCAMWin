// @lat: [[process/test-strategy#skill round-trip]]
//
// contact_probe_array_scan — INSPECTION skill: multi-probe parallel
// contact sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/contact_probe_array_scan.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillContactProbeArrayScan, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::contact_probe_array_scan::Input in;
    in.probe_count   = 16;
    in.probe_dia_mm  = 2.0;
    in.sweep_pattern = "raster";

    auto out = skill::contact_probe_array_scan::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_TRUE(out.signature.pattern["is_inspection_only"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ── 2. ValidateRejectsBadInput: zero probes / zero dia ───────────────────
TEST(SkillContactProbeArrayScan, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::contact_probe_array_scan::Input noProbes;
    noProbes.probe_count   = 0;
    noProbes.probe_dia_mm  = 2.0;
    noProbes.sweep_pattern = "raster";
    auto r1 = skill::contact_probe_array_scan::validate(*stock, noProbes);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::contact_probe_array_scan::apply(*stock, noProbes),
                 skill::SkillError);

    skill::contact_probe_array_scan::Input badDia;
    badDia.probe_count   = 8;
    badDia.probe_dia_mm  = 0.0;
    badDia.sweep_pattern = "raster";
    auto r2 = skill::contact_probe_array_scan::validate(*stock, badDia);
    EXPECT_FALSE(r2.passed);
}

// ── 3. Signature records kind + tooling metadata ─────────────────────────
TEST(SkillContactProbeArrayScan, SignatureRecordsKindAndTooling)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::contact_probe_array_scan::Input in;
    in.probe_count   = 32;
    in.probe_dia_mm  = 1.5;
    in.sweep_pattern = "spiral";

    auto out = skill::contact_probe_array_scan::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id,
              std::string("contact_probe_array_scan"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("contact_probe_array_scan"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("probe_array"));
    EXPECT_EQ(out.signature.tooling.tool_material, std::string("ruby"));
    EXPECT_NEAR(out.signature.tooling.tool_dia_mm, 1.5, 1e-9);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
    EXPECT_GT(out.signature.tooling.est_cycle_time_s, 0.0);
}

// ── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillContactProbeArrayScan, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::contact_probe_array_scan::Input in;
    in.probe_count   = 8;
    in.probe_dia_mm  = 2.0;
    in.sweep_pattern = "serpentine";
    auto out = skill::contact_probe_array_scan::apply(*stock, in);

    auto cands = skill::contact_probe_array_scan::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].skill_id, std::string("contact_probe_array_scan"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(
        skill::contact_probe_array_scan::recognize(raw).empty());
}

// ── 5. probe_count + sweep_pattern + parallel flag round-trip ────────────
TEST(SkillContactProbeArrayScan, ProbeCountSweepPatternRoundtrip)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::contact_probe_array_scan::Input single;
    single.probe_count   = 1;
    single.probe_dia_mm  = 2.0;
    single.sweep_pattern = "raster";
    auto outS = skill::contact_probe_array_scan::apply(*stock, single);
    EXPECT_EQ(outS.signature.pattern["probe_count"].get<int>(), 1);
    EXPECT_FALSE(outS.signature.pattern["parallel_acquisition"].get<bool>())
        << "1 probe is sequential, not parallel";

    skill::contact_probe_array_scan::Input many;
    many.probe_count   = 64;
    many.probe_dia_mm  = 1.0;
    many.sweep_pattern = "spiral";
    auto outM = skill::contact_probe_array_scan::apply(*stock, many);
    EXPECT_EQ(outM.signature.pattern["probe_count"].get<int>(), 64);
    EXPECT_EQ(outM.signature.pattern["sweep_pattern"].get<std::string>(),
              std::string("spiral"));
    EXPECT_TRUE(outM.signature.pattern["parallel_acquisition"].get<bool>());
    EXPECT_NEAR(outM.signature.pattern["probe_dia_mm"].get<double>(),
                1.0, 1e-12);
}
