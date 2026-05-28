// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/probe_point.hpp"

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

// ── 1. Apply: no geometric change, signature recorded ────────────────────
TEST(SkillProbePoint, ApplyIsGeometricNoOp)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::probe_point::Input in;
    in.probe_point_3d = gp_Pnt(25.0, 25.0, 10.0);   // top centre
    in.tolerance_mm   = 0.05;

    auto out = skill::probe_point::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    // Volume unchanged.
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);

    EXPECT_EQ(out.signature.skill_id, std::string("probe_point"));
    EXPECT_TRUE(out.signature.pattern["is_inspection_only"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ── 2. DFM-INPUT: zero/negative tolerance rejected ───────────────────────
TEST(SkillProbePoint, ValidateRejectsZeroTolerance)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::probe_point::Input in;
    in.probe_point_3d = gp_Pnt(25.0, 25.0, 10.0);
    in.tolerance_mm   = 0.0;

    auto r = skill::probe_point::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::probe_point::apply(*stock, in), skill::SkillError);
}

// ── 3. DFM-PROBE-LOC: point outside bbox flagged as warning ──────────────
TEST(SkillProbePoint, ValidateWarnsOnOutOfBboxPoint)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::probe_point::Input in;
    in.probe_point_3d = gp_Pnt(1000.0, 1000.0, 1000.0);   // far outside
    in.tolerance_mm   = 0.05;

    auto r = skill::probe_point::validate(*stock, in);
    // Out-of-bbox is a warning (not error) — validate still passes.
    EXPECT_TRUE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PROBE-LOC" && f.severity == "warning") {
            found = true;
            break;
        }
    EXPECT_TRUE(found);
}

// ── 4. Signature carries probe point + tolerance verbatim ────────────────
TEST(SkillProbePoint, SignatureRecordsProbeData)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::probe_point::Input in;
    in.probe_point_3d = gp_Pnt(12.5, 37.5, 10.0);
    in.tolerance_mm   = 0.02;

    auto out = skill::probe_point::apply(*stock, in);
    const auto& p = out.signature.params["probe_point_3d"];
    EXPECT_NEAR(p[0].get<double>(), 12.5, 1e-9);
    EXPECT_NEAR(p[1].get<double>(), 37.5, 1e-9);
    EXPECT_NEAR(p[2].get<double>(), 10.0, 1e-9);
    EXPECT_NEAR(out.signature.params["tolerance_mm"].get<double>(), 0.02, 1e-9);
}

// ── 5. Recognize is always empty (metadata-only skill) ───────────────────
TEST(SkillProbePoint, RecognizeAlwaysEmpty)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto cands = skill::probe_point::recognize(*stock);
    EXPECT_TRUE(cands.empty());

    // And after applying a probe, recognize() still finds nothing
    // (the probe leaves no geometric trace).
    skill::probe_point::Input in;
    in.probe_point_3d = gp_Pnt(25.0, 25.0, 10.0);
    in.tolerance_mm   = 0.05;
    auto out = skill::probe_point::apply(*stock, in);
    EXPECT_TRUE(skill::probe_point::recognize(*out.workpiece).empty());
}

// ── 6. Multiple probe_points chain via feature history ───────────────────
TEST(SkillProbePoint, MultipleProbesAccumulateInFeatureHistory)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::probe_point::Input in1, in2;
    in1.probe_point_3d = gp_Pnt(10.0, 10.0, 10.0);
    in1.tolerance_mm   = 0.05;
    in2.probe_point_3d = gp_Pnt(40.0, 40.0, 10.0);
    in2.tolerance_mm   = 0.05;

    auto out1 = skill::probe_point::apply(*stock, in1);
    auto out2 = skill::probe_point::apply(*out1.workpiece, in2);
    EXPECT_EQ(out2.workpiece->features().size(), 2u);
    EXPECT_EQ(out2.workpiece->features()[0].skill_id, std::string("probe_point"));
    EXPECT_EQ(out2.workpiece->features()[1].skill_id, std::string("probe_point"));
}
