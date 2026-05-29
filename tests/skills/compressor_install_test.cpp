// @lat: [[process/test-strategy#skill round-trip]]
//
// compressor_install — 5-case sweep covering identity-geometry synthesis,
// type / power / suction-pressure DFM gates, metadata signature replay,
// and the centrifugal-vs-reciprocating cycle-time differential.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/compressor_install.hpp"

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
TEST(SkillCompressorInstall, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::compressor_install::Input in;
    in.compressor_type      = "centrifugal";
    in.power_kw             = 2000.0;
    in.suction_pressure_kpa = 3000.0;

    auto out = skill::compressor_install::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("compressor_install"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: zero power / pressure rejected; unknown type info ────────────
TEST(SkillCompressorInstall, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::compressor_install::Input bad;
    bad.compressor_type      = "centrifugal";
    bad.power_kw             = 0.0;            // invalid
    bad.suction_pressure_kpa = 3000.0;
    auto r = skill::compressor_install::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::compressor_install::apply(*stock, bad),
                 skill::SkillError);

    bad.power_kw             = 2000.0;
    bad.suction_pressure_kpa = 0.0;            // invalid
    EXPECT_FALSE(skill::compressor_install::validate(*stock, bad).passed);

    // Unknown type → info, still passes
    bad.compressor_type      = "screw";
    bad.power_kw             = 2000.0;
    bad.suction_pressure_kpa = 3000.0;
    auto r2 = skill::compressor_install::validate(*stock, bad);
    EXPECT_TRUE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-CI-TYPE"));
}

// ── 3. Signature records kind + type/power/pressure ──────────────────────
TEST(SkillCompressorInstall, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::compressor_install::Input in;
    in.compressor_type      = "reciprocating";
    in.power_kw             = 1500.0;
    in.suction_pressure_kpa = 4000.0;

    auto out = skill::compressor_install::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("compressor_install"));
    EXPECT_EQ(out.signature.pattern["compressor_type"].get<std::string>(),
              std::string("reciprocating"));
    EXPECT_NEAR(out.signature.pattern["power_kw"].get<double>(), 1500.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["suction_pressure_kpa"].get<double>(),
                4000.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("crane_alignment_spread"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillCompressorInstall, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::compressor_install::Input in;
    in.compressor_type      = "centrifugal";
    in.power_kw             = 3500.0;
    in.suction_pressure_kpa = 5500.0;

    auto out = skill::compressor_install::apply(*stock, in);
    auto cands = skill::compressor_install::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["compressor_type"].get<std::string>(),
              std::string("centrifugal"));
    EXPECT_NEAR(cands[0].recovered_params["power_kw"].get<double>(), 3500.0, 1e-9);

    // Same shape without metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::compressor_install::recognize(raw).empty());
}

// ── 5. Reciprocating cycle time > centrifugal at equal power ─────────────
TEST(SkillCompressorInstall, ReciprocatingCycleTimeExceedsCentrifugal)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::compressor_install::Input centri;
    centri.compressor_type      = "centrifugal";
    centri.power_kw             = 2000.0;
    centri.suction_pressure_kpa = 3000.0;
    auto outC = skill::compressor_install::apply(*stock, centri);

    skill::compressor_install::Input recip = centri;
    recip.compressor_type = "reciprocating";
    auto outR = skill::compressor_install::apply(*stock, recip);

    EXPECT_GT(outR.signature.tooling.est_cycle_time_s,
              outC.signature.tooling.est_cycle_time_s);

    // Centrifugal exact: 43200 + 7200 * (2000/1000) = 43200 + 14400 = 57600 s
    EXPECT_NEAR(outC.signature.tooling.est_cycle_time_s, 57600.0, 1e-6);
    // Reciprocating exact: 86400 + 14400 = 100800 s
    EXPECT_NEAR(outR.signature.tooling.est_cycle_time_s, 100800.0, 1e-6);
}
