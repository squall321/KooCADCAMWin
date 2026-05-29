// @lat: [[process/test-strategy#skill round-trip]]
//
// cable_pull — 5-case sweep: identity-geometry synthesis, DFM gates on
// length / force / cable_type, metadata signature replay, force-warning
// specific case.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cable_pull.hpp"

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
TEST(SkillCablePull, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::cable_pull::Input in;
    in.cable_type   = "fibre_smf";
    in.length_m     = 75.0;
    in.pull_force_n = 120.0;

    auto out = skill::cable_pull::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("cable_pull"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: empty cable_type, zero length, zero force rejected ───────────
TEST(SkillCablePull, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::cable_pull::Input bad;
    bad.cable_type   = "";             // invalid
    bad.length_m     = 50.0;
    bad.pull_force_n = 100.0;
    auto r = skill::cable_pull::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::cable_pull::apply(*stock, bad), skill::SkillError);

    bad.cable_type   = "cat6";
    bad.length_m     = 0.0;            // invalid
    EXPECT_FALSE(skill::cable_pull::validate(*stock, bad).passed);

    bad.length_m     = 50.0;
    bad.pull_force_n = -1.0;           // invalid
    EXPECT_FALSE(skill::cable_pull::validate(*stock, bad).passed);
}

// ── 3. Signature records kind + cable_type/length/force ──────────────────
TEST(SkillCablePull, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::cable_pull::Input in;
    in.cable_type   = "coax_rg6";
    in.length_m     = 125.0;
    in.pull_force_n = 200.0;

    auto out = skill::cable_pull::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("cable_pull"));
    EXPECT_EQ(out.signature.pattern["cable_type"].get<std::string>(),
              std::string("coax_rg6"));
    EXPECT_NEAR(out.signature.pattern["length_m"].get<double>(),     125.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["pull_force_n"].get<double>(), 200.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("cable_puller"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillCablePull, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::cable_pull::Input in;
    in.cable_type   = "fibre_smf";
    in.length_m     = 60.0;
    in.pull_force_n = 150.0;

    auto out = skill::cable_pull::apply(*stock, in);
    auto cands = skill::cable_pull::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["cable_type"].get<std::string>(),
              std::string("fibre_smf"));
    EXPECT_NEAR(cands[0].recovered_params["length_m"].get<double>(), 60.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::cable_pull::recognize(raw).empty());
}

// ── 5. SPECIFIC: pull_force_n > 600 N emits CABLE-FORCE warning ──────────
TEST(SkillCablePull, ExcessivePullForceEmitsWarning)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::cable_pull::Input in;
    in.cable_type   = "fibre_smf";
    in.length_m     = 50.0;
    in.pull_force_n = 800.0;          // exceeds 600 N

    auto r = skill::cable_pull::validate(*stock, in);
    EXPECT_TRUE(r.passed);              // warnings do not block
    EXPECT_TRUE(hasFinding(r, "DFM-CABLE-FORCE"));
    EXPECT_NO_THROW(skill::cable_pull::apply(*stock, in));
}
