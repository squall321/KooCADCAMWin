// @lat: [[process/test-strategy#skill round-trip]]
//
// fire_protection_install skill — 5-case sweep covering identity-geometry
// synthesis, DFM rejection of zero coverage, and metadata-only recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/fire_protection_install.hpp"

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

skill::fire_protection_install::Input goodInput()
{
    skill::fire_protection_install::Input in;
    in.sprinkler_count    = 12;
    in.supply_pipe_dia_mm = 65.0;
    in.coverage_m2        = 200.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillFireProtectionInstall, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::fire_protection_install::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("fire_protection_install"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (zero sprinklers / oversize pipe) ──────
TEST(SkillFireProtectionInstall, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.sprinkler_count = 0;       // must be >= 1
    auto r = skill::fire_protection_install::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::fire_protection_install::apply(*stock, in),
                 skill::SkillError);

    auto in2 = goodInput();
    in2.supply_pipe_dia_mm = 600.0;  // > 300 mm limit
    auto r2 = skill::fire_protection_install::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-FIRE-DIA"));
}

// ─── 3. Signature records kind + key params + coverage-per-head ───────────
TEST(SkillFireProtectionInstall, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.sprinkler_count    = 20;
    in.supply_pipe_dia_mm = 100.0;
    in.coverage_m2        = 300.0;       // 15 m^2 / head

    auto out = skill::fire_protection_install::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("fire_protection_install"));
    EXPECT_EQ(out.signature.pattern["sprinkler_count"].get<int>(), 20);
    EXPECT_NEAR(out.signature.pattern["supply_pipe_dia_mm"].get<double>(),
                100.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["coverage_m2"].get<double>(),
                300.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["coverage_per_head_m2"].get<double>(),
                15.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillFireProtectionInstall, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::fire_protection_install::apply(*stock, goodInput());

    auto cands = skill::fire_protection_install::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["sprinkler_count"].get<int>(), 12);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::fire_protection_install::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "fire_protection_install cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: over-NFPA-13 coverage emits info but hazard class flips ─
TEST(SkillFireProtectionInstall, OverCoverageEmitsInfoAndFlipsHazardClass)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.sprinkler_count = 5;
    in.coverage_m2     = 200.0;   // 40 m^2 / head > 20 m^2 light-hazard limit

    auto r = skill::fire_protection_install::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "info-level finding must not block";
    EXPECT_TRUE(hasFinding(r, "DFM-FIRE-COVERAGE"));

    auto out = skill::fire_protection_install::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["hazard_class"].get<std::string>(),
              std::string("over_limit"));
}
