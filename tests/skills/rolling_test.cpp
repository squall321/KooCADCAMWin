// @lat: [[process/test-strategy#skill round-trip]]
//
// rolling skill — reduce sheet thickness by passing through rollers.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/rolling.hpp"

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

// ── 1. Apply: 2.0 mm → 1.5 mm reduction (25%) — REAL geometric rebuild ─────
TEST(SkillRolling, ApplyReducesThickness)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 2.0);
    EXPECT_NEAR(skill::rolling::currentThickness(*stock), 2.0, 1e-6);
    const double v0 = volumeOf(stock->shape());

    skill::rolling::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.target_thickness_mm = 1.5;
    in.reduction_pct_per_pass = 30.0;

    auto out = skill::rolling::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // bbox thickness reduced from 2.0 → 1.5 (geometric change verified).
    EXPECT_NEAR(skill::rolling::currentThickness(*out.workpiece), 1.5, 1e-6);

    // Volume reduced by ΔV = dx · dy · (t0 − t1) = 100 · 80 · 0.5 = 4000 mm³.
    // Output bbox volume = 100·80·1.5 = 12000 mm³ (vs. stock 16000 mm³).
    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_NEAR(v1, 12000.0, 1e-3);
    EXPECT_NEAR(v0 - v1, 4000.0, 1e-3);

    // Tooling reports the geometric bbox delta.
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 4000.0, 1e-3);

    EXPECT_EQ(out.signature.skill_id, std::string("rolling"));
    EXPECT_NEAR(out.signature.pattern["original_thickness_mm"].get<double>(), 2.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["target_thickness_mm"].get<double>(), 1.5, 1e-6);
    EXPECT_NEAR(out.signature.pattern["effective_reduction_pct"].get<double>(), 25.0, 1e-3);
}

// ── 2. DFM-ROLL-PASS: > 50% reduction per pass rejected ────────────────
TEST(SkillRolling, ValidateRejectsExcessiveReductionPct)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 2.0);

    skill::rolling::Input in;
    in.entry_face             = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.target_thickness_mm    = 0.5;       // 75% reduction
    in.reduction_pct_per_pass = 80.0;

    auto r = skill::rolling::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ROLL-PASS") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::rolling::apply(*stock, in), skill::SkillError);
}

// ── 3. DFM-ROLL-MIN: target_thickness < 0.1 × original rejected ────────
TEST(SkillRolling, ValidateRejectsBelowMinFraction)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 2.0);

    skill::rolling::Input in;
    in.entry_face             = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.target_thickness_mm    = 0.15;      // < 0.2 (= 0.1 × 2.0)
    in.reduction_pct_per_pass = 30.0;

    auto r = skill::rolling::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ROLL-MIN") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 4. DFM-INPUT: target >= original rejected (rolling REDUCES) ────────
TEST(SkillRolling, ValidateRejectsTargetThicker)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 2.0);

    skill::rolling::Input in;
    in.entry_face             = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.target_thickness_mm    = 2.5;       // > 2.0 input
    in.reduction_pct_per_pass = 30.0;

    auto r = skill::rolling::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ── 5. Recognize returns the current thickness and metadata if any ─────
TEST(SkillRolling, RecognizeReturnsThickness)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 2.0);

    skill::rolling::Input in;
    in.entry_face             = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.target_thickness_mm    = 1.5;
    in.reduction_pct_per_pass = 30.0;

    auto out = skill::rolling::apply(*stock, in);
    auto cands = skill::rolling::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("rolling"));
    EXPECT_NEAR(c.recovered_params["target_thickness_mm"].get<double>(), 1.5, 1e-6);
    EXPECT_NEAR(c.recovered_params["original_thickness_mm"].get<double>(), 2.0, 1e-6);
    EXPECT_GE(c.confidence, 0.9);          // metadata present
}
