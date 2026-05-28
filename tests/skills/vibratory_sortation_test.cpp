// @lat: [[process/test-strategy#skill round-trip]]
//
// vibratory_sortation — 5-case sweep covering identity geometry, DFM
// rejection of out-of-range orientation_quality_pct, metadata replay,
// and cycle-time arithmetic.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/vibratory_sortation.hpp"

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

}  // namespace

// ── 1. Apply: geometry is COMPLETELY unchanged ───────────────────────────
TEST(SkillVibratorySortation, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::vibratory_sortation::Input in;
    in.feed_rate_per_min       = 120.0;
    in.orientation_quality_pct = 95.0;

    auto out = skill::vibratory_sortation::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("vibratory_sortation"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: feed rate ≤ 0 and quality outside [0, 100] rejected ──────────
TEST(SkillVibratorySortation, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::vibratory_sortation::Input zero;
    zero.feed_rate_per_min       = 0.0;
    zero.orientation_quality_pct = 95.0;
    EXPECT_FALSE(skill::vibratory_sortation::validate(*stock, zero).passed);
    EXPECT_THROW(skill::vibratory_sortation::apply(*stock, zero),
                 skill::SkillError);

    skill::vibratory_sortation::Input negRate;
    negRate.feed_rate_per_min       = -10.0;
    negRate.orientation_quality_pct = 95.0;
    EXPECT_FALSE(skill::vibratory_sortation::validate(*stock, negRate).passed);

    skill::vibratory_sortation::Input over;
    over.feed_rate_per_min       = 60.0;
    over.orientation_quality_pct = 110.0;            // > 100
    EXPECT_FALSE(skill::vibratory_sortation::validate(*stock, over).passed);

    skill::vibratory_sortation::Input under;
    under.feed_rate_per_min       = 60.0;
    under.orientation_quality_pct = -5.0;            // < 0
    EXPECT_FALSE(skill::vibratory_sortation::validate(*stock, under).passed);
}

// ── 3. Signature records kind + feed_rate + orientation_quality ──────────
TEST(SkillVibratorySortation, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::vibratory_sortation::Input in;
    in.feed_rate_per_min       = 150.0;
    in.orientation_quality_pct = 92.5;

    auto out = skill::vibratory_sortation::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("vibratory_sortation"));
    EXPECT_NEAR(out.signature.pattern["feed_rate_per_min"].get<double>(),
                150.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["orientation_quality_pct"].get<double>(),
                92.5, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("vibratory_bowl_feeder"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillVibratorySortation, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::vibratory_sortation::Input in;
    in.feed_rate_per_min       = 90.0;
    in.orientation_quality_pct = 88.0;

    auto out = skill::vibratory_sortation::apply(*stock, in);
    auto cands = skill::vibratory_sortation::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(
        cands[0].recovered_params["feed_rate_per_min"].get<double>(),
        90.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::vibratory_sortation::recognize(raw).empty());
}

// ── 5. Yield gate: < 50% blocks, 50-80% warns, ≥ 80% passes silently ─────
TEST(SkillVibratorySortation, YieldGateTiering)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    // < 50% → error (blocks)
    skill::vibratory_sortation::Input bad;
    bad.feed_rate_per_min       = 60.0;
    bad.orientation_quality_pct = 30.0;
    auto r1 = skill::vibratory_sortation::validate(*stock, bad);
    EXPECT_FALSE(r1.passed);
    bool errCode = false;
    for (const auto& f : r1.findings) {
        if (f.code == "DFM-VIB-YIELD" && f.severity == "error") {
            errCode = true; break;
        }
    }
    EXPECT_TRUE(errCode);
    EXPECT_THROW(skill::vibratory_sortation::apply(*stock, bad),
                 skill::SkillError);

    // 50-80% → warning (still passes)
    skill::vibratory_sortation::Input meh;
    meh.feed_rate_per_min       = 60.0;
    meh.orientation_quality_pct = 70.0;
    auto r2 = skill::vibratory_sortation::validate(*stock, meh);
    EXPECT_TRUE(r2.passed);
    bool warned = false;
    for (const auto& f : r2.findings) {
        if (f.code == "DFM-VIB-YIELD" && f.severity == "warning") {
            warned = true; break;
        }
    }
    EXPECT_TRUE(warned);

    // ≥ 80% → no yield finding
    skill::vibratory_sortation::Input good;
    good.feed_rate_per_min       = 60.0;
    good.orientation_quality_pct = 95.0;
    auto r3 = skill::vibratory_sortation::validate(*stock, good);
    EXPECT_TRUE(r3.passed);
    for (const auto& f : r3.findings) {
        EXPECT_NE(f.code, std::string("DFM-VIB-YIELD"));
    }

    // cycle_time_per_part_s = 60 / feed_rate
    auto out = skill::vibratory_sortation::apply(*stock, good);
    EXPECT_NEAR(
        out.signature.pattern["cycle_time_per_part_s"].get<double>(),
        1.0, 1e-9);    // 60 / 60 = 1 s/part
}
