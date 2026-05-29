// @lat: [[process/test-strategy#compound gauge_block_step]]
//
// Verifies REAL stepped geometry: N successive rectangular box-cuts at
// monotonically increasing depths.  Volume delta sums the staircase
// material removed.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/gauge_block_step.hpp"

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

// ─── 1. apply: stair-shaped volume removal ───────────────────────────────
TEST(SkillGaugeBlockStep, ApplyRemovesStaircaseVolume)
{
    // Block 80 × 40 × 20.
    auto stock = skill::createCuboidStock(80.0, 40.0, 20.0);
    const int    stockFaces = stock->faceCount();
    const double stockVol   = volumeOf(stock->shape());

    skill::gauge_block_step::Input in;
    in.top_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.x_start_mm      = 10.0;
    in.step_length_mm  = 10.0;
    in.y_full_mm       = 40.0;
    in.step_depths_mm  = { 1.0, 3.0, 5.0 };

    auto out = skill::gauge_block_step::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Each step floor extends to the +X end of the block.
    //   step 0: x ∈ [10, 80], dx = 70, depth = 1
    //   step 1: x ∈ [20, 80], dx = 60, depth = 3 (incremental: 2)
    //   step 2: x ∈ [30, 80], dx = 50, depth = 5 (incremental: 2)
    // Removed volume = ∑ dx · y · (d_i − d_{i-1})
    const double Y = 40.0;
    const double removedExpected =
        70.0 * Y * 1.0 +
        60.0 * Y * 2.0 +
        50.0 * Y * 2.0;
    const double removed = stockVol - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, removedExpected, removedExpected * 0.05);

    // N steps → expect at least (N) new horizontal floors → faceCount grew.
    EXPECT_GT(out.workpiece->faceCount(), stockFaces);
}

// ─── 2. validate rejects non-monotonic + sub-min step ────────────────────
TEST(SkillGaugeBlockStep, ValidateRejectsBadDepths)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 20.0);

    // (a) non-monotonic
    {
        skill::gauge_block_step::Input in;
        in.top_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.x_start_mm     = 10.0;
        in.step_length_mm = 10.0;
        in.step_depths_mm = { 3.0, 1.0, 5.0 };
        auto r = skill::gauge_block_step::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-STEP-ORDER") { found = true; break; }
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::gauge_block_step::apply(*stock, in), skill::SkillError);
    }
    // (b) sub-min depth (0.1 < 0.5)
    {
        skill::gauge_block_step::Input in;
        in.top_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.x_start_mm     = 10.0;
        in.step_length_mm = 10.0;
        in.step_depths_mm = { 0.1, 1.0 };
        auto r = skill::gauge_block_step::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-STEP-DEPTH") { found = true; break; }
        EXPECT_TRUE(found);
    }
    // (c) step_length < 5 mm
    {
        skill::gauge_block_step::Input in;
        in.top_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.x_start_mm     = 10.0;
        in.step_length_mm = 3.0;
        in.step_depths_mm = { 1.0, 2.0 };
        auto r = skill::gauge_block_step::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-STEP-LEN") { found = true; break; }
        EXPECT_TRUE(found);
    }
}

// ─── 3. signature compound + params round-trip ───────────────────────────
TEST(SkillGaugeBlockStep, SignatureCompoundRoundTrip)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 20.0);

    skill::gauge_block_step::Input in;
    in.top_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.x_start_mm     = 5.0;
    in.step_length_mm = 10.0;
    in.step_depths_mm = { 1.0, 2.0, 3.0, 5.0 };

    auto out = skill::gauge_block_step::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 4);
    EXPECT_EQ(out.signature.params.at("step_depths_mm").size(),
              in.step_depths_mm.size());
    EXPECT_NEAR(out.signature.params.at("step_depths_mm")[3].get<double>(),
                5.0, 1e-9);
}

// ─── 4. recognize returns ≥ 1 candidate with confidence > 0.5 ───────────
TEST(SkillGaugeBlockStep, RecognizeReturnsCandidate)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 20.0);
    skill::gauge_block_step::Input in;
    in.top_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.x_start_mm     = 10.0;
    in.step_length_mm = 10.0;
    in.step_depths_mm = { 1.0, 2.0, 3.0 };

    auto out = skill::gauge_block_step::apply(*stock, in);
    auto cands = skill::gauge_block_step::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_GT(cands.front().confidence, 0.5);
    EXPECT_EQ(cands.front().skill_id, std::string("gauge_block_step"));
}

// ─── 5. feature-specific: step_count = depths.size() ─────────────────────
TEST(SkillGaugeBlockStep, StepCountMatches)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 20.0);
    skill::gauge_block_step::Input in;
    in.top_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.x_start_mm     = 10.0;
    in.step_length_mm = 8.0;
    in.step_depths_mm = { 1.0, 2.0, 3.0, 5.0, 10.0 };

    auto out = skill::gauge_block_step::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern.at("step_count").get<int>(),
              static_cast<int>(in.step_depths_mm.size()));
    // Deepest must match input
    const auto depthsArr = out.signature.pattern.at("step_depths_mm");
    EXPECT_NEAR(depthsArr.back().get<double>(), 10.0, 1e-9);
}
