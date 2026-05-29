// @lat: [[process/test-strategy#compound macro]]
//
// intermittent_fillet_pattern — N segment witness marks along an edge.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/intermittent_fillet_pattern.hpp"

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

// ─── 1. Apply produces valid shape with multiple sub-features ────────────
TEST(SkillIntermittentFilletPattern, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(400.0, 40.0, 10.0);
    const int faces0 = stock->faceCount();

    skill::intermittent_fillet_pattern::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.edge_dir         = gp_Dir(1, 0, 0);
    in.segment_count    = 4;
    in.segment_length   = 50.0;
    in.pitch            = 75.0;
    in.edge_center_x_mm = 200.0;
    in.edge_center_y_mm = 20.0;
    in.witness_depth_mm = 0.1;
    in.witness_width_mm = 2.0;

    auto out = skill::intermittent_fillet_pattern::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume removed ≈ N × L × W × D witness marks.
    const double approx = in.segment_count * in.segment_length *
                          in.witness_width_mm * in.witness_depth_mm;
    const double removed = volumeOf(stock->shape()) -
                           volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.20);

    // Face count must reflect N witness marks (each adds faces).
    EXPECT_GT(out.workpiece->faceCount(), faces0);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillIntermittentFilletPattern, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(400.0, 40.0, 10.0);

    // 2a. segment_count < 2
    skill::intermittent_fillet_pattern::Input bad1;
    bad1.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad1.edge_dir         = gp_Dir(1, 0, 0);
    bad1.segment_count    = 1;
    bad1.segment_length   = 50.0;
    bad1.pitch            = 75.0;
    auto r1 = skill::intermittent_fillet_pattern::validate(*stock, bad1);
    EXPECT_FALSE(r1.passed);

    // 2b. segment_length < 25 mm
    skill::intermittent_fillet_pattern::Input bad2 = bad1;
    bad2.segment_count    = 4;
    bad2.segment_length   = 10.0;
    bad2.pitch            = 30.0;
    auto r2 = skill::intermittent_fillet_pattern::validate(*stock, bad2);
    EXPECT_FALSE(r2.passed);

    // 2c. pitch < segment_length
    skill::intermittent_fillet_pattern::Input bad3 = bad1;
    bad3.segment_count    = 4;
    bad3.segment_length   = 50.0;
    bad3.pitch            = 40.0;
    auto r3 = skill::intermittent_fillet_pattern::validate(*stock, bad3);
    EXPECT_FALSE(r3.passed);

    EXPECT_THROW(skill::intermittent_fillet_pattern::apply(*stock, bad1),
                 skill::SkillError);
}

// ─── 3. Signature records compound kind & subfeature_count == N ─────────
TEST(SkillIntermittentFilletPattern, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(400.0, 40.0, 10.0);
    skill::intermittent_fillet_pattern::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.edge_dir         = gp_Dir(1, 0, 0);
    in.segment_count    = 5;
    in.segment_length   = 40.0;
    in.pitch            = 60.0;
    in.edge_center_x_mm = 200.0;
    in.edge_center_y_mm = 20.0;
    auto out = skill::intermittent_fillet_pattern::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("intermittent_fillet_pattern"));
    EXPECT_TRUE(pat.at("is_compound").get<bool>());
    EXPECT_EQ(pat.at("subfeature_count").get<int>(), 5);
    EXPECT_EQ(pat.at("segment_count").get<int>(), 5);
    EXPECT_NEAR(pat.at("segment_length").get<double>(), 40.0, 1e-6);
    EXPECT_NEAR(pat.at("pitch").get<double>(), 60.0, 1e-6);
    EXPECT_TRUE(pat.contains("gap_mm"));
    EXPECT_TRUE(pat.contains("duty_cycle"));
    EXPECT_EQ(pat.at("segments").size(), 5u);
}

// ─── 4. Recognize metadata replay ────────────────────────────────────────
TEST(SkillIntermittentFilletPattern, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(400.0, 40.0, 10.0);
    skill::intermittent_fillet_pattern::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.edge_dir         = gp_Dir(1, 0, 0);
    in.segment_count    = 3;
    in.segment_length   = 40.0;
    in.pitch            = 60.0;
    in.edge_center_x_mm = 200.0;
    in.edge_center_y_mm = 20.0;
    auto out = skill::intermittent_fillet_pattern::apply(*stock, in);

    auto cands = skill::intermittent_fillet_pattern::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("segment_count").get<int>(), 3);
}

// ─── 5. Pattern math: total_length = (N−1) × pitch + L; gap = pitch − L ──
TEST(SkillIntermittentFilletPattern, PatternMathRelations)
{
    auto stock = skill::createCuboidStock(400.0, 40.0, 10.0);
    skill::intermittent_fillet_pattern::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.edge_dir         = gp_Dir(1, 0, 0);
    in.segment_count    = 4;
    in.segment_length   = 50.0;
    in.pitch            = 100.0;
    in.edge_center_x_mm = 200.0;
    in.edge_center_y_mm = 20.0;
    auto out = skill::intermittent_fillet_pattern::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    // total = (4−1) × 100 + 50 = 350
    EXPECT_NEAR(pat.at("total_pattern_length_mm").get<double>(), 350.0, 1e-6);
    // gap = 100 − 50 = 50
    EXPECT_NEAR(pat.at("gap_mm").get<double>(), 50.0, 1e-6);
    // duty = 50 / 100 = 0.5
    EXPECT_NEAR(pat.at("duty_cycle").get<double>(), 0.5, 1e-6);
}
