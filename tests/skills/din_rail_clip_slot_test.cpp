// @lat: [[process/test-strategy#compound feature]]
//
// din_rail_clip_slot — 3-cut compound: rail groove + hook ledge + chamfer.
//
// Test cases:
//   1. ApplyProducesValidShapeWithMultipleSubFeatures — volume + face change
//   2. ValidateRejectsBadInput — sub-min rail length
//   3. SignatureRecordsCompoundKind — pattern.is_compound + subfeature_count
//   4. RecognizeMetadataReplay — confidence 1.0 from history
//   5. HookGrooveIsOffsetByDinSpec — hook offset = 17 mm per EN 60715

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/din_rail_clip_slot.hpp"

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

// ─── 1. Apply produces valid shape with multiple sub-features ─────────────
TEST(SkillDinRailClipSlot, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(150.0, 80.0, 10.0);

    skill::din_rail_clip_slot::Input in;
    in.enclosure_face  = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.center_x_mm     = 75.0;
    in.center_y_mm     = 40.0;
    in.axis_dir        = gp_Dir(0, 0, -1);
    in.rail_dir_deg    = 0.0;
    in.rail_length_mm  = 100.0;

    const int facesBefore = stock->faceCount();
    auto out = skill::din_rail_clip_slot::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected total removed = groove (100 × 27.5 × 5.5) + hook (100 × 5 × 2)
    //                          + chamfer (100 × 0.8 × 0.8)
    const double grooveV  = 100.0 * 27.5 * 5.5;
    const double hookV    = 100.0 * 5.0  * 2.0;
    const double chamferV = 100.0 * 0.8  * 0.8;
    const double approx   = grooveV + hookV + chamferV;
    const double removed  = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    // ±20% tolerance — chamfer may overlap groove edge.
    EXPECT_NEAR(removed, approx, approx * 0.2);

    // Face count must grow (3 sub-features add multiple walls).
    EXPECT_GT(out.workpiece->faceCount(), facesBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("din_rail_clip_slot"));
}

// ─── 2. Validate rejects bad input (rail too short) ───────────────────────
TEST(SkillDinRailClipSlot, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(150.0, 80.0, 10.0);

    skill::din_rail_clip_slot::Input in;
    in.enclosure_face  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm     = 75.0;
    in.center_y_mm     = 40.0;
    in.rail_length_mm  = 20.0;   // < 40 mm min

    auto r = skill::din_rail_clip_slot::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::din_rail_clip_slot::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records compound kind + subfeature_count ────────────────
TEST(SkillDinRailClipSlot, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(150.0, 80.0, 10.0);

    skill::din_rail_clip_slot::Input in;
    in.enclosure_face  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm     = 75.0;
    in.center_y_mm     = 40.0;
    in.rail_length_mm  = 80.0;
    auto out = skill::din_rail_clip_slot::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    ASSERT_TRUE(pat.contains("is_compound"));
    EXPECT_TRUE(pat.at("is_compound").get<bool>());
    ASSERT_TRUE(pat.contains("subfeature_count"));
    EXPECT_GE(pat.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(pat.at("kind").get<std::string>(), std::string("din_rail_clip_slot"));
    ASSERT_TRUE(pat.contains("subfeatures"));
    EXPECT_EQ(pat.at("subfeatures").size(), 3u);
}

// ─── 4. Recognize: metadata replay → confidence 1.0 ───────────────────────
TEST(SkillDinRailClipSlot, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(150.0, 80.0, 10.0);
    skill::din_rail_clip_slot::Input in;
    in.enclosure_face  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm     = 75.0;
    in.center_y_mm     = 40.0;
    in.rail_length_mm  = 80.0;
    auto out = skill::din_rail_clip_slot::apply(*stock, in);

    auto cands = skill::din_rail_clip_slot::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("din_rail_clip_slot"));
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["rail_length_mm"].get<double>(),
                80.0, 1e-6);
}

// ─── 5. Hook offset matches EN 60715 (17 mm from rail centerline) ─────────
TEST(SkillDinRailClipSlot, HookGrooveIsOffsetByDinSpec)
{
    auto stock = skill::createCuboidStock(150.0, 80.0, 10.0);
    skill::din_rail_clip_slot::Input in;
    in.enclosure_face  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm     = 75.0;
    in.center_y_mm     = 40.0;
    in.rail_length_mm  = 80.0;
    auto out = skill::din_rail_clip_slot::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    EXPECT_NEAR(pat.at("hook_offset_mm").get<double>(), 17.0, 1e-6);
    EXPECT_NEAR(pat.at("groove_width_mm").get<double>(), 27.5, 1e-6);
    EXPECT_NEAR(pat.at("groove_depth_mm").get<double>(), 5.5, 1e-6);
}
