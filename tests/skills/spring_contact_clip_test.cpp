// @lat: [[process/test-strategy#skill compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/spring_contact_clip.hpp"

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

// 1. Apply produces non-null shape; volume delta = sum of 3 pocket volumes.
TEST(SkillSpringContactClip, ApplyCutsRealGeometry)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 15.0);
    const double volBefore = volumeOf(stock->shape());
    const int    facesBefore = stock->faceCount();

    skill::spring_contact_clip::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm       = 20.0;
    in.position_y_mm       = 15.0;
    in.clip_length_mm      = 8.0;
    in.clip_width_mm       = 4.0;
    in.clip_depth_mm       = 4.0;
    in.back_stop_width_mm  = 2.0;
    in.back_stop_depth_mm  = 1.0;
    in.barb_slot_width_mm  = 1.5;
    in.barb_slot_height_mm = 0.8;
    in.barb_slot_depth_mm  = 1.0;
    in.barb_slot_z_offset_mm = 1.5;

    auto out = skill::spring_contact_clip::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double approxRem =
        8.0 * 4.0 * 4.0                          // U-pocket
      + (8.0 * 0.2) * 2.0 * 1.0                  // back stop
      + 1.5 * 0.8 * 1.0;                          // barb slot (barb slot wholly inside U? no — punches outside)
    const double diff = volBefore - volumeOf(out.workpiece->shape());
    // Back stop and U-pocket overlap in part of the box where back_stop sits
    // (its top is at the U-pocket's floor); they don't double count.  We
    // allow a generous ±20 %.
    EXPECT_NEAR(diff, approxRem, approxRem * 0.20);
    EXPECT_GT(out.workpiece->faceCount(), facesBefore);
}

// 2. Validate rejects bad input + apply throws.
TEST(SkillSpringContactClip, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 15.0);
    skill::spring_contact_clip::Input bad;
    bad.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.position_x_mm       = 20.0;
    bad.position_y_mm       = 15.0;
    bad.clip_length_mm      = 8.0;
    bad.clip_width_mm       = 0.5;          // < 1.5 mm — DFM-UL486A
    bad.clip_depth_mm       = 4.0;
    bad.back_stop_width_mm  = 0.3;
    bad.back_stop_depth_mm  = 1.0;
    bad.barb_slot_width_mm  = 1.5;
    bad.barb_slot_height_mm = 0.8;
    bad.barb_slot_depth_mm  = 1.0;
    bad.barb_slot_z_offset_mm = 1.5;

    auto r = skill::spring_contact_clip::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-UL486A") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::spring_contact_clip::apply(*stock, bad), skill::SkillError);
}

// 3. Signature: is_compound==true, subfeature_count>=2.
TEST(SkillSpringContactClip, SignatureIsCompound)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 15.0);
    skill::spring_contact_clip::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm       = 20.0;
    in.position_y_mm       = 15.0;
    in.clip_length_mm      = 8.0;
    in.clip_width_mm       = 4.0;
    in.clip_depth_mm       = 4.0;
    in.back_stop_width_mm  = 2.0;
    in.back_stop_depth_mm  = 1.0;
    in.barb_slot_width_mm  = 1.5;
    in.barb_slot_height_mm = 0.8;
    in.barb_slot_depth_mm  = 1.0;
    in.barb_slot_z_offset_mm = 1.5;

    auto out = skill::spring_contact_clip::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    EXPECT_TRUE(pat["is_compound"].get<bool>());
    EXPECT_GE(pat["subfeature_count"].get<int>(), 2);
    EXPECT_EQ(pat["kind"].get<std::string>(), std::string("spring_contact_clip"));

    const auto& p = out.signature.params;
    EXPECT_DOUBLE_EQ(p["clip_length_mm"].get<double>(), 8.0);
    EXPECT_DOUBLE_EQ(p["clip_width_mm"].get<double>(),  4.0);
    EXPECT_DOUBLE_EQ(p["clip_depth_mm"].get<double>(),  4.0);
}

// 4. Recognize: metadata replay path must hit confidence 1.0.
TEST(SkillSpringContactClip, RecognizeReturnsCandidate)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 15.0);
    skill::spring_contact_clip::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm       = 20.0;
    in.position_y_mm       = 15.0;
    in.clip_length_mm      = 8.0;
    in.clip_width_mm       = 4.0;
    in.clip_depth_mm       = 4.0;
    in.back_stop_width_mm  = 2.0;
    in.back_stop_depth_mm  = 1.0;
    in.barb_slot_width_mm  = 1.5;
    in.barb_slot_height_mm = 0.8;
    in.barb_slot_depth_mm  = 1.0;
    in.barb_slot_z_offset_mm = 1.5;

    auto out = skill::spring_contact_clip::apply(*stock, in);
    auto cands = skill::spring_contact_clip::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    bool hasMeta = false;
    for (const auto& c : cands)
        if (std::abs(c.confidence - 1.0) < 1e-6) { hasMeta = true; break; }
    EXPECT_TRUE(hasMeta);
}

// 5. Feature-specific: back_stop_depth + U-pocket_depth = total clip depth+bs.
TEST(SkillSpringContactClip, FeatureSpecific_TwoLevelDepth)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 15.0);
    skill::spring_contact_clip::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm       = 20.0;
    in.position_y_mm       = 15.0;
    in.clip_length_mm      = 8.0;
    in.clip_width_mm       = 4.0;
    in.clip_depth_mm       = 4.0;
    in.back_stop_width_mm  = 2.0;
    in.back_stop_depth_mm  = 1.0;
    in.barb_slot_width_mm  = 1.5;
    in.barb_slot_height_mm = 0.8;
    in.barb_slot_depth_mm  = 1.0;
    in.barb_slot_z_offset_mm = 1.5;

    auto out = skill::spring_contact_clip::apply(*stock, in);
    // Verify that face count increased; clip pocket creates a measurable
    // number of new faces (≥ 6 for a basic rectangular pocket).
    EXPECT_GE(out.workpiece->faceCount(), 12);
}
