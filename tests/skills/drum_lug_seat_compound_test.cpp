// @lat: [[process/test-strategy#drum_lug_seat_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drum_lug_seat_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::drum_lug_seat_compound::Input goodInput()
{
    skill::drum_lug_seat_compound::Input in;
    in.face_id          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm      = 30.0;
    in.center_y_mm      = 30.0;
    in.axis_dir         = gp_Dir(0, 0, -1);
    in.lug_size         = "small";   // default #8-32
    in.tap_depth_mm     = 12.0;
    in.insert_recess_dia_mm   = 8.5;
    in.insert_recess_depth_mm = 4.0;
    return in;
}
}  // namespace

// Test 1: apply removes material.
TEST(SkillDrumLugSeatCompound, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::drum_lug_seat_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillDrumLugSeatCompound, ValidateRejectsUnknownThreadKey)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    auto in = goodInput();
    in.thread_size_key = "BOGUS-99";   // unknown

    auto r = skill::drum_lug_seat_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD-KEY") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::drum_lug_seat_compound::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillDrumLugSeatCompound, SignatureRecordsCompound)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::drum_lug_seat_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("drum_lug_seat_compound"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("audio_feature_type", std::string()),
              std::string("drum_lug_seat"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 2);
    EXPECT_EQ(out.signature.pattern.value("thread_size_key", std::string()),
              std::string("#8-32"));
    // Pilot dia for #8-32 from the central UNC table = 3.55 mm.
    EXPECT_NEAR(out.signature.pattern.value("pilot_dia_mm", 0.0), 3.55, 1e-6);
}

TEST(SkillDrumLugSeatCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    auto in    = goodInput();
    in.lug_size = "large";   // → 1/4-20 default
    auto out   = skill::drum_lug_seat_compound::apply(*stock, in);
    auto cands = skill::drum_lug_seat_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.value("lug_size", std::string()),
              std::string("large"));
}
