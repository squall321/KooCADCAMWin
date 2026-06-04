// @lat: [[process/test-strategy#compound screw_down_caseback_6_notches]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/screw_down_caseback_6_notches.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::screw_down_caseback_6_notches::Input goodInput() {
    skill::screw_down_caseback_6_notches::Input in;
    in.case_bottom_face = skill::FaceByNormal{ gp_Dir(0, 0, -1) };
    in.case_axis        = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    in.caseback_dia_mm  = 30.0;
    in.thread_pitch_mm  = 1.0;
    in.thread_depth_mm  = 2.5;
    in.notch_count      = 6;
    in.notch_width_mm   = 1.5;
    in.notch_depth_mm   = 0.6;
    in.notch_radial_extent_mm = 2.0;
    return in;
}
}  // namespace

// ─── 1. Apply removes thread cylinder + 6 notch volume ────────────────────
TEST(SkillScrewDownCaseback, ApplyRemovesThreadAndNotches)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::screw_down_caseback_6_notches::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double threadVol = M_PI * 15.0 * 15.0 * 2.5;
    const double notchVol  = 2.0 * 1.5 * 0.6 * 6;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());

    EXPECT_GT(diff, threadVol * 0.90);
    EXPECT_LT(diff, (threadVol + notchVol) * 1.15);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects bad thread_pitch ────────────────────────────────
TEST(SkillScrewDownCaseback, ValidateRejectsBadPitch)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    in.thread_pitch_mm = 3.0;   // > 2.0
    auto r = skill::screw_down_caseback_6_notches::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD-PITCH") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::screw_down_caseback_6_notches::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects bad notch_count ─────────────────────────────────
TEST(SkillScrewDownCaseback, ValidateRejectsBadNotchCount)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    in.notch_count = 12;   // > 8
    auto r = skill::screw_down_caseback_6_notches::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-NOTCH-COUNT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records compound + watch + 6 notches ───────────────────
TEST(SkillScrewDownCaseback, SignatureRecordsWatchFeature)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::screw_down_caseback_6_notches::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("screw_down_caseback_6_notches"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_TRUE(out.signature.pattern.at("is_watch_feature").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("notch_count").get<int>(), 6);
    EXPECT_NEAR(out.signature.pattern.at("notch_step_deg").get<double>(),
                60.0, 1e-6);
}

// ─── 5. Recognize via metadata replay ────────────────────────────────────
TEST(SkillScrewDownCaseback, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::screw_down_caseback_6_notches::apply(*stock, in);

    auto cands = skill::screw_down_caseback_6_notches::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("notch_count").get<int>(), 6);
}
