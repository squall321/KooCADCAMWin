// @lat: [[process/test-strategy#compound case_back_screw_down]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/case_back_screw_down.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::case_back_screw_down::Input goodInput() {
    skill::case_back_screw_down::Input in;
    in.case_bottom_face     = skill::FaceByNormal{ gp_Dir(0, 0, -1) };
    in.case_axis            = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    in.back_dia_mm          = 36.0;
    in.thread_pitch_mm      = 0.6;
    in.thread_depth_mm      = 0.5;
    in.thread_band_axial_mm = 1.8;
    in.o_ring_groove_position = "flat";
    in.o_ring_size_key      = "-908";   // CS = 1.27, fits 36 mm back
    return in;
}
}  // namespace

// ─── 1. Apply removes thread band + O-ring groove volume ─────────────────
TEST(SkillCaseBackScrewDown, ApplyRemovesThreadAndGroove)
{
    auto stock = skill::createCylindricalStock(40.0, 5.0);

    auto in = goodInput();
    auto out = skill::case_back_screw_down::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected lower bound: thread band volume only.
    const double rBack       = in.back_dia_mm / 2.0;
    const double rThreadIn   = rBack - in.thread_depth_mm;
    const double threadVol   = M_PI * (rBack * rBack - rThreadIn * rThreadIn) *
                               in.thread_band_axial_mm;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_GT(diff, threadVol * 0.5);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects o_ring_size not in AS568 central table ──────────
TEST(SkillCaseBackScrewDown, ValidateRejectsBadORingDash)
{
    auto stock = skill::createCylindricalStock(40.0, 5.0);

    auto in = goodInput();
    in.o_ring_size_key = "-007";   // not in central AS568 table

    auto r = skill::case_back_screw_down::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ORING-SIZE") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::case_back_screw_down::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects pitch out of range ──────────────────────────────
TEST(SkillCaseBackScrewDown, ValidateRejectsBadPitch)
{
    auto stock = skill::createCylindricalStock(40.0, 5.0);

    auto in = goodInput();
    in.thread_pitch_mm = 2.5;   // > 1.5 mm

    auto r = skill::case_back_screw_down::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD-PITCH") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records compound + screw_down_caseback ─────────────────
TEST(SkillCaseBackScrewDown, SignatureRecordsScrewDown)
{
    auto stock = skill::createCylindricalStock(40.0, 5.0);

    auto in = goodInput();
    auto out = skill::case_back_screw_down::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("case_back_screw_down"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_TRUE(out.signature.pattern.at("is_watch_feature").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("watch_feature_type").get<std::string>(),
              std::string("screw_down_caseback"));
    EXPECT_EQ(out.signature.pattern.at("o_ring_dash").get<std::string>(),
              std::string("-908"));
    EXPECT_NEAR(out.signature.pattern.at("o_ring_cross_section_mm").get<double>(),
                1.27, 1e-6);
}

// ─── 5. Recognize via metadata replay ────────────────────────────────────
TEST(SkillCaseBackScrewDown, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 5.0);

    auto in = goodInput();
    auto out = skill::case_back_screw_down::apply(*stock, in);

    auto cands = skill::case_back_screw_down::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("o_ring_size_key").get<std::string>(),
              std::string("-908"));
}
