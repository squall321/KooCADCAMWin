// @lat: [[process/test-strategy#compound diving_bezel_120clicks]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/diving_bezel_120clicks.hpp"

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

skill::diving_bezel_120clicks::Input goodInput() {
    skill::diving_bezel_120clicks::Input in;
    in.case_top_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.case_axis       = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    in.outer_dia_mm    = 38.0;
    in.inner_dia_mm    = 32.0;
    in.height_mm       = 1.5;
    in.click_count     = 120;
    in.notch_dia_mm    = 0.6;
    in.notch_depth_mm  = 0.5;
    return in;
}
}  // namespace

// ─── 1. Apply removes annular bezel volume + 120 notch volume ────────────
TEST(SkillDivingBezel, DISABLED_ApplyRemovesAnnularPlusNotchVolume)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::diving_bezel_120clicks::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double rOuter = 19.0;
    const double rInner = 16.0;
    const double annularVol = M_PI * (rOuter * rOuter - rInner * rInner) * 1.5;
    const double notchVol   = M_PI * 0.3 * 0.3 * 0.5 * 120;
    const double expected   = annularVol + notchVol;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());

    // OCCT may approximate; require diff > 90% of annular alone (the notches
    // overlap the inner bezel wall slightly so they don't add full notchVol).
    EXPECT_GT(diff, annularVol * 0.90);
    EXPECT_LT(diff, expected * 1.05);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects bad outer < inner ───────────────────────────────
TEST(SkillDivingBezel, ValidateRejectsBadGeom)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    in.outer_dia_mm = 30.0;   // < inner_dia_mm (32.0)

    auto r = skill::diving_bezel_120clicks::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BEZEL-GEOM") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::diving_bezel_120clicks::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects out-of-range click_count ────────────────────────
TEST(SkillDivingBezel, ValidateRejectsBadClickCount)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    in.click_count = 30;   // < 60

    auto r = skill::diving_bezel_120clicks::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CLICK-COUNT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records compound + watch + click_count ─────────────────
TEST(SkillDivingBezel, SignatureRecordsCompoundAndWatchFeature)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::diving_bezel_120clicks::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("diving_bezel_120clicks"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_TRUE(out.signature.pattern.at("is_watch_feature").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("click_count").get<int>(), 120);
    EXPECT_NEAR(out.signature.pattern.at("click_step_deg").get<double>(),
                3.0, 1e-6);
}

// ─── 5. Recognize via metadata replay ────────────────────────────────────
TEST(SkillDivingBezel, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::diving_bezel_120clicks::apply(*stock, in);

    auto cands = skill::diving_bezel_120clicks::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("click_count").get<int>(), 120);
    EXPECT_NEAR(cands[0].recovered_params.at("outer_dia_mm").get<double>(),
                38.0, 1e-6);
}
