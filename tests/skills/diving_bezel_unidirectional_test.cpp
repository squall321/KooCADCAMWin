// @lat: [[process/test-strategy#compound diving_bezel_unidirectional]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/diving_bezel_unidirectional.hpp"

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

skill::diving_bezel_unidirectional::Input goodInput() {
    skill::diving_bezel_unidirectional::Input in;
    in.case_top_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.case_axis          = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    in.bezel_outer_dia_mm = 38.0;
    in.bezel_inner_dia_mm = 32.0;
    in.bezel_height_mm    = 1.5;
    in.click_count        = 120;
    in.click_depth_mm     = 0.4;
    in.click_notch_dia_mm = 0.5;
    return in;
}
}  // namespace

// ─── 1. Apply removes annular bezel seat + 120 ratchet notch volume ──────
TEST(SkillDivingBezelUnidirectional, ApplyRemovesSeatAndRatchet)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::diving_bezel_unidirectional::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double rOuter = 19.0;
    const double rInner = 16.0;
    const double seatVol = M_PI * (rOuter * rOuter - rInner * rInner) * 1.5;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());

    EXPECT_GT(diff, seatVol * 0.85);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects bad geometry ────────────────────────────────────
TEST(SkillDivingBezelUnidirectional, ValidateRejectsBadGeom)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    in.bezel_outer_dia_mm = 30.0;   // < inner

    auto r = skill::diving_bezel_unidirectional::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BEZEL-GEOM") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::diving_bezel_unidirectional::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects non-{60,120} click count ────────────────────────
TEST(SkillDivingBezelUnidirectional, ValidateRejectsBadClickCount)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    in.click_count = 90;   // not in {60, 120}

    auto r = skill::diving_bezel_unidirectional::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CLICK-COUNT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records compound + watch + click_count ─────────────────
TEST(SkillDivingBezelUnidirectional, SignatureRecordsCompoundAndWatchFeature)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::diving_bezel_unidirectional::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("diving_bezel_unidirectional"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_TRUE(out.signature.pattern.at("is_watch_feature").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("watch_feature_type").get<std::string>(),
              std::string("diving_bezel_seat_with_ratchet"));
    EXPECT_EQ(out.signature.pattern.at("click_count").get<int>(), 120);
}

// ─── 5. Recognize via metadata replay ────────────────────────────────────
TEST(SkillDivingBezelUnidirectional, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::diving_bezel_unidirectional::apply(*stock, in);

    auto cands = skill::diving_bezel_unidirectional::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("click_count").get<int>(), 120);
}
