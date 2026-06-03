// @lat: [[process/test-strategy#compound gmt_bezel_24hr]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/gmt_bezel_24hr.hpp"

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

skill::gmt_bezel_24hr::Input goodInput() {
    skill::gmt_bezel_24hr::Input in;
    in.case_top_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.case_axis       = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    in.outer_dia_mm    = 38.0;
    in.inner_dia_mm    = 32.0;
    in.height_mm       = 1.5;
    in.hour_marker_count = 24;
    in.marker_dia_mm   = 1.2;
    in.marker_depth_mm = 0.5;
    return in;
}
}  // namespace

// ─── 1. Apply removes annular bezel + 24 markers ─────────────────────────
TEST(SkillGmtBezel, ApplyRemovesBezelAnd24Markers)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::gmt_bezel_24hr::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double rOuter = 19.0;
    const double rInner = 16.0;
    const double bezelVol  = M_PI * (rOuter * rOuter - rInner * rInner) * 1.5;
    const double markerVol = M_PI * 0.6 * 0.6 * 0.5 * 24;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());

    EXPECT_GT(diff, bezelVol * 0.90);
    EXPECT_LT(diff, (bezelVol + markerVol) * 1.10);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects bad marker count ────────────────────────────────
TEST(SkillGmtBezel, ValidateRejectsBadMarkerCount)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    in.hour_marker_count = 36;   // > 24
    auto r = skill::gmt_bezel_24hr::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-MARKER-COUNT") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::gmt_bezel_24hr::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects bad geometry ────────────────────────────────────
TEST(SkillGmtBezel, ValidateRejectsBadGeom)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    in.inner_dia_mm = 40.0;   // > outer_dia_mm
    auto r = skill::gmt_bezel_24hr::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BEZEL-GEOM") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records compound + watch + marker_step_deg = 15 ────────
TEST(SkillGmtBezel, SignatureRecordsWatchFeatureWith15DegStep)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::gmt_bezel_24hr::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("gmt_bezel_24hr"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_TRUE(out.signature.pattern.at("is_watch_feature").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("hour_marker_count").get<int>(), 24);
    EXPECT_NEAR(out.signature.pattern.at("marker_step_deg").get<double>(),
                15.0, 1e-6);
}

// ─── 5. Recognize via metadata replay ────────────────────────────────────
TEST(SkillGmtBezel, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::gmt_bezel_24hr::apply(*stock, in);

    auto cands = skill::gmt_bezel_24hr::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("hour_marker_count").get<int>(), 24);
}
