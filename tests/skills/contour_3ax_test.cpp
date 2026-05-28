// @lat: [[process/test-strategy#skill round-trip]]
//
// contour_3ax — 3-axis surface following along a 3D polyline.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/contour_3ax.hpp"

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

// ── 1. Apply removes material along the contour ──────────────────────────
TEST(SkillContour3ax, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 20.0);

    skill::contour_3ax::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.tool_dia_mm  = 4.0;
    in.contour_polyline_3d = {
        { 10.0, 20.0, 18.0 },
        { 20.0, 20.0, 16.0 },
        { 30.0, 20.0, 14.0 },
        { 40.0, 20.0, 12.0 },
        { 50.0, 20.0, 10.0 },
    };

    auto out = skill::contour_3ax::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Something was removed (≥ 1 mm³).  Exact volume is hard to compute
    // because of overlap between adjacent segment cutters.
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 1.0);

    EXPECT_EQ(out.signature.skill_id, std::string("contour_3ax"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("contour_3ax"));
    EXPECT_EQ(out.signature.pattern["segment_count"].get<size_t>(), 4u);
}

// ── 2. DFM-002: sub-min tool rejected ────────────────────────────────────
TEST(SkillContour3ax, ValidateRejectsSubMinTool)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::contour_3ax::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.tool_dia_mm  = 0.4;
    in.contour_polyline_3d = {
        { 10.0, 25.0, 9.0 },
        { 40.0, 25.0, 8.0 },
    };

    auto r = skill::contour_3ax::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-002") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 3. DFM-3AX-PTS: empty / single-point polyline rejected ───────────────
TEST(SkillContour3ax, ValidateRejectsTooFewPoints)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::contour_3ax::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.tool_dia_mm  = 4.0;
    in.contour_polyline_3d = {
        { 25.0, 25.0, 9.0 },     // only 1 point
    };

    auto r = skill::contour_3ax::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-3AX-PTS") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::contour_3ax::apply(*stock, in), skill::SkillError);
}

// ── 4. DFM-3AX-BBOX: point outside workpiece bbox rejected ───────────────
TEST(SkillContour3ax, ValidateRejectsOutOfBbox)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::contour_3ax::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.tool_dia_mm  = 4.0;
    in.contour_polyline_3d = {
        { 25.0, 25.0, 9.0 },
        { 100.0, 25.0, 8.0 },     // x=100 outside [0, 50]
    };

    auto r = skill::contour_3ax::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-3AX-BBOX") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 5. Signature records the segments with varying Z floors ──────────────
TEST(SkillContour3ax, SignatureRecordsSegmentsWithVaryingZ)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 20.0);
    skill::contour_3ax::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.tool_dia_mm  = 5.0;
    in.contour_polyline_3d = {
        { 10.0, 20.0, 18.0 },
        { 30.0, 20.0, 14.0 },
        { 50.0, 20.0, 10.0 },
    };

    auto out = skill::contour_3ax::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    ASSERT_TRUE(pat.contains("segments"));
    EXPECT_EQ(pat.at("segments").size(), 2u);
    // The second segment's floor (= min of {14, 10}) should be 10.
    EXPECT_NEAR(pat.at("segments").at(1).at("z_floor").get<double>(),
                10.0, 1e-6);
    // Tool type is a ball-mill for 3-axis surface follow.
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("ball_mill"));
}

// ── 6. Recognize via feature history ─────────────────────────────────────
TEST(SkillContour3ax, RecognizeFindsViaHistory)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 20.0);
    skill::contour_3ax::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.tool_dia_mm  = 4.0;
    in.contour_polyline_3d = {
        { 10.0, 20.0, 18.0 },
        { 25.0, 20.0, 15.0 },
        { 40.0, 20.0, 12.0 },
    };

    auto out = skill::contour_3ax::apply(*stock, in);
    auto cands = skill::contour_3ax::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands[0].skill_id, std::string("contour_3ax"));
    EXPECT_GE(cands[0].confidence, 0.80);
    EXPECT_NEAR(cands[0].recovered_params["tool_dia_mm"].get<double>(),
                4.0, 1e-6);
    ASSERT_TRUE(cands[0].recovered_params.contains("contour_polyline_3d"));
    EXPECT_EQ(cands[0].recovered_params["contour_polyline_3d"].size(), 3u);
}
