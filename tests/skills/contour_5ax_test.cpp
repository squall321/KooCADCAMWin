// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/contour_5ax.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

skill::contour_5ax::Input makeValidInput()
{
    skill::contour_5ax::Input in;
    in.entry_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.contour_polyline_3d = {
        // (x, y, z, tilt_deg)
        { 20.0, 30.0, 13.0,  5.0 },
        { 30.0, 30.0, 13.0, 10.0 },
        { 40.0, 30.0, 13.0, 15.0 },
        { 50.0, 30.0, 13.0, 20.0 },
    };
    in.tool_dia_mm    = 5.0;
    in.tool_length_mm = 20.0;
    return in;
}

}  // namespace

// ── 1. Apply: chain of varying-tilt cylinders removes material ───────────
TEST(SkillContour5ax, ApplyCreatesVaryingTiltChain)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 15.0);
    const double volBefore = volumeOf(stock->shape());

    auto in = makeValidInput();
    auto out = skill::contour_5ax::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_LT(volumeOf(out.workpiece->shape()), volBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("contour_5ax"));
    EXPECT_TRUE(out.signature.pattern["is_5axis_cut"].get<bool>());
    EXPECT_EQ(out.signature.pattern["waypoint_count"].get<size_t>(), 4u);
    EXPECT_TRUE(out.signature.pattern["tilt_varies"].get<bool>());
}

// ── 2. DFM-002: small tool rejected ──────────────────────────────────────
TEST(SkillContour5ax, ValidateRejectsSmallTool)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto in = makeValidInput();
    in.tool_dia_mm = 1.0;

    auto r = skill::contour_5ax::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-002") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::contour_5ax::apply(*stock, in), skill::SkillError);
}

// ── 3. DFM-CONTOUR-TILT: per-waypoint tilt > 60° rejected ────────────────
TEST(SkillContour5ax, ValidateRejectsExcessiveTilt)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto in = makeValidInput();
    in.contour_polyline_3d[2][3] = 75.0;   // > 60

    auto r = skill::contour_5ax::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CONTOUR-TILT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 4. DFM-005 info always emitted ───────────────────────────────────────
TEST(SkillContour5ax, ValidateEmitsDFM005Info)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto in = makeValidInput();

    auto r = skill::contour_5ax::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-005" && f.severity == "info") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 5. Recognize: tilted chain candidate emitted (low confidence) ────────
TEST(SkillContour5ax, RecognizeDetectsVaryingTiltChain)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 15.0);
    auto in = makeValidInput();
    auto out = skill::contour_5ax::apply(*stock, in);

    auto cands = skill::contour_5ax::recognize(*out.workpiece);
    if (!cands.empty()) {
        EXPECT_EQ(cands[0].skill_id, std::string("contour_5ax"));
        EXPECT_LE(cands[0].confidence, 0.5);
        EXPECT_GE(cands[0].confidence, 0.2);
    }
}

// ── 6. Empty polyline rejected ───────────────────────────────────────────
TEST(SkillContour5ax, ValidateRejectsEmptyPolyline)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto in = makeValidInput();
    in.contour_polyline_3d.clear();

    auto r = skill::contour_5ax::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}
