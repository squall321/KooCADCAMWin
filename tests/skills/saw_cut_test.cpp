// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/saw_cut.hpp"

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

// ── 1. Linear through-cut removes a stadium-shaped volume ────────────────
TEST(SkillSawCut, ApplyLinearThroughCut)
{
    auto stock = skill::createCuboidStock(100.0, 50.0, 10.0);

    skill::saw_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind        = skill::saw_cut::Input::Kind::Linear;
    in.start_x_mm  = 20.0; in.start_y_mm = 25.0;
    in.end_x_mm    = 80.0; in.end_y_mm   = 25.0;

    auto out = skill::saw_cut::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Stadium: π r² × thickness + L × kerf × thickness, kerf = 2.0 mm.
    const double kerf = 2.0;
    const double r    = kerf / 2.0;
    const double L    = 60.0;
    const double expected = (M_PI * r * r + L * kerf) * 10.0;
    const double removed  = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.15);

    EXPECT_EQ(out.signature.skill_id, std::string("saw_cut"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(), "saw_cut");
    EXPECT_EQ(out.signature.pattern["cut_kind"].get<std::string>(), "Linear");
    EXPECT_NEAR(out.signature.pattern["kerf_mm"].get<double>(), 2.0, 1e-6);
    EXPECT_EQ(out.signature.pattern["tool_type"].get<std::string>(), "circular_saw");
    EXPECT_TRUE(out.signature.pattern["is_through_cut"].get<bool>());
}

// ── 2. Circular through-cut emits an annular sector ──────────────────────
// Bug fix: buildCircularCutter() was passing the inner-arc parameters in
// reversed order (a1, a0) on the assumption that OCCT would produce a
// reversed edge — but OCCT silently normalises the parameter range, so the
// inner arc traversed CCW like the outer arc, producing a degenerate/full-
// disc footprint instead of an annular sector.  Fix builds the inner arc
// CCW (a0, a1) and applies TopoDS_Edge::Reversed() explicitly.
TEST(SkillSawCut, ApplyCircularThroughCut)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 8.0);

    skill::saw_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind            = skill::saw_cut::Input::Kind::Circular;
    in.cx_mm           = 40.0;
    in.cy_mm           = 40.0;
    in.radius_mm       = 20.0;
    in.start_angle_deg = 0.0;
    in.end_angle_deg   = 90.0;

    auto out = skill::saw_cut::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Annular sector volume ≈ kerf × R × Δθ × thickness.
    const double kerf  = 2.0;
    const double dTh   = M_PI / 2.0;
    const double expected = kerf * 20.0 * dTh * 8.0;
    const double removed  = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.20);

    EXPECT_EQ(out.signature.pattern["cut_kind"].get<std::string>(), "Circular");
}

// ── 3. Polyline through-cut on a multi-waypoint zig-zag path ─────────────
TEST(SkillSawCut, ApplyPolylineThroughCut)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 6.0);

    skill::saw_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind      = skill::saw_cut::Input::Kind::Polyline;
    in.waypoints = {
        { 20.0, 20.0 },
        { 80.0, 20.0 },
        { 80.0, 80.0 },
        { 20.0, 80.0 },
    };

    auto out = skill::saw_cut::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_GT(volumeOf(stock->shape()), volumeOf(out.workpiece->shape()));
    EXPECT_EQ(out.signature.pattern["cut_kind"].get<std::string>(), "Polyline");
    EXPECT_GE(out.signature.pattern["path_length_mm"].get<double>(), 180.0 - 1e-3);
}

// ── 4. DFM-THICKNESS error on too-thick stock ────────────────────────────
TEST(SkillSawCut, DfmThicknessRejection)
{
    // 200 mm > saw_cut's 100 mm limit.
    auto stock = skill::createCuboidStock(40.0, 40.0, 200.0);

    skill::saw_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind        = skill::saw_cut::Input::Kind::Linear;
    in.start_x_mm = 5.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 35.0; in.end_y_mm  = 20.0;

    auto r = skill::saw_cut::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THICKNESS" && f.severity == "error") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::saw_cut::apply(*stock, in), skill::SkillError);
}

// ── 5. DFM-SURFACE info finding always emitted ───────────────────────────
TEST(SkillSawCut, DfmEmitsSurfaceFinishInfo)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 8.0);

    skill::saw_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::saw_cut::Input::Kind::Linear;
    in.start_x_mm = 10.0; in.start_y_mm = 25.0;
    in.end_x_mm   = 40.0; in.end_y_mm   = 25.0;

    auto r = skill::saw_cut::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SURFACE" && f.severity == "info"
            && f.message.find("rough") != std::string::npos) { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 6. DFM-INPUT error on degenerate linear path ────────────────────────
TEST(SkillSawCut, DfmRejectsZeroLengthLinear)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);
    skill::saw_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::saw_cut::Input::Kind::Linear;
    in.start_x_mm = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 20.0; in.end_y_mm   = 20.0;  // same as start

    auto r = skill::saw_cut::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ── 7. Recognize: saw_cut returns candidate within its kerf bucket ───────
TEST(SkillSawCut, RecognizeReportsKerfMatchedCandidate)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 6.0);

    skill::saw_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind        = skill::saw_cut::Input::Kind::Linear;
    in.start_x_mm = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 60.0; in.end_y_mm   = 20.0;

    auto out   = skill::saw_cut::apply(*stock, in);
    auto cands = skill::saw_cut::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands[0].skill_id, std::string("saw_cut"));
    // Saw confidence at kerf 2.0 mm → 0.60.
    EXPECT_NEAR(cands[0].confidence, 0.60, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["kerf_mm"].get<double>(), 2.0, 0.05);
}
