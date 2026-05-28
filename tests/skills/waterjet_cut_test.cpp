// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/waterjet_cut.hpp"

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

// ── 1. Linear cut: smooth, narrow kerf ───────────────────────────────────
TEST(SkillWaterjetCut, ApplyLinearThroughCut)
{
    auto stock = skill::createCuboidStock(60.0, 30.0, 12.0);

    skill::waterjet_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::waterjet_cut::Input::Kind::Linear;
    in.start_x_mm = 10.0; in.start_y_mm = 15.0;
    in.end_x_mm   = 50.0; in.end_y_mm   = 15.0;

    auto out = skill::waterjet_cut::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double kerf = 0.9;
    const double r    = kerf / 2.0;
    const double L    = 40.0;
    const double expected = (M_PI * r * r + L * kerf) * 12.0;
    const double removed  = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.20);

    EXPECT_EQ(out.signature.pattern["tool_type"].get<std::string>(), "waterjet");
    EXPECT_NEAR(out.signature.pattern["kerf_mm"].get<double>(), 0.9, 1e-6);
}

// ── 2. Polyline cut, closed contour ──────────────────────────────────────
TEST(SkillWaterjetCut, ApplyPolylineClosedContour)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 15.0);

    skill::waterjet_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::waterjet_cut::Input::Kind::Polyline;
    in.waypoints = {
        { 20.0, 20.0 },
        { 60.0, 20.0 },
        { 60.0, 60.0 },
        { 20.0, 60.0 },
        { 20.0, 20.0 },   // closed
    };

    auto out = skill::waterjet_cut::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_GT(volumeOf(stock->shape()), volumeOf(out.workpiece->shape()));
    EXPECT_EQ(out.signature.pattern["cut_kind"].get<std::string>(), "Polyline");
}

// ── 3. Thickness OK at 100 mm — borderline warning at 90% ────────────────
TEST(SkillWaterjetCut, DfmBorderlineThicknessWarning)
{
    // 190 mm > 0.9 × 200, but ≤ 200 → warning, not error.
    auto stock = skill::createCuboidStock(40.0, 40.0, 190.0);

    skill::waterjet_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::waterjet_cut::Input::Kind::Linear;
    in.start_x_mm = 10.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 30.0; in.end_y_mm   = 20.0;

    auto r = skill::waterjet_cut::validate(*stock, in);
    EXPECT_TRUE(r.passed);   // warning doesn't fail validate
    bool warnFound = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THICKNESS" && f.severity == "warning") {
            warnFound = true; break;
        }
    EXPECT_TRUE(warnFound);
}

// ── 4. DFM-THICKNESS error above 200 mm ──────────────────────────────────
TEST(SkillWaterjetCut, DfmExceededThicknessRejected)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 250.0);

    skill::waterjet_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::waterjet_cut::Input::Kind::Linear;
    in.start_x_mm = 10.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 30.0; in.end_y_mm   = 20.0;

    auto r = skill::waterjet_cut::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::waterjet_cut::apply(*stock, in), skill::SkillError);
}

// ── 5. Surface-finish info finding emitted ───────────────────────────────
TEST(SkillWaterjetCut, DfmSmoothFinishInfo)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::waterjet_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::waterjet_cut::Input::Kind::Linear;
    in.start_x_mm = 10.0; in.start_y_mm = 25.0;
    in.end_x_mm   = 40.0; in.end_y_mm   = 25.0;

    auto r = skill::waterjet_cut::validate(*stock, in);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SURFACE" && f.severity == "info"
            && f.message.find("smooth") != std::string::npos) { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 6. Recognize at kerf ≈ 0.9 mm → waterjet conf 0.7 ────────────────────
// Fixed: BRepPrimAPI_MakeCylinder's seam (at local +X) survives the
// stadium-cutter boolean on the END (forward-from-start) cap because that
// half of the cylinder is OUTSIDE the connecting box, so OCCT splits the
// surviving half-cyl into two quarter-cyl faces with identical axis/radius.
// scanThroughChannels() then mis-paired them as a concentric annular sector
// (kerf = |r-r| = 0), which dropped into the `kerf < 0.5` band → waterjet
// confidence 0.5.  The fix adds a seam-split guard in the concentric pair
// scan (skip when kerf < 1e-3) so the fragments fall through to the lone
// emitter and report kerf = 2 × radius = 0.9 mm → conf 0.7.
TEST(SkillWaterjetCut, RecognizeReturnsConfWeightedCandidates)
{
    auto stock = skill::createCuboidStock(60.0, 30.0, 10.0);

    skill::waterjet_cut::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.kind = skill::waterjet_cut::Input::Kind::Linear;
    in.start_x_mm = 10.0; in.start_y_mm = 15.0;
    in.end_x_mm   = 50.0; in.end_y_mm   = 15.0;

    auto out = skill::waterjet_cut::apply(*stock, in);
    auto cands = skill::waterjet_cut::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    // kerf 0.9 → waterjet 0.7
    EXPECT_NEAR(cands[0].confidence, 0.70, 1e-9);
}
