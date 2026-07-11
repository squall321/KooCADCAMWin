// @lat: [[process/test-strategy#skill round-trip]]
//
// fillet_edge skill — 5-case sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_hole.hpp"
#include "skills/fillet_edge.hpp"
#include "skills/mill_slot.hpp"

#include <memory>

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <algorithm>
#include <cmath>
#include <vector>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. Apply: fillet top rim of cuboid ───────────────────────────────────
TEST(SkillFilletEdge, ApplyTopRimSucceeds)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const int facesBefore = stock->faceCount();
    const double volBefore = volumeOf(stock->shape());

    skill::fillet_edge::Input in;
    in.edge_selector = skill::fillet_edge::EdgesAtZBand{ 10.0, 1e-3 };
    in.radius_mm     = 1.0;

    auto out = skill::fillet_edge::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // A cuboid filleted on top rim gains 4 new toroidal/cylindrical blend
    // faces (or one swept face per edge).  At minimum, face count grows.
    EXPECT_GT(out.workpiece->faceCount(), facesBefore);

    // Volume is slightly less than the original (rounded corners remove
    // material).  For r=1 mm fillet on a 50×50 perimeter, removal is small.
    const double volAfter = volumeOf(out.workpiece->shape());
    EXPECT_LT(volAfter, volBefore);
    EXPECT_GT(volAfter, volBefore * 0.95);

    EXPECT_EQ(out.signature.skill_id, std::string("fillet_edge"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. DFM-004 catches sub-0.2 mm radius ────────────────────────────────
TEST(SkillFilletEdge, ValidateRejectsSubMinRadius)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::fillet_edge::Input in;
    in.edge_selector = skill::fillet_edge::EdgesAtZBand{ 10.0, 1e-3 };
    in.radius_mm     = 0.1;   // < DFM-004 minimum

    auto r = skill::fillet_edge::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-004" && f.severity == "error") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::fillet_edge::apply(*stock, in), skill::SkillError);
}

// ─── 3. Recognize: finds the fillet rim ───────────────────────────────────
TEST(SkillFilletEdge, RecognizeFindsFilletedRim)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::fillet_edge::Input in;
    in.edge_selector = skill::fillet_edge::EdgesAtZBand{ 10.0, 1e-3 };
    in.radius_mm     = 1.5;

    auto out = skill::fillet_edge::apply(*stock, in);
    auto candidates = skill::fillet_edge::recognize(*out.workpiece);

    ASSERT_FALSE(candidates.empty());
    // There should be at least one candidate whose recovered radius matches.
    bool radiusFound = false;
    for (const auto& c : candidates) {
        const double recR = c.recovered_params["radius_mm"].get<double>();
        if (std::abs(recR - 1.5) < 1e-2) { radiusFound = true; break; }
    }
    EXPECT_TRUE(radiusFound);
}

// ─── 4. Round-trip: re-synthesise from recovered radius ───────────────────
TEST(SkillFilletEdge, RoundTripRadiusRecoverable)
{
    auto stock1 = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::fillet_edge::Input in1;
    in1.edge_selector = skill::fillet_edge::EdgesAtZBand{ 10.0, 1e-3 };
    in1.radius_mm     = 2.0;
    auto out1 = skill::fillet_edge::apply(*stock1, in1);

    auto cands = skill::fillet_edge::recognize(*out1.workpiece);
    ASSERT_FALSE(cands.empty());

    // Pick the candidate with the highest confidence.
    auto best = std::max_element(cands.begin(), cands.end(),
        [](const auto& a, const auto& b) { return a.confidence < b.confidence; });
    const double recR = (*best).recovered_params["radius_mm"].get<double>();
    EXPECT_NEAR(recR, in1.radius_mm, 1e-2);

    // Re-synthesise from the recovered params.
    auto stock2 = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::fillet_edge::Input in2;
    in2.edge_selector = skill::fillet_edge::EdgesAtZBand{ 10.0, 1e-3 };
    in2.radius_mm     = recR;
    auto out2 = skill::fillet_edge::apply(*stock2, in2);

    EXPECT_EQ(out1.workpiece->faceCount(), out2.workpiece->faceCount());
    EXPECT_NEAR(volumeOf(out1.workpiece->shape()),
                volumeOf(out2.workpiece->shape()), 1e-3);
}

// ─── 5. Edge case: no edges match the selector ────────────────────────────
TEST(SkillFilletEdge, NoMatchingEdgesThrows)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::fillet_edge::Input in;
    // Z = 100 is far above any cuboid edge.
    in.edge_selector = skill::fillet_edge::EdgesAtZBand{ 100.0, 1e-3 };
    in.radius_mm     = 1.0;

    EXPECT_THROW(skill::fillet_edge::apply(*stock, in), skill::SkillError);
}

// ─── 6. SPLIT hole walls are not fillet blends (summed-wrap exclusion) ─────
// A slot crossing a drilled hole SPLITS its cylindrical wall into partial
// faces; foreign STEP kernels do the same.  Each partial face passes a naive
// per-face wrap gate, and several same-radius partials then cluster into a
// bogus "rim-fillet" (measured pre-fix: 4 faces → confidence 0.90) that
// subsumes the real holes in dedupe.  The recogniser must merge COAXIAL
// same-radius faces and gate on the SUMMED wrap.
TEST(SkillFilletEdge, SplitHoleWallsAreNotFilletBlends)
{
    // Two d6 holes crossed by one slot → four partial wall faces (r = 3).
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    std::shared_ptr<skill::Workpiece> wp = stock;
    for (double hx : { 20.0, 40.0 }) {
        skill::drill_hole::Input h;
        h.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        h.position_x_mm = hx;
        h.position_y_mm = 30.0;
        h.axis_dir      = gp_Dir(0, 0, -1);
        h.diameter_mm   = 6.0;
        h.depth_mm      = 8.0;
        wp = skill::drill_hole::apply(*wp, h).workpiece;
    }
    skill::mill_slot::Input s;
    s.entry_face  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    s.start_x_mm  = 12.0;  s.start_y_mm = 30.0;
    s.end_x_mm    = 48.0;  s.end_y_mm   = 30.0;
    s.width_mm    = 4.0;                       // narrower than the d6 holes
    s.depth_mm    = 4.0;                       // upper half of the walls only
    wp = skill::mill_slot::apply(*wp, s).workpiece;

    skill::Workpiece foreign(wp->shape());
    const auto cands = skill::fillet_edge::recognize(foreign);
    for (const auto& c : cands) {
        const double r = c.recovered_params.value("radius_mm", -1.0);
        EXPECT_FALSE(std::abs(r - 3.0) < 0.1)
            << "split hole-wall partials (r=3) must never cluster into a "
               "fillet blend (summed wrap identifies them as one wall)";
    }
}
