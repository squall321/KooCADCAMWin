// @lat: [[process/test-strategy#skill round-trip]]
//
// ream skill — precision enlarging of an existing drill_hole.
//
// Tests:
//   1. Apply enlarges the bore by the requested delta-radius.
//   2. DFM rejects out-of-range enlarge_by (too small / too large).
//   3. apply() throws when datum doesn't resolve to a cylinder.
//   4. recognize() returns LOW-confidence ream candidates that mirror the
//      bore (this skill cannot be uniquely recognized without metadata).
//   5. Composed workflow: drill_hole → ream produces the expected final radius.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_hole.hpp"
#include "skills/ream.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Cylinder.hxx>

#include <algorithm>
#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

// Return the cylindrical face with the largest radius (proxy for "the bore").
double largestCylinderRadius(const skill::Workpiece& wp)
{
    double best = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        best = std::max(best, surf.Cylinder().Radius());
    }
    return best;
}

// Helper: create a stock with a drilled hole at (25,25), dia=3, depth=6.
std::shared_ptr<skill::Workpiece> stockWithHole(double dia = 3.0, double depth = 6.0)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::drill_hole::Input din;
    din.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    din.position_x_mm = 25.0;
    din.position_y_mm = 25.0;
    din.diameter_mm   = dia;
    din.depth_mm      = depth;
    auto holed = skill::drill_hole::apply(*stock, din);
    return holed.workpiece;
}

}  // namespace

// ─── 1. Apply enlarges the bore ──────────────────────────────────────────
TEST(SkillReam, ApplyEnlargesExistingBore)
{
    auto wp = stockWithHole(3.0, 6.0);
    const double rBefore = largestCylinderRadius(*wp);
    EXPECT_NEAR(rBefore, 1.5, 1e-3);

    skill::ream::Input in;
    in.existing_hole_datum = skill::FaceCylinderByAxis{ gp_Ax1(gp_Pnt(25, 25, 5), gp_Dir(0, 0, -1)) };
    in.enlarge_by_mm = 0.10;

    auto out = skill::ream::apply(*wp, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double rAfter = largestCylinderRadius(*out.workpiece);
    EXPECT_NEAR(rAfter, rBefore + 0.10, 1e-3)
        << "ream should enlarge bore radius by enlarge_by_mm";

    // Chain contract: prior bore feature + ream = 2 (full history).
    EXPECT_EQ(out.workpiece->features().size(), 2u);
    EXPECT_EQ(out.workpiece->features().back().skill_id, std::string("ream"));
    EXPECT_EQ(out.signature.skill_id, std::string("ream"));

    // Removed volume = annulus area × extent (≈ depth).
    const double annulusArea = M_PI * ((rBefore + 0.10) * (rBefore + 0.10)
                                      - rBefore * rBefore);
    const double diff = volumeOf(wp->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, annulusArea * 6.0, annulusArea * 6.0 * 0.10);
}

// ─── 2. DFM rejects out-of-range enlarge_by ──────────────────────────────
TEST(SkillReam, ValidateRejectsBadEnlargeBy)
{
    auto wp = stockWithHole();

    // Too small
    {
        skill::ream::Input in;
        in.existing_hole_datum = skill::FaceCylinderByAxis{
            gp_Ax1(gp_Pnt(25, 25, 5), gp_Dir(0, 0, -1)) };
        in.enlarge_by_mm = 0.005;

        auto r = skill::ream::validate(*wp, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::ream::apply(*wp, in), skill::SkillError);
    }

    // Too large
    {
        skill::ream::Input in;
        in.existing_hole_datum = skill::FaceCylinderByAxis{
            gp_Ax1(gp_Pnt(25, 25, 5), gp_Dir(0, 0, -1)) };
        in.enlarge_by_mm = 0.50;

        auto r = skill::ream::validate(*wp, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::ream::apply(*wp, in), skill::SkillError);
    }

    // Zero / negative
    {
        skill::ream::Input in;
        in.existing_hole_datum = skill::FaceCylinderByAxis{
            gp_Ax1(gp_Pnt(25, 25, 5), gp_Dir(0, 0, -1)) };
        in.enlarge_by_mm = 0.0;

        auto r = skill::ream::validate(*wp, in);
        EXPECT_FALSE(r.passed);
    }
}

// ─── 3. apply() throws when datum fails to resolve ───────────────────────
TEST(SkillReam, ApplyThrowsOnNoCylinder)
{
    // Plain cuboid — no cylindrical face anywhere.
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::ream::Input in;
    in.existing_hole_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(25, 25, 5), gp_Dir(0, 0, -1)) };
    in.enlarge_by_mm = 0.10;

    auto r = skill::ream::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::ream::apply(*stock, in), skill::SkillError);
}

// ─── 4. Recognize returns low-confidence candidate ───────────────────────
TEST(SkillReam, RecognizeReturnsLowConfidence)
{
    auto wp = stockWithHole(3.0, 6.0);
    skill::ream::Input in;
    in.existing_hole_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(25, 25, 5), gp_Dir(0, 0, -1)) };
    in.enlarge_by_mm = 0.10;
    auto out = skill::ream::apply(*wp, in);

    auto cands = skill::ream::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty()) << "ream should at least emit a low-confidence candidate";

    // Confidence must be low (≤ 0.50) because ream looks like drill_hole.
    for (const auto& c : cands) {
        EXPECT_LE(c.confidence, 0.50)
            << "ream recognize() should NOT be high-confidence (looks like drill_hole)";
        EXPECT_EQ(c.skill_id, std::string("ream"));
    }
}

// ─── 5. Composed: drill_hole then ream gives final diameter ──────────────
TEST(SkillReam, ComposedWorkflowProducesFinalDiameter)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Step 1: drill 4.90 mm (under-size for a precision 5.00 mm bore).
    skill::drill_hole::Input din;
    din.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    din.position_x_mm = 25.0;
    din.position_y_mm = 25.0;
    din.diameter_mm   = 4.90;
    din.depth_mm      = 7.0;
    auto drilled = skill::drill_hole::apply(*stock, din);

    EXPECT_NEAR(largestCylinderRadius(*drilled.workpiece), 2.45, 1e-3);

    // Step 2: ream by 0.05 mm radial → final radius 2.50 mm (= 5.00 mm dia).
    skill::ream::Input rin;
    rin.existing_hole_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(25, 25, 5), gp_Dir(0, 0, -1)) };
    rin.enlarge_by_mm = 0.05;

    auto reamed = skill::ream::apply(*drilled.workpiece, rin);
    EXPECT_NEAR(largestCylinderRadius(*reamed.workpiece), 2.50, 1e-3);

    // Feature history contains BOTH steps (Workpiece chain contract, B2):
    // [drill_hole, ream] — apply() carries the full prior history forward.
    EXPECT_EQ(reamed.workpiece->features().size(), 2u)
        << "ream must carry the prior drill_hole feature per the chain "
           "contract enforced by finalizeOutput/check_feature_chain at "
           "the plan level";
}
