// @lat: [[engine/reverse-route#tests]]
//
// Layer 4 — Recognizer pipeline tests.
//
// Verifies:
//   1. Empty stock → no candidates.
//   2. drill_hole only → drill_hole survives dedupe (its high-confidence
//      cylindrical-face claim beats the competing pocket / bore claims).
//   3. drill_hole + chamfer_edge → at least one of each, classified into
//      hole vs. edge groups.
//   4. hollow_cavity + drill_hole + chamfer_edge → inferred ProcessPlan
//      orders the steps stock-removal → hole → edge.

#include <gtest/gtest.h>

#include "re/Recognizer.hpp"

#include "process/ProcessPlan.hpp"

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/annular_groove.hpp"
#include "skills/chamfer_edge.hpp"
#include "skills/crown_knurl.hpp"
#include "skills/drill_hole.hpp"
#include "skills/fillet_edge.hpp"   // for EdgesAtZBand selector
#include "skills/hollow_cavity.hpp"

#include "engine/primitives/Tools.hpp"
#include "engine/primitives/Cuts.hpp"

#include <gp_Ax2.hxx>

#include <cmath>
#include <map>
#include <string>

using namespace koocadcam;

namespace {

// Convenience: classify a skill_id into the same three groups the
// Recognizer uses, mirroring its prefix logic.  Tests use this to assert
// ordering without poking into Recognizer internals.
int groupOf(const std::string& sid)
{
    if (sid == "hollow_cavity" ||
        sid == "mill_open_pocket" || sid == "profile_milling" ||
        sid == "mill_rect_pocket" || sid == "mill_circular_pocket" ||
        sid == "mill_slot"        || sid == "mill_keyway"        ||
        sid == "dovetail_slot"    || sid == "T_slot")
    {
        return 0;
    }
    if (sid.rfind("drill", 0) == 0 || sid.rfind("bore", 0) == 0) return 1;
    if (sid == "counterbore" || sid == "countersink" ||
        sid == "spot_drill"  || sid == "ream"        ||
        sid == "pocket_with_corner_relief")
    {
        return 1;
    }
    if (sid == "chamfer_edge" || sid == "fillet_edge" || sid == "face_milling")
    {
        return 2;
    }
    return 3;
}

}  // namespace

// ─── 1. Bare stock → no candidates ───────────────────────────────────────
// Re-enabled in slice-9 after tightening the bare-stock false-positive
// gates in two compound recognizers (top_face_recess_with_walls and
// curved_lip_around_face).  Both now demand TWO DISTINCT upward planar Z
// levels (panel top + raised wall top); a pristine cuboid only has one
// (its single top face at zMax), so neither fires.  The other 152
// registered recognizers either need a cylindrical face, a tilted chamfer
// face, repeated parallel fins, or multiple distinct upward Zs — none of
// which exist on bare stock.
TEST(Recognizer, AnalyzeBareStock)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto cands = re::analyze(*stock);

    // A pristine cuboid has only 6 planar faces and 12 line edges — none of
    // the registered recognizers should fire on it.
    EXPECT_TRUE(cands.empty())
        << "bare stock should produce no recognized features; got "
        << cands.size();
}

// ─── 2. After drill_hole → finds drill_hole ──────────────────────────────
TEST(Recognizer, AnalyzeAfterDrillHole)
{
    // NOTE: drill is placed OFF-CENTER so that the hollow_cavity recognizer
    // does not mis-identify the drill bottom face as a centered hollow
    // cavity (its symmetric-gap heuristic gives a high-confidence false
    // positive for a centered cuboid drill).
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::drill_hole::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 8.0;
    in.position_y_mm = 8.0;
    in.diameter_mm   = 4.0;
    in.depth_mm      = 6.0;
    auto out = skill::drill_hole::apply(*stock, in);

    auto cands = re::analyzeFiltered(*out.workpiece, 0.7);
    ASSERT_FALSE(cands.empty()) << "expected at least one candidate";

    // After dedupe, only the cylindrical-face candidate (drill_hole, 0.95)
    // and any independent claims should remain.  The drill_hole identity
    // must be present — competing skills (mill_circular_pocket at 0.9 on
    // the same face) collapse into drill_hole during dedupe.
    auto dedup = re::dedupe(cands);
    bool foundDrill = false;
    for (const auto& c : dedup) {
        if (c.skill_id == "drill_hole") { foundDrill = true; break; }
    }
    EXPECT_TRUE(foundDrill) << "dedupe should preserve a drill_hole candidate";
    // The drill_hole candidate should be high-confidence.
    for (const auto& c : dedup) {
        if (c.skill_id == "drill_hole") {
            EXPECT_GT(c.confidence, 0.8);
            break;
        }
    }
}

// ─── 3. drill_hole + chamfer_edge → both groups represented ──────────────
TEST(Recognizer, AnalyzeAfterTwoFeatures)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::drill_hole::Input dh;
    dh.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    dh.position_x_mm = 8.0;     // off-center (see test 2 note)
    dh.position_y_mm = 8.0;
    dh.diameter_mm   = 4.0;
    dh.depth_mm      = 5.0;
    auto wp1 = skill::drill_hole::apply(*stock, dh);

    // Chamfer the BOTTOM rim (z = 0) so the drill's top entry circle
    // (z = 10) is not consumed by the chamfer cut.
    skill::chamfer_edge::Input ch;
    ch.edge_selector   = skill::fillet_edge::EdgesAtZBand{ 0.0, 1e-3 };
    ch.chamfer_size_mm = 1.0;
    ch.angle_deg       = 45.0;
    auto wp2 = skill::chamfer_edge::apply(*wp1.workpiece, ch);

    auto cands = re::analyzeFiltered(*wp2.workpiece, 0.7);
    auto dedup = re::dedupe(cands);

    bool hasHole = false, hasEdge = false;
    for (const auto& c : dedup) {
        const int g = groupOf(c.skill_id);
        if (g == 1) hasHole = true;
        if (g == 2) hasEdge = true;
    }
    EXPECT_TRUE(hasHole) << "expected a hole-group candidate";
    EXPECT_TRUE(hasEdge) << "expected an edge-group candidate";
}

// ─── 4. Full plan inference → group ordering preserved ───────────────────
TEST(Recognizer, InferPlanSequence)
{
    // Build a workpiece with one of each group.  We deliberately use
    // hollow_cavity (not mill_circular_pocket) for the stock-removal stage
    // because its rectangular cavity creates only planar walls, so it does
    // NOT compete with drill_hole on a shared cylindrical face during dedupe.
    // hollow_cavity is recognised here via metadata replay (the signature is
    // in the synthetic chain history), so it stays above the 0.7 inference
    // threshold despite the foreign-CAD geometric-fallback cap.  Chamfer the
    // BOTTOM rim (z = 0) so its EdgesAtZBand does not accidentally trim the
    // drill's top entry circle (z = 12).
    //   A_stock:  hollow_cavity (rectangular cavity)
    //   B_hole:   drill_hole    (in the rim, away from the cavity)
    //   C_edge:   chamfer_edge  (bottom outer rim)
    auto stock = skill::createCuboidStock(60.0, 60.0, 12.0);

    skill::hollow_cavity::Input hc;
    hc.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    hc.wall_thickness_mm = 22.0;    // cavity ≈ 16×16 mm centered, 22 mm rim
    hc.depth_mm          = 5.0;
    auto wp1 = skill::hollow_cavity::apply(*stock, hc);

    // Drill in the outer rim (x = 6 is inside the 22 mm-wide rim).
    skill::drill_hole::Input dh;
    dh.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    dh.position_x_mm = 6.0;
    dh.position_y_mm = 30.0;
    dh.diameter_mm   = 3.0;
    dh.depth_mm      = 5.0;
    auto wp2 = skill::drill_hole::apply(*wp1.workpiece, dh);

    skill::chamfer_edge::Input ch;
    ch.edge_selector   = skill::fillet_edge::EdgesAtZBand{ 0.0, 1e-3 };
    ch.chamfer_size_mm = 1.0;
    ch.angle_deg       = 45.0;
    auto wp3 = skill::chamfer_edge::apply(*wp2.workpiece, ch);

    auto plan = re::inferProcessPlan(*wp3.workpiece, 0.7);
    ASSERT_FALSE(plan.empty()) << "plan should not be empty";

    // Walk the plan and find the FIRST index of each group.  Group A
    // (stock removal) must appear before group B (holes), which must
    // appear before group C (edges).
    int firstA = -1, firstB = -1, firstC = -1;
    const auto& steps = plan.steps();
    for (int i = 0; i < static_cast<int>(steps.size()); ++i) {
        const int g = groupOf(steps[static_cast<size_t>(i)].skill_id);
        if (g == 0 && firstA < 0) firstA = i;
        if (g == 1 && firstB < 0) firstB = i;
        if (g == 2 && firstC < 0) firstC = i;
    }

    // Each group must be represented.
    EXPECT_GE(firstA, 0) << "stock-removal step missing from plan";
    EXPECT_GE(firstB, 0) << "hole step missing from plan";
    EXPECT_GE(firstC, 0) << "edge step missing from plan";

    // And ordered A < B < C.
    if (firstA >= 0 && firstB >= 0) EXPECT_LT(firstA, firstB);
    if (firstB >= 0 && firstC >= 0) EXPECT_LT(firstB, firstC);
}

// ─── liftRecoveredStep: edge-op params become replay-ready ────────────────
TEST(Recognizer, LiftRecoveredStepRaisesEdgeSelector)
{
    using namespace koocadcam;
    process::StepInvocation step;
    step.skill_id = "chamfer_edge";
    step.params = {
        { "edge_selector", { { "z_mm", 7.5 }, { "tolerance_mm", 0.01 } } },
        { "chamfer_size_mm", 0.05 },   // below the 0.1 mm floor
        { "angle_deg", 45.0 },
    };
    const auto s = re::liftRecoveredStep(step);
    EXPECT_NEAR(s.params.value("edges_at_z_mm", -1.0), 7.5, 1e-9);
    EXPECT_NEAR(s.params.value("tolerance_mm", -1.0), 0.01, 1e-9);
    EXPECT_NEAR(s.params.value("chamfer_size_mm", 0.0), 0.1, 1e-9);   // clamped up

    // Idempotent.
    const auto s2 = re::liftRecoveredStep(s);
    EXPECT_EQ(s2.params, s.params);
}

TEST(Recognizer, LiftRecoveredStepPreservesMeasuredChamfer)
{
    using namespace koocadcam;
    // A measured chamfer is trustworthy across the recognizer's band — it must
    // NOT be shrunk.  Only a sub-0.1 mm value is floored to the DFM minimum.
    process::StepInvocation big;
    big.skill_id = "chamfer_edge";
    big.params = { { "chamfer_size_mm", 3.0 } };
    EXPECT_NEAR(re::liftRecoveredStep(big).params.value("chamfer_size_mm", 0.0),
                3.0, 1e-9) << "a measured 3 mm chamfer must survive the lift";

    process::StepInvocation tiny;
    tiny.skill_id = "chamfer_edge";
    tiny.params = { { "chamfer_size_mm", 0.04 } };
    EXPECT_NEAR(re::liftRecoveredStep(tiny).params.value("chamfer_size_mm", 0.0),
                0.1, 1e-9) << "a sub-0.1 mm chamfer is floored to the DFM minimum";
}

TEST(Recognizer, LiftRecoveredStepPassesThroughNonEdge)
{
    using namespace koocadcam;
    process::StepInvocation step;
    step.skill_id = "drill_hole";
    step.params = { { "diameter_mm", 8.0 }, { "edge_selector", { { "z_mm", 1.0 } } } };
    const auto s = re::liftRecoveredStep(step);
    EXPECT_EQ(s.params, step.params);   // untouched
    EXPECT_FALSE(s.params.contains("edges_at_z_mm"));
}

// ─── VERIFIER: crown_knurl duplicate-drill / double-cut path (finding #2) ────
// A crown_knurl emits NO face-id key and is NOT in the hole-pattern subsumption
// set, so if its notch cylinders ALSO pass drill_hole's ring+wrap gates, the
// inferred plan carries BOTH a crown_knurl step AND N drill_hole steps for the
// same notches — re-execution double-cuts them.  Build a knurl with LARGE, DEEP
// notches (big enough to present >=2 rings and a >81 deg wall arc) to probe
// whether the double-cut path is reachable through the FULL inferProcessPlan.
TEST(Recognizer, CrownKnurlNotchesDoNotSurviveAsDuplicateDrills)
{
    namespace pr = koocadcam::engine::prim;
    auto stock = skill::createCuboidStock(40.0, 40.0, 14.0);
    // A larger cone knob (radius 6) at the top centre.
    const gp_Ax2 knobAx(gp_Pnt(20, 20, 14), gp_Dir(0, 0, 1));
    TopoDS_Shape s = pr::fuse(stock->shape(),
        pr::coneFrustum(knobAx, 6.0, 6.0 * 0.85, 4.0));
    // 6 LARGE, DEEP radial notches (r=1.2, length 5 -> penetrate well past the
    // rim, presenting a full cylindrical wall + entry + far ring).
    for (int i = 0; i < 6; ++i) {
        const double phi = 2.0 * M_PI * i / 6;
        const gp_Pnt c(20 + std::cos(phi) * 6.0, 20 + std::sin(phi) * 6.0, 16.0);
        const gp_Dir inDir(-std::cos(phi), -std::sin(phi), 0);
        s = pr::cut(s, pr::cylinder(gp_Ax2(c, inDir), 1.2, 5.0));
    }
    skill::Workpiece foreign(s);

    // Raw recognizer view: does drill_hole independently claim the notches?
    const auto drills = skill::drill_hole::recognize(foreign);
    std::printf("[KnurlDup] raw drill_hole candidates on notch ring = %zu\n",
                drills.size());

    // Full pipeline: what steps does the recovered plan carry?
    auto plan = re::inferProcessPlan(foreign, 0.7);
    std::map<std::string, int> byKind;
    for (const auto& st : plan.steps()) byKind[st.skill_id]++;
    std::printf("[KnurlDup] plan steps:");
    for (const auto& [k, n] : byKind) std::printf(" %s x%d", k.c_str(), n);
    std::printf("\n");

    const int knurl = byKind["crown_knurl"];
    const int drill = byKind["drill_hole"] + byKind["drill_through_hole"];
    // If BOTH a knurl AND per-notch drills survive, the plan double-cuts on
    // regeneration — the finding-#2 failure mode.  This EXPECT documents the
    // actual reachability of that path.
    EXPECT_FALSE(knurl >= 1 && drill >= 1)
        << "DOUBLE-CUT PATH REACHABLE: plan carries crown_knurl x" << knurl
        << " AND drill_hole x" << drill
        << " for the same notch ring (no subsumption, no face-id dedupe)";
}

namespace {

// Two annular grooves on one plate, machined at face-local offsets so their
// WORLD centres differ; dims per ring supplied by the caller.
skill::Workpiece twoGroovePlate(double depthLeft, double depthRight)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);
    skill::annular_groove::Input g;
    g.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    g.outer_dia_mm = 10.0;
    g.inner_dia_mm =  8.0;
    g.center_x_mm  = -20.0;
    g.depth_mm     = depthLeft;
    const auto left = skill::annular_groove::apply(*stock, g);
    g.center_x_mm  = 20.0;
    g.depth_mm     = depthRight;
    const auto both = skill::annular_groove::apply(*left.workpiece, g);
    return skill::Workpiece(both.workpiece->shape());   // NO history
}

int annularStepsInPlan(const skill::Workpiece& foreign)
{
    // NAMED plan: ranging over `inferProcessPlan(...).steps()` directly would
    // iterate a reference into a destroyed temporary (C++17 range-for does
    // not extend the temporary's lifetime through a member call).
    const process::ProcessPlan plan = re::inferProcessPlan(foreign, 0.7);
    int n = 0;
    for (const auto& st : plan.steps())
        if (st.skill_id == "annular_groove") ++n;
    return n;
}

}  // namespace

// ─── Multi-emit must SURVIVE dedupe: twin same-dimension grooves ────────────
// The default phone carries two IDENTICAL camera deco rings at different
// world positions.  The geometric path stamps center (0,0) ("on its own
// face"), so without world_center in the fingerprint the twins collide and
// one is silently dropped in inferProcessPlan — multi-emit nullified one
// stage later.
TEST(Recognizer, TwinSameDimensionGroovesBothSurviveDedupe)
{
    const skill::Workpiece foreign = twoGroovePlate(0.5, 0.5);
    EXPECT_EQ(annularStepsInPlan(foreign), 2)
        << "two dimensionally identical grooves at different WORLD centres "
           "are two features — the fingerprint must not collapse them";
}

// ─── Same-OD, different-depth grooves: the outer-wall stamp must be COAXIAL ─
// Their fingerprints differ (depth is a key), so the binding failure is the
// face-set: a bare radius match let each floor grab the OTHER ring's outer
// wall (last match wins), the shared face id made the two candidates
// "overlap", and dedupe blocked the second at equal confidence.
TEST(Recognizer, SameODDifferentDepthGroovesBothSurviveDedupe)
{
    const skill::Workpiece foreign = twoGroovePlate(0.5, 1.2);
    EXPECT_EQ(annularStepsInPlan(foreign), 2)
        << "same-OD grooves at different centres must not contaminate each "
           "other's outer-wall face id";
}
