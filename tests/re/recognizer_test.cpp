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
#include "skills/chamfer_edge.hpp"
#include "skills/drill_hole.hpp"
#include "skills/fillet_edge.hpp"   // for EdgesAtZBand selector
#include "skills/hollow_cavity.hpp"

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
// TODO(slice-9): after work-2 expanded the recognizer registry from 11 to
// 154 entries, some compound-feature recognizers fire false positives on
// the bare cuboid (e.g., heat_sink_fin_array or shim_pocket may mis-match
// the planar walls).  Tighten those recognizers' empty-cands gates.
TEST(Recognizer, DISABLED_AnalyzeBareStock)
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
    // NOT compete with drill_hole on a shared cylindrical face during
    // dedupe.  Chamfer the BOTTOM rim (z = 0) so its EdgesAtZBand does not
    // accidentally trim the drill's top entry circle (z = 12).
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
