// @lat: [[engine/skills#Layer 5 LLM adapter]]
//
// FeatureTransfer — cross-product transfer unit spec, two placement families:
//
//   1–12  FACE-LOCAL annular_groove — classify table, positional scaling,
//         intrinsic preservation, ratio-preserving OD/ID clamp, refusals
//         (unsupported skill / non-±Z / too-thin / off-envelope / thin wall),
//         and FACE-ANCHORED placement (offset deck clamp, recessed-deck
//         depth limit).
//   13–22 WORLD-XY pitch-circle patterns (bolt_circle / counterbore_ring /
//         countersink_ring) — frame-RELATIVE world re-expression (13),
//         PCD-only fit clamp with fastener dias preserved (14), member-merge
//         refusal (15), seat/cone-preserving depth clamp with refusal when
//         the stock cannot hold the fastener intrinsic (16, 17), the
//         through-drill depth-clamp exemption (18), non-vertical/missing
//         axis refusal (19), the face-anchored recessed-deck depth limit
//         (20), the validate/composed-atom gate mirror sweep (21), and the
//         per-axis real-eccentricity clamp discrimination (22).

#include <gtest/gtest.h>

#include "adapt/FeatureTransfer.hpp"

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/mill_rect_pocket.hpp"

#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>

#include <string>

using namespace koocadcam;
using nlohmann::json;

namespace {

// A hand-authored destination/source frame (half-extents only — slice-1
// transfer scales against half-extents; centres are unused).
adapt::AnchorFrame frame(double hx, double hy, double hz)
{
    adapt::AnchorFrame f;
    f.hx = hx;
    f.hy = hy;
    f.hz = hz;
    return f;
}

// The canonical recovered bezel-ring step used across the tests: intrinsics
// of the default watch bezel plus the product-bound breadcrumbs a recovered
// step drags along (entry_face_id from metadata replay, world_center from
// the geometric recognizer).
process::StepInvocation makeGrooveStep()
{
    process::StepInvocation s;
    s.skill_id = "annular_groove";
    s.params = {
        { "center_x_mm",    5.0 },
        { "center_y_mm",   -3.0 },
        { "outer_dia_mm",  44.0 },
        { "inner_dia_mm",  38.0 },
        { "depth_mm",       1.0 },
        { "taper_deg",      0.0 },
        { "entry_face_id",  7 },
        { "world_center",   json::array({ 1.0, 2.0 }) },
    };
    s.depends_on = { 0 };
    return s;
}

}  // namespace

// ── 1. ClassifyParamTable ────────────────────────────────────────────────
TEST(FeatureTransfer, ClassifyParamTable)
{
    using adapt::ParamRole;
    using adapt::classifyParam;

    // Positional — the (position_|center_|offset_)(x|y|z)_mm family (the
    // same set parts/DatumGraph.cpp scans) plus its friends.
    EXPECT_EQ(classifyParam("position_x_mm"), ParamRole::Positional);
    EXPECT_EQ(classifyParam("center_y_mm"),   ParamRole::Positional);
    EXPECT_EQ(classifyParam("offset_z_mm"),   ParamRole::Positional);
    EXPECT_EQ(classifyParam("start_x_mm"),    ParamRole::Positional);
    EXPECT_EQ(classifyParam("origin_x_mm"),   ParamRole::Positional);
    EXPECT_EQ(classifyParam("origin_y_mm"),   ParamRole::Positional);
    EXPECT_EQ(classifyParam("world_oy_mm"),   ParamRole::Positional);
    EXPECT_EQ(classifyParam("edges_at_z_mm"), ParamRole::Positional);

    // Datum — face/axis selectors.
    EXPECT_EQ(classifyParam("entry_face"),    ParamRole::Datum);
    EXPECT_EQ(classifyParam("entry_face_id"), ParamRole::Datum);
    EXPECT_EQ(classifyParam("face_normal"),   ParamRole::Datum);
    EXPECT_EQ(classifyParam("axis_dir"),      ParamRole::Datum);

    // Intrinsic — the feature's own dimensions/counts.
    EXPECT_EQ(classifyParam("outer_dia_mm"),   ParamRole::Intrinsic);
    EXPECT_EQ(classifyParam("seat_depth_mm"),  ParamRole::Intrinsic);
    EXPECT_EQ(classifyParam("depth_mm"),       ParamRole::Intrinsic);
    EXPECT_EQ(classifyParam("taper_deg"),      ParamRole::Intrinsic);
    EXPECT_EQ(classifyParam("chamfer_angle_deg"), ParamRole::Intrinsic);
    EXPECT_EQ(classifyParam("length_mm"),      ParamRole::Intrinsic);
    EXPECT_EQ(classifyParam("hole_count"),     ParamRole::Intrinsic);
    EXPECT_EQ(classifyParam("row_spacing_mm"), ParamRole::Intrinsic);
    EXPECT_EQ(classifyParam("through_hole"),   ParamRole::Intrinsic);

    // Diagnostic — recognizer breadcrumbs.
    EXPECT_EQ(classifyParam("world_center"),  ParamRole::Diagnostic);
    EXPECT_EQ(classifyParam("hole_centers"),  ParamRole::Diagnostic);
    EXPECT_EQ(classifyParam("cyl_face_ids"),  ParamRole::Diagnostic);

    // Unrecognised keys default to Intrinsic (leave untouched).
    EXPECT_EQ(classifyParam("mystery_key"),   ParamRole::Intrinsic);
}

// ── 2. AnnularGrooveTransfers ────────────────────────────────────────────
TEST(FeatureTransfer, AnnularGrooveScalesPositionsPreservesIntrinsics)
{
    const auto src = makeGrooveStep();
    const auto res = adapt::transferFeature(src, "watch", "phone",
                                            frame(22.0, 22.0, 5.0),
                                            frame(38.0, 80.0, 4.0));
    ASSERT_TRUE(res.transferred);
    EXPECT_FALSE(res.fit_clamped);
    EXPECT_EQ(res.step.skill_id, "annular_groove");

    const json& p = res.step.params;

    // Positions re-expressed in the destination frame (per-axis ratio).
    EXPECT_NEAR(p["center_x_mm"].get<double>(),  5.0 * 38.0 / 22.0, 1e-9);
    EXPECT_NEAR(p["center_y_mm"].get<double>(), -3.0 * 80.0 / 22.0, 1e-9);

    // Intrinsics preserved bit-for-bit — the phone is not a scaled watch.
    EXPECT_DOUBLE_EQ(p["outer_dia_mm"].get<double>(), 44.0);
    EXPECT_DOUBLE_EQ(p["inner_dia_mm"].get<double>(), 38.0);
    EXPECT_DOUBLE_EQ(p["depth_mm"].get<double>(),      1.0);
    EXPECT_DOUBLE_EQ(p["taper_deg"].get<double>(),     0.0);

    // Product-bound keys stripped; portable front-face datum injected.
    EXPECT_FALSE(p.contains("entry_face_id"));
    EXPECT_FALSE(p.contains("world_center"));
    ASSERT_TRUE(p.contains("face_normal"));
    EXPECT_EQ(p["face_normal"], json::array({ 0.0, 0.0, 1.0 }));

    // Cross-product: source-plan dependencies are meaningless here.
    EXPECT_TRUE(res.step.depends_on.empty());
}

// ── 3. FitClampRatioPreserving ───────────────────────────────────────────
TEST(FeatureTransfer, FitClampScalesODAndIDRatioPreserving)
{
    process::StepInvocation s;
    s.skill_id = "annular_groove";
    s.params = {
        { "center_x_mm",   0.0 },
        { "center_y_mm",   0.0 },
        { "outer_dia_mm", 80.0 },
        { "inner_dia_mm", 38.0 },
        { "depth_mm",      1.0 },
    };
    const auto res = adapt::transferFeature(s, "watch", "phone",
                                            frame(50.0, 50.0, 5.0),
                                            frame(38.0, 80.0, 4.0));
    ASSERT_TRUE(res.transferred);
    EXPECT_TRUE(res.fit_clamped);

    // factor = max feasible OD / OD = (2 * (38 - margin 2)) / 80 = 0.9;
    // ID scales by the SAME factor (ratio-preserving).
    EXPECT_NEAR(res.step.params["outer_dia_mm"].get<double>(), 72.0, 1e-9);
    EXPECT_NEAR(res.step.params["inner_dia_mm"].get<double>(), 34.2, 1e-9);
}

// ── 4. ImpossibleFitRefuses ──────────────────────────────────────────────
TEST(FeatureTransfer, ImpossibleFitRefusesInsteadOfLying)
{
    process::StepInvocation s;
    s.skill_id = "annular_groove";
    s.params = {
        { "center_x_mm",   0.0 },
        { "center_y_mm",   0.0 },
        { "outer_dia_mm", 80.0 },
        { "inner_dia_mm", 79.5 },
        { "depth_mm",      1.0 },
    };
    const auto res = adapt::transferFeature(s, "watch", "phone",
                                            frame(50.0, 50.0, 5.0),
                                            frame(38.0, 80.0, 4.0));
    // Post-clamp wall = (72 - 71.55) / 2 = 0.225 mm < 0.5 mm — no machinable
    // groove remains, so the transfer must REFUSE, not emit a lying step.
    EXPECT_FALSE(res.transferred);
}

// ── 5. UnsupportedSkillRefuses ───────────────────────────────────────────
TEST(FeatureTransfer, UnsupportedSkillRefusesWithNote)
{
    process::StepInvocation s;
    s.skill_id = "drill_hole";
    s.params = { { "position_x_mm", 1.0 }, { "diameter_mm", 3.0 } };

    const auto res = adapt::transferFeature(s, "watch", "phone",
                                            frame(22.0, 22.0, 5.0),
                                            frame(38.0, 80.0, 4.0));
    EXPECT_FALSE(res.transferred);

    bool mentionsUnsupported = false;
    for (const auto& n : res.notes)
        if (n.find("unsupported") != std::string::npos) mentionsUnsupported = true;
    EXPECT_TRUE(mentionsUnsupported)
        << "the refusal must say WHY (unsupported skill for cross-product transfer)";
}

// ── 6. Non-front entry refuses ───────────────────────────────────────────
// The fit clamp maps depth→Z and footprint→X/Y, which is only sound for a ±Z
// entry plane; a surviving side-entry normal must refuse, never mis-clamp.
TEST(FeatureTransfer, NonFrontFaceNormalRefuses)
{
    auto s = makeGrooveStep();
    s.params["face_normal"] = json::array({ 1.0, 0.0, 0.0 });   // side entry
    const auto res = adapt::transferFeature(s, "watch", "phone",
                                            frame(22.0, 22.0, 5.0),
                                            frame(38.0, 80.0, 4.0));
    EXPECT_FALSE(res.transferred);
    EXPECT_EQ(res.step.skill_id, "refused_transfer") << "refusals are neutered";
}

// ── 7. Too-thin destination refuses ──────────────────────────────────────
// A destination with < ~1 mm of stock cannot hold ANY groove + floor; the old
// code stamped a NEGATIVE depth with transferred=true (contract violation).
TEST(FeatureTransfer, TooThinDestinationRefusesInsteadOfNegativeDepth)
{
    const auto src = makeGrooveStep();
    const auto res = adapt::transferFeature(src, "watch", "phone",
                                            frame(22.0, 22.0, 5.0),
                                            frame(38.0, 80.0, 0.4));   // 0.8 mm slab
    EXPECT_FALSE(res.transferred);
    // Never a nonsense depth on the returned step.
    if (res.step.params.contains("depth_mm") &&
        res.step.params["depth_mm"].is_number())
        EXPECT_GT(res.step.params["depth_mm"].get<double>(), 0.0);
}

// ── 8. Depth clamp branch ────────────────────────────────────────────────
// A groove deeper than the destination thickness clamps to thickness minus
// 1 mm of floor and reports fit_clamped (previously untested branch).
TEST(FeatureTransfer, DepthClampsToDestinationThickness)
{
    auto s = makeGrooveStep();
    s.params["depth_mm"] = 9.0;                       // deeper than the 8 mm slab
    const auto res = adapt::transferFeature(s, "watch", "phone",
                                            frame(22.0, 22.0, 5.0),
                                            frame(38.0, 80.0, 4.0));   // 8 mm thick
    ASSERT_TRUE(res.transferred);
    EXPECT_TRUE(res.fit_clamped);
    EXPECT_NEAR(res.step.params["depth_mm"].get<double>(), 7.0, 1e-9)
        << "clamped to 2*hz - 1 mm floor";
}

// ── 9. Off-centre beyond the concentric envelope refuses ─────────────────
// The executed position is FACE-LOCAL; the frame-ratio math is only faithful
// near the frame centre.  Past half the destination half-extent → refuse.
TEST(FeatureTransfer, OffCentreBeyondEnvelopeRefuses)
{
    auto s = makeGrooveStep();
    s.params["center_x_mm"] = 15.0;   // scales to 15*38/22 = 25.9 > 0.5*38 = 19
    const auto res = adapt::transferFeature(s, "watch", "phone",
                                            frame(22.0, 22.0, 5.0),
                                            frame(38.0, 80.0, 4.0));
    EXPECT_FALSE(res.transferred);
    EXPECT_EQ(res.step.skill_id, "refused_transfer");
}

// ── 10. A pre-existing unmachinable ring refuses even when it FITS ───────
// The 0.5 mm groove-wall floor applies always, not only inside the clamp
// branch — transferring a DFM-invalid feature would be a lie either way.
TEST(FeatureTransfer, UnmachinableSourceWallRefusesWithoutClamp)
{
    auto s = makeGrooveStep();
    s.params["outer_dia_mm"] = 20.0;
    s.params["inner_dia_mm"] = 19.4;                  // wall 0.3 mm, fits easily
    const auto res = adapt::transferFeature(s, "watch", "phone",
                                            frame(22.0, 22.0, 5.0),
                                            frame(38.0, 80.0, 4.0));
    EXPECT_FALSE(res.transferred);
    EXPECT_FALSE(res.fit_clamped) << "refused before any clamp was needed";
}

// ─── 11. FACE-ANCHORED placement: an OFF-CENTRE largest entry face ─────────
// A stepped plate whose largest +Z face sits left of the frame centre: the
// executed ring lands on THAT face's centre, so the radial clamp must use the
// real eccentricity (frame-only math would see |cx|=0 and never clamp).
TEST(FeatureTransfer, FaceAnchoredOffsetDeckClampsExactly)
{
    // 80x80x10 plate; step the right 30 mm down by 3 → largest +Z face is the
    // LEFT ~50x80 deck at z=10, centre x ≈ 25.5 (the pocket's corner fillets
    // nudge the boundary); frame centre (40, 40, 5).  The plate is WIDE in Y
    // so only the X eccentricity binds the clamp.
    auto stock = skill::createCuboidStock(80.0, 80.0, 10.0);
    skill::mill_rect_pocket::Input mp;
    mp.entry_face  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    mp.center_x_mm = 65.0;
    mp.center_y_mm = 40.0;
    mp.length_mm   = 30.0;
    mp.width_mm    = 80.0;
    mp.depth_mm    = 3.0;
    auto stepped = skill::mill_rect_pocket::apply(*stock, mp);
    ASSERT_NE(stepped.workpiece, nullptr);
    const TopoDS_Shape dst = stepped.workpiece->shape();

    process::StepInvocation s;
    s.skill_id = "annular_groove";
    s.params = {
        { "center_x_mm",   0.0 },
        { "center_y_mm",   0.0 },
        { "outer_dia_mm", 48.0 },     // ecc ~14.5 + 24 = 38.5 > 40 - margin 2
        { "inner_dia_mm", 40.0 },
        { "depth_mm",      1.0 },
    };
    adapt::TransferOptions opts;
    opts.dst_shape = &dst;
    const auto res = adapt::transferFeature(
        s, "watch", "phone",
        adapt::AnchorFrame{ 0, 0, 0, 22, 22, 5 },
        adapt::AnchorFrame::fromShape(dst), opts);
    ASSERT_TRUE(res.transferred);
    EXPECT_TRUE(res.fit_clamped)
        << "the face-anchored eccentricity (~14.5) forces a clamp that the "
           "centred-face assumption (|cx|=0 -> 24 <= 38) would miss";
    // The MEASURED footprint binds at the pocket wall (x = 50), not the plate
    // edge: a ring reaching over the recessed pocket would cut air there and
    // execute as an incomplete arc — not the transferred feature.  feasible
    // OD = (pocket wall 50 - exec x ~25.5 - margin 2) * 2 ≈ 45 (±corner-
    // fillet wobble on the resolved face centre).
    const double odClamped = res.step.params["outer_dia_mm"].get<double>();
    EXPECT_GT(odClamped, 43.5);
    EXPECT_LT(odClamped, 46.5) << "clamped strictly below the source OD 48";
}

// ─── 12. FACE-ANCHORED depth: a RECESSED largest entry face limits depth by
// the stock actually below it, not the whole bbox thickness. ────────────────
TEST(FeatureTransfer, FaceAnchoredRecessedDeckLimitsDepth)
{
    // Recess most of the top by 3 → the largest +Z face is the pocket FLOOR at
    // z = 7; only 7 mm of stock sits below it (bbox thickness is 10).
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);
    skill::mill_rect_pocket::Input mp;
    mp.entry_face  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    mp.center_x_mm = 50.0;
    mp.center_y_mm = 20.0;
    mp.length_mm   = 60.0;                    // floor 60x40 > remaining 20x40 top
    mp.width_mm    = 40.0;
    mp.depth_mm    = 3.0;
    auto stepped = skill::mill_rect_pocket::apply(*stock, mp);
    ASSERT_NE(stepped.workpiece, nullptr);
    const TopoDS_Shape dst = stepped.workpiece->shape();

    process::StepInvocation s;
    s.skill_id = "annular_groove";
    s.params = {
        { "center_x_mm",   0.0 },
        { "center_y_mm",   0.0 },
        { "outer_dia_mm", 20.0 },
        { "inner_dia_mm", 14.0 },
        { "depth_mm",      8.0 },             // > 7 - 1 = 6 mm of usable stock
    };
    adapt::TransferOptions opts;
    opts.dst_shape = &dst;
    const auto res = adapt::transferFeature(
        s, "watch", "phone",
        adapt::AnchorFrame{ 0, 0, 0, 22, 22, 5 },
        adapt::AnchorFrame::fromShape(dst), opts);
    ASSERT_TRUE(res.transferred);
    EXPECT_TRUE(res.fit_clamped);
    EXPECT_NEAR(res.step.params["depth_mm"].get<double>(), 6.0, 1e-6)
        << "depth limit = faceZ(7) - bbox bottom(0) - 1 mm floor = 6, NOT the "
           "bbox-thickness limit (2*5 - 1 = 9) that would sever the recessed deck";
}

// ═══ WORLD-XY pattern family (bolt_circle / counterbore_ring /
//     countersink_ring) — positions are world coordinates, so they re-express
//     RELATIVE to the frame centres; the pitch circle alone absorbs the fit
//     clamp; fastener-critical member dimensions refuse rather than shrink. ═══

namespace {

// The canonical recovered counterbore-ring step: measured intrinsics plus the
// breadcrumbs the grammar drags along (entry_face_id, hole_centers).
process::StepInvocation makeCounterboreRingStep()
{
    process::StepInvocation s;
    s.skill_id = "counterbore_ring_pattern";
    s.params = {
        { "count",              6 },
        { "bolt_circle_dia_mm", 40.0 },
        { "center_x_mm",        0.0 },
        { "center_y_mm",        0.0 },
        { "position_z_mm",      10.0 },
        { "axis_dir",           json::array({ 0.0, 0.0, -1.0 }) },
        { "pilot_dia_mm",       3.0 },
        { "pilot_depth_mm",     6.0 },
        { "seat_dia_mm",        6.0 },
        { "seat_depth_mm",      2.0 },
        { "start_angle_deg",    15.0 },
        { "hole_centers",       json::array({ json::array({ 20.0, 0.0, 8.0 }) }) },
        { "entry_face_id",      9 },
    };
    return s;
}

}  // namespace

// ─── 13. Pattern positions are WORLD coordinates: frame-RELATIVE scaling ───
// A source product NOT modelled about the world origin discriminates: a bare
// ratio would fling the centre to 34*(40/20) = 68; the world-relative math
// lands it at dstC + (34-30)*(40/20) = 8.
TEST(FeatureTransfer, BoltCircleWorldCentreReExpressesRelativeToFrames)
{
    process::StepInvocation s;
    s.skill_id = "bolt_circle_pattern";
    s.params = {
        { "hole_count",         6 },
        { "bolt_circle_dia_mm", 20.0 },
        { "hole_dia_mm",        3.0 },
        { "center_x_mm",        34.0 },
        { "center_y_mm",        10.0 },
        { "start_angle_deg",    12.0 },
        { "axis_dir",           json::array({ 0.0, 0.0, -1.0 }) },
        { "position_z_mm",      10.0 },
        { "hole_centers",       json::array({ json::array({ 44.0, 10.0, 10.0 }) }) },
        { "entry_face_id",      4 },
    };
    s.depends_on = { 2 };

    const auto res = adapt::transferFeature(
        s, "watch", "phone",
        adapt::AnchorFrame{ 30.0, 10.0, 5.0, 20.0, 20.0, 5.0 },
        adapt::AnchorFrame{  0.0,  0.0, 0.0, 40.0, 40.0, 5.0 });
    ASSERT_TRUE(res.transferred);
    EXPECT_FALSE(res.fit_clamped);

    const json& p = res.step.params;
    EXPECT_NEAR(p["center_x_mm"].get<double>(), 8.0, 1e-9)
        << "world-relative: 0 + (34-30)*(40/20), NOT the bare ratio 34*2 = 68";
    EXPECT_NEAR(p["center_y_mm"].get<double>(), 0.0, 1e-9);

    // Intrinsics preserved bit-for-bit; the members are re-derived by apply()
    // from centre + PCD + count + phase, so the breadcrumbs must be GONE.
    EXPECT_DOUBLE_EQ(p["bolt_circle_dia_mm"].get<double>(), 20.0);
    EXPECT_DOUBLE_EQ(p["hole_dia_mm"].get<double>(),         3.0);
    EXPECT_EQ(p["hole_count"].get<int>(),                    6);
    EXPECT_DOUBLE_EQ(p["start_angle_deg"].get<double>(),    12.0);
    EXPECT_FALSE(p.contains("hole_centers"));
    EXPECT_FALSE(p.contains("entry_face_id"));
    ASSERT_TRUE(p.contains("axis_dir")) << "the portable pattern datum survives";
    EXPECT_TRUE(res.step.depends_on.empty());
}

// ─── 14. Pattern fit clamp: the PITCH CIRCLE shrinks, the fasteners don't ──
TEST(FeatureTransfer, CounterboreRingClampShrinksPitchCirclePreservesFastenerDias)
{
    const auto src = makeCounterboreRingStep();   // PCD 40, seat 6 -> rOut 23
    const auto res = adapt::transferFeature(
        src, "watch", "phone",
        adapt::AnchorFrame{ 0, 0, 0, 30, 30, 10 },
        adapt::AnchorFrame{ 0, 0, 0, 20, 20, 10 });   // feasible R 18
    ASSERT_TRUE(res.transferred);
    EXPECT_TRUE(res.fit_clamped);

    const json& p = res.step.params;
    EXPECT_NEAR(p["bolt_circle_dia_mm"].get<double>(), 30.0, 1e-9)
        << "newPcd = 2*feasibleR - widest = 36 - 6";
    // The member diameters are the fastener sizes being transferred.
    EXPECT_DOUBLE_EQ(p["seat_dia_mm"].get<double>(),  6.0);
    EXPECT_DOUBLE_EQ(p["pilot_dia_mm"].get<double>(), 3.0);
    // chord = 30*sin(pi/6) = 15 > seat 6 — still a pattern.
}

// ─── 15. A clamp that would MERGE adjacent members refuses ─────────────────
TEST(FeatureTransfer, CounterboreRingMembersMergeAfterClampRefuses)
{
    auto s = makeCounterboreRingStep();
    s.params["count"] = 24;   // post-clamp chord = 30*sin(pi/24) ≈ 3.92 < seat 6
    const auto res = adapt::transferFeature(
        s, "watch", "phone",
        adapt::AnchorFrame{ 0, 0, 0, 30, 30, 10 },
        adapt::AnchorFrame{ 0, 0, 0, 20, 20, 10 });
    EXPECT_FALSE(res.transferred);
    EXPECT_EQ(res.step.skill_id, "refused_transfer")
        << "24 tangent-to-overlapping seats are a ring groove, not a pattern";
}

// ─── 16. Depth clamp may not eat the counterbore SEAT ──────────────────────
// The seat depth is a fastener-critical intrinsic (the screw head sits in it);
// clamping the pilot INTO the seat is a validate error on the skill and a
// different feature dimensionally — refuse instead.
TEST(FeatureTransfer, CounterboreRingDepthClampPreservesSeatOrRefuses)
{
    // (a) enough stock for the seat: pilot depth clamps, seat survives intact.
    auto s = makeCounterboreRingStep();           // pilot_depth 6, PCD 40 -> use
    s.params["bolt_circle_dia_mm"] = 20.0;        // a small ring so radial fits
    s.params["seat_depth_mm"]      = 0.8;
    const auto clamped = adapt::transferFeature(
        s, "watch", "phone",
        adapt::AnchorFrame{ 0, 0, 0, 30, 30, 10 },
        adapt::AnchorFrame{ 0, 0, 0, 20, 20, 1.5 });   // maxDepth = 3 - 1 = 2
    ASSERT_TRUE(clamped.transferred);
    EXPECT_TRUE(clamped.fit_clamped);
    EXPECT_NEAR(clamped.step.params["pilot_depth_mm"].get<double>(), 2.0, 1e-9);
    EXPECT_DOUBLE_EQ(clamped.step.params["seat_depth_mm"].get<double>(), 0.8)
        << "the seat is preserved, only the pilot shortens";

    // (b) the stock cannot even hold the seat: refuse, don't alter it.
    s.params["seat_depth_mm"] = 2.5;              // > maxDepth 2
    const auto refused = adapt::transferFeature(
        s, "watch", "phone",
        adapt::AnchorFrame{ 0, 0, 0, 30, 30, 10 },
        adapt::AnchorFrame{ 0, 0, 0, 20, 20, 1.5 });
    EXPECT_FALSE(refused.transferred);
    EXPECT_EQ(refused.step.skill_id, "refused_transfer");
}

// ─── 17. Depth clamp may not eat the countersink CONE ──────────────────────
TEST(FeatureTransfer, CountersinkRingDepthClampPreservesConeOrRefuses)
{
    process::StepInvocation s;
    s.skill_id = "countersink_ring_pattern";
    s.params = {
        { "count",              6 },
        { "bolt_circle_dia_mm", 20.0 },
        { "center_x_mm",        0.0 },
        { "center_y_mm",        0.0 },
        { "axis_dir",           json::array({ 0.0, 0.0, -1.0 }) },
        { "pilot_dia_mm",       3.0 },
        { "pilot_depth_mm",     6.0 },
        { "cone_top_dia_mm",    8.0 },
        { "cone_angle_deg",    90.0 },   // cone depth = (8-3)/2 / tan(45) = 2.5
        { "start_angle_deg",    0.0 },
    };

    // (a) maxDepth 4 > cone depth 2.5: the pilot clamps, the cone is intact.
    const auto clamped = adapt::transferFeature(
        s, "watch", "phone",
        adapt::AnchorFrame{ 0, 0, 0, 30, 30, 10 },
        adapt::AnchorFrame{ 0, 0, 0, 20, 20, 2.5 });   // maxDepth = 5 - 1 = 4
    ASSERT_TRUE(clamped.transferred);
    EXPECT_TRUE(clamped.fit_clamped);
    EXPECT_NEAR(clamped.step.params["pilot_depth_mm"].get<double>(), 4.0, 1e-9);
    EXPECT_DOUBLE_EQ(clamped.step.params["cone_top_dia_mm"].get<double>(), 8.0);
    EXPECT_DOUBLE_EQ(clamped.step.params["cone_angle_deg"].get<double>(), 90.0);

    // (b) maxDepth 2 <= cone depth 2.5: the cone itself cannot fit — refuse.
    const auto refused = adapt::transferFeature(
        s, "watch", "phone",
        adapt::AnchorFrame{ 0, 0, 0, 30, 30, 10 },
        adapt::AnchorFrame{ 0, 0, 0, 20, 20, 1.5 });   // maxDepth = 3 - 1 = 2
    EXPECT_FALSE(refused.transferred);
    EXPECT_EQ(refused.step.skill_id, "refused_transfer");
}

// ─── 18. A THROUGH bolt circle skips the depth clamp (through is through) ──
TEST(FeatureTransfer, ThroughBoltCircleSkipsDepthClampOnThinDestination)
{
    process::StepInvocation s;
    s.skill_id = "bolt_circle_pattern";
    s.params = {
        { "hole_count",         6 },
        { "bolt_circle_dia_mm", 10.0 },
        { "hole_dia_mm",        3.0 },
        { "center_x_mm",        0.0 },
        { "center_y_mm",        0.0 },
        { "axis_dir",           json::array({ 0.0, 0.0, -1.0 }) },
        { "start_angle_deg",    0.0 },
        // no through_hole key: parseBoltCirclePattern defaults it to TRUE,
        // and the transfer mirrors that default.
    };
    const auto through = adapt::transferFeature(
        s, "watch", "phone",
        adapt::AnchorFrame{ 0, 0, 0, 30, 30, 10 },
        adapt::AnchorFrame{ 0, 0, 0, 20, 20, 0.4 });   // 0.8 mm foil
    ASSERT_TRUE(through.transferred)
        << "a through pattern drills through whatever thickness is there";
    EXPECT_FALSE(through.fit_clamped);

    // An explicitly BLIND bolt circle on the same foil must refuse (no room
    // for any cut + 1 mm floor).
    s.params["through_hole"] = false;
    s.params["depth_mm"]     = 2.0;
    const auto blind = adapt::transferFeature(
        s, "watch", "phone",
        adapt::AnchorFrame{ 0, 0, 0, 30, 30, 10 },
        adapt::AnchorFrame{ 0, 0, 0, 20, 20, 0.4 });
    EXPECT_FALSE(blind.transferred);
}

// ─── 19. A non-vertical (or missing) pattern axis refuses ──────────────────
// Mirrors the pattern skills' own validate error (|axis Z| >= 0.99): a
// transferred=true step must never throw at apply().
TEST(FeatureTransfer, NonVerticalPatternAxisRefuses)
{
    auto s = makeCounterboreRingStep();
    s.params["axis_dir"] = json::array({ 1.0, 0.0, 0.0 });   // side drilling
    const auto side = adapt::transferFeature(
        s, "watch", "phone",
        adapt::AnchorFrame{ 0, 0, 0, 30, 30, 10 },
        adapt::AnchorFrame{ 0, 0, 0, 20, 20, 10 });
    EXPECT_FALSE(side.transferred);
    EXPECT_EQ(side.step.skill_id, "refused_transfer");

    s.params.erase("axis_dir");
    const auto missing = adapt::transferFeature(
        s, "watch", "phone",
        adapt::AnchorFrame{ 0, 0, 0, 30, 30, 10 },
        adapt::AnchorFrame{ 0, 0, 0, 20, 20, 10 });
    EXPECT_FALSE(missing.transferred);
    EXPECT_EQ(missing.step.skill_id, "refused_transfer");
}

// ─── 20. FACE-ANCHORED pattern: a recessed deck limits the PILOT depth ─────
// Same recessed fixture as test 12: the largest +Z face is a pocket floor at
// z = 7, so only 6 mm of clamped depth is honest; the frame-only bbox limit
// (2*5 - 1 = 9) would leave the 8 mm pilot unclamped and sever the deck.
TEST(FeatureTransfer, FaceAnchoredPatternRecessedDeckLimitsPilotDepth)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);
    skill::mill_rect_pocket::Input mp;
    mp.entry_face  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    mp.center_x_mm = 50.0;
    mp.center_y_mm = 20.0;
    mp.length_mm   = 60.0;
    mp.width_mm    = 40.0;
    mp.depth_mm    = 3.0;
    auto stepped = skill::mill_rect_pocket::apply(*stock, mp);
    ASSERT_NE(stepped.workpiece, nullptr);
    const TopoDS_Shape dst = stepped.workpiece->shape();

    auto s = makeCounterboreRingStep();
    s.params["bolt_circle_dia_mm"] = 16.0;   // rOut 11 — fits the 40-wide plate
    s.params["seat_depth_mm"]      = 1.0;
    s.params["pilot_depth_mm"]     = 8.0;
    adapt::TransferOptions opts;
    opts.dst_shape = &dst;
    const auto res = adapt::transferFeature(
        s, "watch", "phone",
        adapt::AnchorFrame{ 0, 0, 0, 22, 22, 5 },
        adapt::AnchorFrame::fromShape(dst), opts);
    ASSERT_TRUE(res.transferred);
    EXPECT_TRUE(res.fit_clamped);
    EXPECT_NEAR(res.step.params["pilot_depth_mm"].get<double>(), 6.0, 1e-6)
        << "depth limit = faceZ(7) - bbox bottom(0) - 1 mm floor";
    EXPECT_DOUBLE_EQ(res.step.params["seat_depth_mm"].get<double>(), 1.0);
}

// ─── 21. Source-intrinsic validate gates are mirrored: apply() never throws ─
// A recovered step CAN carry values the skills refuse to re-synthesise (a
// real Ø0.5 micro-drilled ring exists in metal, but counterbore_ring_pattern
// ::apply hard-throws below 0.8 mm).  transferred=true promises execution.
TEST(FeatureTransfer, SourceViolatingSkillValidateGatesRefuses)
{
    const adapt::AnchorFrame src{ 0, 0, 0, 30, 30, 10 };
    const adapt::AnchorFrame dst{ 0, 0, 0, 20, 20, 10 };

    {   // sub-0.8 pilot: machinable by a micro drill, unsynthesisable by apply.
        auto s = makeCounterboreRingStep();
        s.params["pilot_dia_mm"] = 0.5;
        const auto r = adapt::transferFeature(s, "watch", "phone", src, dst);
        EXPECT_FALSE(r.transferred);
        EXPECT_EQ(r.step.skill_id, "refused_transfer");
    }
    {   // seat not wider than the pilot.
        auto s = makeCounterboreRingStep();
        s.params["seat_dia_mm"] = 2.5;   // pilot 3.0
        const auto r = adapt::transferFeature(s, "watch", "phone", src, dst);
        EXPECT_FALSE(r.transferred);
    }
    {   // pilot ends inside the seat.
        auto s = makeCounterboreRingStep();
        s.params["pilot_depth_mm"] = 1.5;   // seat_depth 2.0
        const auto r = adapt::transferFeature(s, "watch", "phone", src, dst);
        EXPECT_FALSE(r.transferred);
    }
    {   // pitch circle no wider than the widest member — even when it FITS.
        auto s = makeCounterboreRingStep();
        s.params["bolt_circle_dia_mm"] = 5.0;   // seat 6.0
        const auto r = adapt::transferFeature(s, "watch", "phone", src, dst);
        EXPECT_FALSE(r.transferred);
    }
    {   // countersink cone angle outside the ISO envelope.
        process::StepInvocation s;
        s.skill_id = "countersink_ring_pattern";
        s.params = {
            { "count",              6 },
            { "bolt_circle_dia_mm", 20.0 },
            { "center_x_mm",        0.0 },
            { "center_y_mm",        0.0 },
            { "axis_dir",           json::array({ 0.0, 0.0, -1.0 }) },
            { "pilot_dia_mm",       3.0 },
            { "pilot_depth_mm",     6.0 },
            { "cone_top_dia_mm",    8.0 },
            { "cone_angle_deg",    30.0 },   // < 45
            { "start_angle_deg",    0.0 },
        };
        const auto r = adapt::transferFeature(s, "watch", "phone", src, dst);
        EXPECT_FALSE(r.transferred);
    }
    {   // bolt circle of Ø0.5 micro-drilled holes: bolt_circle_pattern::
        // validate itself has no 0.8 floor, but apply() composes drill_hole,
        // which hard-throws DFM-002 — the transfer must mirror the ATOM's
        // gate (the recovery grammar emits the measured dia with no floor).
        process::StepInvocation s;
        s.skill_id = "bolt_circle_pattern";
        s.params = {
            { "hole_count",         6 },
            { "bolt_circle_dia_mm", 20.0 },
            { "hole_dia_mm",        0.5 },
            { "center_x_mm",        0.0 },
            { "center_y_mm",        0.0 },
            { "axis_dir",           json::array({ 0.0, 0.0, -1.0 }) },
            { "start_angle_deg",    0.0 },
        };
        const auto r = adapt::transferFeature(s, "watch", "phone", src, dst);
        EXPECT_FALSE(r.transferred);
        EXPECT_EQ(r.step.skill_id, "refused_transfer");

        // A BLIND bolt circle with no positive depth throws the atom's
        // "blind drill depth must be > 0" — refuse that too.
        s.params["hole_dia_mm"]  = 3.0;
        s.params["through_hole"] = false;
        s.params["depth_mm"]     = 0.0;
        const auto b = adapt::transferFeature(s, "watch", "phone", src, dst);
        EXPECT_FALSE(b.transferred);
    }
}

// ─── 22. The pattern radial clamp uses the REAL per-axis eccentricity ──────
// An off-centre ring on an asymmetric destination: only the Y axis binds
// (|eccY| + rOut > hy − margin while X has slack), so a centred-pattern
// assumption (off = 0 → 13 <= 18, no clamp) or an X/Y axis mixup would leave
// the PCD at 20 and fail the assertion.
TEST(FeatureTransfer, PatternClampUsesRealEccentricityPerAxis)
{
    auto s = makeCounterboreRingStep();
    s.params["bolt_circle_dia_mm"] = 20.0;   // rOut = (20 + 6)/2 = 13
    s.params["center_y_mm"]        = 8.0;
    const auto res = adapt::transferFeature(
        s, "watch", "phone",
        adapt::AnchorFrame{ 0, 0, 0, 40, 20, 10 },    // identity scaling
        adapt::AnchorFrame{ 0, 0, 0, 40, 20, 10 });
    ASSERT_TRUE(res.transferred);
    EXPECT_TRUE(res.fit_clamped)
        << "|eccY| 8 + rOut 13 = 21 > hy 20 - margin 2 = 18";
    // feasibleR = min(40-2-0, 20-2-8) = 10 -> newPcd = 2*10 - 6 = 14;
    // chord = 14*sin(pi/6) = 7 > seat 6 (still a pattern, inside the DFM-003
    // density warning band — allowed through with a note).
    EXPECT_NEAR(res.step.params["bolt_circle_dia_mm"].get<double>(), 14.0, 1e-9);
    EXPECT_DOUBLE_EQ(res.step.params["seat_dia_mm"].get<double>(), 6.0);
}

// ─── 23. ROUND footprint: a diagonal offset clamps against the RIM ─────────
// On a Ø44 disc, a ring at face-local (10,10) has |ecc| = 14.14; the measured
// footprint clamps its OD to 2*(22 - 2 - 14.14) ≈ 11.7.  The per-axis AABB
// (hx = hy = 22) sees offX + OD/2 = 10 + 10 = 20 <= 20 and would not clamp at
// all — the diagonal is exactly where a square model over-allows a circle.
TEST(FeatureTransfer, FaceAnchoredRoundCaseClampsDiagonalOffset)
{
    auto disc = skill::createCylindricalStock(44.0, 10.0);
    const TopoDS_Shape dst = disc->shape();

    process::StepInvocation s;
    s.skill_id = "annular_groove";
    s.params = {
        { "center_x_mm",  10.0 },
        { "center_y_mm",  10.0 },
        { "outer_dia_mm", 20.0 },
        { "inner_dia_mm", 14.0 },
        { "depth_mm",      1.0 },
    };
    adapt::TransferOptions opts;
    opts.dst_shape = &dst;
    const auto res = adapt::transferFeature(
        s, "phone", "watch",
        adapt::AnchorFrame{ 0, 0, 0, 22, 22, 5 },
        adapt::AnchorFrame::fromShape(dst), opts);
    ASSERT_TRUE(res.transferred);
    EXPECT_TRUE(res.fit_clamped)
        << "the AABB model would see 10 + 10 <= 22 - 2 and skip the clamp";
    const double odClamped = res.step.params["outer_dia_mm"].get<double>();
    EXPECT_GT(odClamped, 11.0);
    EXPECT_LT(odClamped, 12.4)
        << "feasible OD = 2*(rim 22 - margin 2 - |ecc| 14.14) ~ 11.7";
    // Ratio-preserving: the ID shrinks by the same factor.
    EXPECT_NEAR(res.step.params["inner_dia_mm"].get<double>(),
                odClamped * 14.0 / 20.0, 1e-6);
}

// ─── 24. Lug/crown-like protrusions do NOT inflate the footprint ────────────
// Four tabs push the bbox to 60 x 60 (hx = hy = 30), so the AABB test would
// pass a PCD-40 ring (rOut 22 <= 30 - 2) untouched; the measured footprint
// still clamps against the Ø44 case rim between the tabs.
TEST(FeatureTransfer, FaceAnchoredLugsDoNotInflateFootprint)
{
    auto disc = skill::createCylindricalStock(44.0, 10.0);
    TopoDS_Shape dst = disc->shape();
    const double tabs[4][2] = { { 1.0, 0.0 }, { -1.0, 0.0 },
                                { 0.0, 1.0 }, { 0.0, -1.0 } };
    for (const auto& t : tabs) {
        // A 6-wide tab reaching from the rim to |axis| = 30 (z 0..10).
        const gp_Pnt lo(t[0] != 0.0 ? (t[0] > 0 ? 20.0 : -30.0) : -3.0,
                        t[1] != 0.0 ? (t[1] > 0 ? 20.0 : -30.0) : -3.0, 0.0);
        const gp_Pnt hi(t[0] != 0.0 ? (t[0] > 0 ? 30.0 : -20.0) : 3.0,
                        t[1] != 0.0 ? (t[1] > 0 ? 30.0 : -20.0) : 3.0, 10.0);
        dst = BRepAlgoAPI_Fuse(dst, BRepPrimAPI_MakeBox(lo, hi).Shape()).Shape();
    }

    auto s = makeCounterboreRingStep();          // world centre (0, 0)
    s.params["bolt_circle_dia_mm"] = 40.0;       // seat 6 -> rOut 23
    const adapt::AnchorFrame dstFrame = adapt::AnchorFrame::fromShape(dst);
    EXPECT_NEAR(dstFrame.hx, 30.0, 0.2) << "the tabs must inflate the bbox";
    adapt::TransferOptions opts;
    opts.dst_shape = &dst;
    const auto res = adapt::transferFeature(
        s, "phone", "watch",
        adapt::AnchorFrame{ 0, 0, 0, 30, 30, 10 },   // identity scaling
        dstFrame, opts);
    ASSERT_TRUE(res.transferred);
    EXPECT_TRUE(res.fit_clamped)
        << "the inflated AABB (0 + 23 <= 30 - 2) would skip the clamp; the "
           "45-degree samples lie off the Ø44 case and off the tabs";
    // feasibleR + margin <= rim 22 -> feasibleR ~ 20 -> newPcd ~ 34.
    EXPECT_NEAR(res.step.params["bolt_circle_dia_mm"].get<double>(), 34.0, 0.8);
    EXPECT_DOUBLE_EQ(res.step.params["seat_dia_mm"].get<double>(), 6.0);
}

// ─── 24b. Ring fit is PER-MEMBER: a void between members must not clamp ────
// A Ø3 pocket sits on the OUTER test circle's path (radius 15, at 45°) but
// exactly between two members (15° and 75°): the outer-circle bound would
// cross the void and clamp; the members' own circles (pitch radius 10,
// rm = 5) stay clear of it — the ring fits as-is.
TEST(FeatureTransfer, FaceAnchoredRingVoidBetweenMembersDoesNotClamp)
{
    auto disc = skill::createCylindricalStock(44.0, 10.0);
    skill::mill_rect_pocket::Input mp;      // a small square void stands in
    mp.entry_face  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    mp.center_x_mm = 15.0 * std::cos(M_PI / 4.0);   // (10.6, 10.6): radius 15,
    mp.center_y_mm = 15.0 * std::sin(M_PI / 4.0);   // 45 deg — between members
    mp.length_mm   = 3.0;
    mp.width_mm    = 3.0;
    mp.depth_mm    = 3.0;
    const auto pocketed = skill::mill_rect_pocket::apply(*disc, mp);
    ASSERT_NE(pocketed.workpiece, nullptr);
    const TopoDS_Shape dst = pocketed.workpiece->shape();

    auto s = makeCounterboreRingStep();     // PCD 40 -> use 20; start 15 deg
    s.params["bolt_circle_dia_mm"] = 20.0;
    adapt::TransferOptions opts;
    opts.dst_shape = &dst;
    const auto res = adapt::transferFeature(
        s, "phone", "watch",
        adapt::AnchorFrame{ 0, 0, 0, 22, 22, 5 },
        adapt::AnchorFrame::fromShape(dst), opts);
    ASSERT_TRUE(res.transferred);
    EXPECT_FALSE(res.fit_clamped)
        << "the void at radius 15/45deg lies on the OUTER circle (rOut+margin "
           "= 15) but between the members at 15 and 75 deg — a per-member fit "
           "leaves the ring untouched";
    EXPECT_DOUBLE_EQ(res.step.params["bolt_circle_dia_mm"].get<double>(), 20.0);
}

// ═══ LINEAR hole array (speaker-grille class): start/direction/pitch
//     placement, per-member fit, pitch-only clamp about the array centre. ═══

namespace {

process::StepInvocation makeLinearArrayStep()
{
    process::StepInvocation s;
    s.skill_id = "linear_hole_array";
    s.params = {
        { "hole_count",   5 },
        { "hole_dia_mm",  1.5 },
        { "pitch_mm",     3.0 },
        { "span_mm",     12.0 },
        { "start_x_mm",  -6.0 },
        { "start_y_mm",   0.0 },
        { "direction",    json::array({ 1.0, 0.0, 0.0 }) },
        { "axis_dir",     json::array({ 0.0, 0.0, -1.0 }) },
        { "position_z_mm", 10.0 },
        { "hole_centers", json::array({ json::array({ -6.0, 0.0, 10.0 }) }) },
    };
    return s;
}

}  // namespace

// ─── 25. Linear start re-expresses world-RELATIVE; intrinsics survive ───────
TEST(FeatureTransfer, LinearArrayStartReExpressesRelativeToFrames)
{
    auto s = makeLinearArrayStep();
    s.params["start_x_mm"] = 24.0;   // world, in a frame centred at 30
    s.params["start_y_mm"] = 10.0;
    const auto res = adapt::transferFeature(
        s, "phone", "watch",
        adapt::AnchorFrame{ 30.0, 10.0, 5.0, 20.0, 20.0, 5.0 },
        adapt::AnchorFrame{  0.0,  0.0, 0.0, 40.0, 40.0, 5.0 });
    ASSERT_TRUE(res.transferred);
    EXPECT_FALSE(res.fit_clamped);

    const json& tp = res.step.params;
    EXPECT_NEAR(tp["start_x_mm"].get<double>(), -12.0, 1e-9)
        << "0 + (24-30)*(40/20) — world-relative, not the bare ratio 48";
    EXPECT_NEAR(tp["start_y_mm"].get<double>(), 0.0, 1e-9);
    EXPECT_DOUBLE_EQ(tp["pitch_mm"].get<double>(),    3.0);
    EXPECT_DOUBLE_EQ(tp["hole_dia_mm"].get<double>(), 1.5);
    EXPECT_EQ(tp["hole_count"].get<int>(),            5);
    EXPECT_FALSE(tp.contains("hole_centers"));
    ASSERT_TRUE(tp.contains("axis_dir"));
    ASSERT_TRUE(tp.contains("direction")) << "the row direction is intrinsic";
}

// ─── 26. Linear fit clamps the PITCH about the array centre ────────────────
// Frame-only: an X-row of 5 holes centred at x = 10 with halfSpan 6 must end
// at 10 + 6 + dia/2 <= hx - margin; on a hx = 14 destination the pitch
// shrinks from 3.0 to s*3.0 with s = (14-2-0.75-10)/6 = 0.208.
TEST(FeatureTransfer, LinearArrayClampShrinksPitchPreservesDiaAndCount)
{
    auto s = makeLinearArrayStep();
    s.params["start_x_mm"] = 4.0;    // centre 10, halfSpan 6 -> reaches 16.75
    const auto res = adapt::transferFeature(
        s, "phone", "watch",
        adapt::AnchorFrame{ 0, 0, 0, 14, 14, 5 },
        adapt::AnchorFrame{ 0, 0, 0, 14, 14, 5 });   // identity scaling
    // s = 0.208 -> newPitch 0.625 < dia 1.5 -> the holes would merge: REFUSE.
    EXPECT_FALSE(res.transferred);
    EXPECT_EQ(res.step.skill_id, "refused_transfer");

    // A milder violation clamps instead: centre 2, halfSpan 6 -> reaches
    // 8.75 <= 12? yes — use a farther start: centre 4 -> 10.75 <= 12 fits.
    // Overhang case: centre 6 -> 12.75 > 12 -> s = (12-0.75-6)/6 = 0.875.
    s.params["start_x_mm"] = 0.0;    // centre 6
    const auto clamped = adapt::transferFeature(
        s, "phone", "watch",
        adapt::AnchorFrame{ 0, 0, 0, 14, 14, 5 },
        adapt::AnchorFrame{ 0, 0, 0, 14, 14, 5 });
    ASSERT_TRUE(clamped.transferred);
    EXPECT_TRUE(clamped.fit_clamped);
    const json& tp = clamped.step.params;
    EXPECT_NEAR(tp["pitch_mm"].get<double>(), 3.0 * 0.875, 1e-6);
    EXPECT_DOUBLE_EQ(tp["hole_dia_mm"].get<double>(), 1.5);
    EXPECT_EQ(tp["hole_count"].get<int>(), 5);
    // The clamp is centre-preserving: start moves in by the same scale.
    EXPECT_NEAR(tp["start_x_mm"].get<double>(), 6.0 - 0.875 * 6.0, 1e-6);
    EXPECT_NEAR(tp["span_mm"].get<double>(), 4.0 * 3.0 * 0.875, 1e-6)
        << "the span breadcrumb must stay true after the clamp";
}

// ─── 27. Linear atom-gate mirror: sub-0.8 holes / blind depth<=0 refuse ────
TEST(FeatureTransfer, LinearArrayAtomGateViolationsRefuse)
{
    const adapt::AnchorFrame f{ 0, 0, 0, 30, 30, 10 };
    {
        auto s = makeLinearArrayStep();
        s.params["hole_dia_mm"] = 0.5;   // micro-drill: apply() throws DFM-002
        EXPECT_FALSE(adapt::transferFeature(s, "phone", "watch", f, f).transferred);
    }
    {
        auto s = makeLinearArrayStep();
        s.params["through_hole"] = false;
        s.params["depth_mm"]     = 0.0;  // blind depth must be > 0
        EXPECT_FALSE(adapt::transferFeature(s, "phone", "watch", f, f).transferred);
    }
    {
        auto s = makeLinearArrayStep();
        s.params["axis_dir"] = json::array({ 1.0, 0.0, 0.0 });   // side row
        EXPECT_FALSE(adapt::transferFeature(s, "phone", "watch", f, f).transferred);
    }
}

// ─── 27b. Rectangular grid: single-scale pitch clamp preserves the aspect ──
// A 4x3 grid overhanging in X on an identity frame: BOTH pitches scale by
// the same s (the grid's aspect is its identity), cols/rows/dia are
// preserved, and the origin moves in so the centre stays put.
TEST(FeatureTransfer, RectangularGridClampScalesBothPitchesAboutCentre)
{
    process::StepInvocation s;
    s.skill_id = "rectangular_hole_grid";
    s.params = {
        { "hole_count",  12 },
        { "hole_dia_mm",  1.5 },
        { "cols",         4 },
        { "rows",         3 },
        { "pitch_u_mm",   4.0 },
        { "pitch_v_mm",   3.0 },
        { "origin_x_mm",  0.0 },     // centre (6, 3): hu = 6, hv = 3
        { "origin_y_mm",  0.0 },
        { "u_dir",        json::array({ 1.0, 0.0, 0.0 }) },
        { "v_dir",        json::array({ 0.0, 1.0, 0.0 }) },
        { "axis_dir",     json::array({ 0.0, 0.0, -1.0 }) },
        { "hole_centers", json::array({ json::array({ 0.0, 0.0, 10.0 }) }) },
    };
    // Frame hx = 10: farthest member x = 12, 12 + 2.75 > 10 -> violated.
    // Closed form: 6 + 6s + 2.75 <= 10 -> s = 1.25/6 = 0.2083 -> min pitch
    // 3*0.2083 = 0.625 < dia 1.5 -> the grid would merge: REFUSE.
    const auto refused = adapt::transferFeature(
        s, "phone", "watch",
        adapt::AnchorFrame{ 0, 0, 0, 10, 10, 5 },
        adapt::AnchorFrame{ 0, 0, 0, 10, 10, 5 });
    EXPECT_FALSE(refused.transferred);

    // Frame hx = 14: 6 + 6s + 2.75 <= 14 -> s = 5.25/6 = 0.875; min pitch
    // 2.625 >= 1.5 -> clamp.  Y never binds (3 + 3s + 2.75 <= 14).
    const auto clamped = adapt::transferFeature(
        s, "phone", "watch",
        adapt::AnchorFrame{ 0, 0, 0, 14, 14, 5 },
        adapt::AnchorFrame{ 0, 0, 0, 14, 14, 5 });
    ASSERT_TRUE(clamped.transferred);
    EXPECT_TRUE(clamped.fit_clamped);
    const json& tp = clamped.step.params;
    EXPECT_NEAR(tp["pitch_u_mm"].get<double>(), 4.0 * 0.875, 1e-6);
    EXPECT_NEAR(tp["pitch_v_mm"].get<double>(), 3.0 * 0.875, 1e-6)
        << "BOTH pitches scale by one s — the grid keeps its aspect";
    EXPECT_EQ(tp["cols"].get<int>(), 4);
    EXPECT_EQ(tp["rows"].get<int>(), 3);
    EXPECT_DOUBLE_EQ(tp["hole_dia_mm"].get<double>(), 1.5);
    // Centre-preserving: origin = centre - s*(hu, hv) = (6-5.25, 3-2.625).
    EXPECT_NEAR(tp["origin_x_mm"].get<double>(), 0.75,  1e-6);
    EXPECT_NEAR(tp["origin_y_mm"].get<double>(), 0.375, 1e-6);
    EXPECT_FALSE(tp.contains("hole_centers"));
}

// ─── 27c. Grid gate mirror: too-few holes / sub-0.8 dia refuse ─────────────
TEST(FeatureTransfer, RectangularGridGateViolationsRefuse)
{
    const adapt::AnchorFrame f{ 0, 0, 0, 30, 30, 10 };
    process::StepInvocation s;
    s.skill_id = "rectangular_hole_grid";
    s.params = {
        { "hole_dia_mm", 1.5 },
        { "cols",        2 },
        { "rows",        2 },     // 4 holes < 6 — the skill's validate error
        { "pitch_u_mm",  4.0 },
        { "pitch_v_mm",  3.0 },
        { "origin_x_mm", 0.0 },
        { "origin_y_mm", 0.0 },
        { "axis_dir",    json::array({ 0.0, 0.0, -1.0 }) },
    };
    EXPECT_FALSE(adapt::transferFeature(s, "phone", "watch", f, f).transferred);

    s.params["rows"] = 3;
    s.params["hole_dia_mm"] = 0.5;   // micro-drill: the atom throws DFM-002
    EXPECT_FALSE(adapt::transferFeature(s, "phone", "watch", f, f).transferred);

    s.params["hole_dia_mm"] = 1.5;   // sanity: the base grid DOES transfer
    EXPECT_TRUE(adapt::transferFeature(s, "phone", "watch", f, f).transferred);
}

// ─── 27d. SKEWED grid: the merge gate uses the true lattice spacing ─────────
// fitGridXY only requires a non-collinear basis, so a legitimately skewed
// grid's closest neighbours sit on the short cross-diagonal |pu·û − pv·v̂|
// (1.58 here), well under min(pu, pv) = 3.81.  A clamp to s = 0.83 fuses
// those neighbours (1.32 < dia 1.5) — a pitch-only gate would pass it.
TEST(FeatureTransfer, SkewedGridMergeGateUsesLatticeSpacing)
{
    process::StepInvocation s;
    s.skill_id = "rectangular_hole_grid";
    s.params = {
        { "hole_dia_mm", 1.5 },
        { "cols",        3 },
        { "rows",        3 },
        { "pitch_u_mm",  3.8079 },   // |(3.5, 1.5)|
        { "pitch_v_mm",  4.0 },
        { "origin_x_mm", 0.0 },
        { "origin_y_mm", 0.0 },
        { "u_dir",       json::array({ 3.5, 1.5, 0.0 }) },   // skewed column axis
        { "v_dir",       json::array({ 1.0, 0.0, 0.0 }) },
        { "axis_dir",    json::array({ 0.0, 0.0, -1.0 }) },
    };
    // Members span x 0..15, centre (7.5, 1.5); an hx = 9 identity frame
    // forces s = (9 - 2.75)/7.5 = 0.833.
    const auto res = adapt::transferFeature(
        s, "phone", "watch",
        adapt::AnchorFrame{ 7.5, 1.5, 5, 9, 9, 5 },
        adapt::AnchorFrame{ 7.5, 1.5, 5, 9, 9, 5 });
    EXPECT_FALSE(res.transferred)
        << "diagonal neighbours at 1.58 * 0.833 = 1.32 mm < dia 1.5 merge — "
           "min(pu, pv) * s = 3.17 would wrongly pass";
    EXPECT_EQ(res.step.skill_id, "refused_transfer");
}

// ─── 28. Face-anchored linear fit is PER-MEMBER, not a bounding circle ─────
// A row along Y on the recessed-deck plate: every member sits in material,
// but the array's BOUNDING circle (halfSpan 6 + margin) would poke past the
// plate's y-extent — a bounding-circle implementation would clamp a row that
// actually fits.
TEST(FeatureTransfer, FaceAnchoredLinearRowFitsPerMember)
{
    auto stock = skill::createCuboidStock(80.0, 16.0, 10.0);   // narrow in Y
    const TopoDS_Shape dst = stock->shape();

    auto s = makeLinearArrayStep();                 // 5 holes, pitch 3, dia 1.5
    s.params["start_x_mm"] = 34.0;                  // row ALONG X: centre (40, 8)
    s.params["start_y_mm"] = 8.0;
    adapt::TransferOptions opts;
    opts.dst_shape = &dst;
    const auto res = adapt::transferFeature(
        s, "phone", "watch",
        adapt::AnchorFrame{ 40, 8, 5, 40, 8, 5 },   // identity
        adapt::AnchorFrame::fromShape(dst), opts);
    ASSERT_TRUE(res.transferred);
    EXPECT_FALSE(res.fit_clamped)
        << "members reach y 8±0.75+margin 2 <= 16 and x 34..46 — in material; "
           "a bounding circle (r = 6+0.75+2 = 8.75 about (40,8)) would spill "
           "past y = 16 and wrongly clamp";
}
