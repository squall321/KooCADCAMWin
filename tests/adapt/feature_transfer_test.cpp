// @lat: [[engine/skills#Layer 5 LLM adapter]]
//
// FeatureTransfer — slice-1 cross-product transfer unit spec.
//
//   1. ClassifyParamTable          — a few keys per role map correctly, and
//                                    unknown keys default to Intrinsic.
//   2. AnnularGrooveTransfers      — positions scale by the half-extent
//                                    ratios, intrinsics survive bit-for-bit,
//                                    product-bound keys are stripped, the
//                                    front-face datum is injected, and
//                                    depends_on is cleared.
//   3. FitClampRatioPreserving     — an OD-80 ring into a 38-half-width
//                                    destination clamps OD/ID by 0.9.
//   4. ImpossibleFitRefuses        — post-clamp groove wall < 0.5 mm must
//                                    refuse (transferred=false), not lie.
//   5. UnsupportedSkillRefuses     — drill_hole is not whitelisted in
//                                    slice-1; explicit refusal + note.

#include <gtest/gtest.h>

#include "adapt/FeatureTransfer.hpp"

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/mill_rect_pocket.hpp"

#include <TopoDS_Shape.hxx>
#include <gp_Dir.hxx>

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
    // feasible OD = (hx 40 - margin 2 - ecc ~14.5) * 2 ≈ 47 (±corner-fillet
    // wobble on the resolved face centre).
    const double odClamped = res.step.params["outer_dia_mm"].get<double>();
    EXPECT_GT(odClamped, 45.0);
    EXPECT_LT(odClamped, 48.0) << "clamped strictly below the source OD 48";
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
