// @lat: [[process/test-strategy#skill round-trip]]
//
// circular_pattern — N holes around an axis on a disc.
//
// Cases (5):
//   1. 6 holes on 360° remove ~6 × per-hole volume on a disc.
//   2. 4 holes on 180° remove ~4 × per-hole volume.
//   3. DFM rejects chord-overlap (small radial_offset + small count).
//   4. DFM rejects count = 0.
//   5. recognize finds the bolt circle (history replay).

#include <gtest/gtest.h>

#include "process/Executor.hpp"
#include "process/ProcessPlan.hpp"
#include "re/Recognizer.hpp"
#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/circular_pattern.hpp"
#include "skills/drill_hole.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. 6 holes around full circle ─────────────────────────────────────────
TEST(SkillCircularPattern, SixHolesFullCircle)
{
    auto stock = skill::createCylindricalStock(80.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::circular_pattern::Input in;
    in.axis_origin_xyz   = { 0.0, 0.0, 10.0 };
    in.axis_dir_xyz      = { 0.0, 0.0, 1.0 };
    in.hole_dia_mm       = 5.0;
    in.hole_depth_mm     = 6.0;
    in.radial_offset_mm  = 30.0;
    in.count             = 6;
    in.total_angle_deg   = 360.0;

    auto out = skill::circular_pattern::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double removed  = volBefore - volumeOf(out.workpiece->shape());
    const double per      = M_PI * 2.5 * 2.5 * 6.0;
    const double expected = 6.0 * per;
    EXPECT_NEAR(removed, expected, expected * 0.15);
    EXPECT_EQ(out.signature.pattern["instance_count"].get<int>(), 6);
    EXPECT_TRUE(out.signature.pattern["is_pattern"].get<bool>());
}

// ─── 2. 4 holes over 180° ──────────────────────────────────────────────────
TEST(SkillCircularPattern, FourHolesHalfCircle)
{
    auto stock = skill::createCylindricalStock(80.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::circular_pattern::Input in;
    in.axis_origin_xyz  = { 0.0, 0.0, 10.0 };
    in.axis_dir_xyz     = { 0.0, 0.0, 1.0 };
    in.hole_dia_mm      = 4.0;
    in.hole_depth_mm    = 5.0;
    in.radial_offset_mm = 25.0;
    in.count            = 4;
    in.total_angle_deg  = 180.0;

    auto out = skill::circular_pattern::apply(*stock, in);
    const double removed = volBefore - volumeOf(out.workpiece->shape());
    const double per      = M_PI * 2.0 * 2.0 * 5.0;
    const double expected = 4.0 * per;
    EXPECT_NEAR(removed, expected, expected * 0.15);
}

// ─── 3. DFM rejects chord-pitch overlap ────────────────────────────────────
TEST(SkillCircularPattern, ValidateRejectsChordOverlap)
{
    auto stock = skill::createCylindricalStock(80.0, 10.0);
    skill::circular_pattern::Input in;
    in.axis_origin_xyz  = { 0.0, 0.0, 10.0 };
    in.hole_dia_mm      = 20.0;
    in.hole_depth_mm    = 5.0;
    in.radial_offset_mm = 5.0;       // chord = 2*5*sin(π/12) ≈ 2.6 < 20
    in.count            = 12;
    in.total_angle_deg  = 360.0;
    auto r = skill::circular_pattern::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings) if (f.code == "DFM-PATT-2") found = true;
    EXPECT_TRUE(found);
}

// ─── 4. DFM rejects count = 0 ──────────────────────────────────────────────
TEST(SkillCircularPattern, ValidateRejectsZeroCount)
{
    auto stock = skill::createCylindricalStock(80.0, 10.0);
    skill::circular_pattern::Input in;
    in.hole_dia_mm      = 4.0;
    in.hole_depth_mm    = 5.0;
    in.radial_offset_mm = 25.0;
    in.count            = 0;
    in.total_angle_deg  = 360.0;
    auto r = skill::circular_pattern::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 5. Recognize identifies circle (history replay) ───────────────────────
TEST(SkillCircularPattern, RecognizeIdentifiesPattern)
{
    auto stock = skill::createCylindricalStock(80.0, 10.0);
    skill::circular_pattern::Input in;
    in.axis_origin_xyz  = { 0.0, 0.0, 10.0 };
    in.axis_dir_xyz     = { 0.0, 0.0, 1.0 };
    in.hole_dia_mm      = 5.0;
    in.hole_depth_mm    = 6.0;
    in.radial_offset_mm = 30.0;
    in.count            = 6;
    in.total_angle_deg  = 360.0;
    auto out = skill::circular_pattern::apply(*stock, in);
    auto cands = skill::circular_pattern::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("circular_pattern"));
    EXPECT_GT(cands[0].confidence, 0.5);
}

// ─── 6. GEOMETRIC recognition (no history): a 6-hole bolt circle is recovered
// as one pattern with the right count + radial offset, from geometry alone. ──
TEST(SkillCircularPattern, RecognizeFromForeignGeometry)
{
    auto stock = skill::createCylindricalStock(80.0, 10.0);
    skill::circular_pattern::Input in;
    in.axis_origin_xyz  = { 0.0, 0.0, 10.0 };
    in.axis_dir_xyz     = { 0.0, 0.0, 1.0 };
    in.hole_dia_mm      = 5.0;
    in.hole_depth_mm    = 6.0;
    in.radial_offset_mm = 30.0;
    in.count            = 6;
    in.total_angle_deg  = 360.0;
    auto built = skill::circular_pattern::apply(*stock, in);

    skill::Workpiece foreign(built.workpiece->shape());   // NO history
    auto cands = skill::circular_pattern::recognize(foreign);
    ASSERT_GE(cands.size(), 1u) << "a 6-hole bolt circle must be recognised";
    const auto& p = cands[0].recovered_params;
    EXPECT_EQ(p.value("count", 0), 6) << "all 6 holes counted";
    EXPECT_NEAR(p.value("radial_offset_mm", 0.0), 30.0, 1.0) << "radius recovered";
    ASSERT_TRUE(p.contains("hole_centers"));
    EXPECT_EQ(p["hole_centers"].size(), 6u) << "centres emitted for subsumption";
    // Depth MUST be recovered (>0) — the old code hard-coded 0, which aborted the
    // Executor replay via validate()'s DFM gate.  Confidence must reach 0.7.
    EXPECT_NEAR(p.value("hole_depth_mm", 0.0), 6.0, 0.5) << "hole depth recovered";
    EXPECT_GE(cands[0].confidence, 0.7) << "must reach the Executor threshold";
    // The axis origin Z must be the real ENTRY plane (stock top = 10), NOT a
    // hard-coded 0 — otherwise the regenerated circle bores into air below the
    // stock instead of the real hole plane.
    ASSERT_TRUE(p.contains("axis_origin_xyz"));
    EXPECT_NEAR(p["axis_origin_xyz"][2].get<double>(), 10.0, 0.5)
        << "axis origin Z must be the hole entry plane, not 0";
}

// ─── 6b. END-TO-END replay: a recovered bolt circle must REGENERATE through the
// Executor (it was dead before — no dispatch entry + zero depth aborted it). ───
TEST(SkillCircularPattern, ForeignBoltCircleReplaysThroughExecutor)
{
    auto stock = skill::createCylindricalStock(80.0, 10.0);
    skill::circular_pattern::Input in;
    in.axis_origin_xyz  = { 0.0, 0.0, 10.0 };
    in.axis_dir_xyz     = { 0.0, 0.0, 1.0 };
    in.hole_dia_mm      = 5.0;
    in.hole_depth_mm    = 6.0;
    in.radial_offset_mm = 30.0;
    in.count            = 6;
    in.total_angle_deg  = 360.0;
    auto built = skill::circular_pattern::apply(*stock, in);
    const double vBuilt = volumeOf(built.workpiece->shape());

    skill::Workpiece foreign(built.workpiece->shape());
    const process::ProcessPlan plan = re::inferProcessPlan(foreign, 0.7);
    // The bolt circle must survive to the plan as ONE editable pattern step and
    // subsume its 6 individual drills.  With circular_pattern now carrying a
    // face-id key, dedupe arbitrates it against the grammar's more-specific
    // bolt_circle_pattern for the same holes — so exactly one of the two remains
    // (no duplicate pattern steps), and no loose drills leak.
    int patternSteps = 0, drillSteps = 0;
    for (const auto& s : plan.steps()) {
        if (s.skill_id == "circular_pattern" || s.skill_id == "bolt_circle_pattern")
            ++patternSteps;
        if (s.skill_id == "drill_hole" || s.skill_id == "drill_through_hole")
            ++drillSteps;
    }
    EXPECT_EQ(patternSteps, 1)
        << "exactly one bolt-circle pattern step (dedupe removed the duplicate)";
    EXPECT_EQ(drillSteps, 0) << "the pattern subsumes the individual drills";

    // Replay the plan on fresh stock → same removed volume (no zero-depth abort).
    process::ProcessPlan replayable;
    for (const auto& step : plan.steps()) replayable.append(re::liftRecoveredStep(step));
    auto fresh = skill::createCylindricalStock(80.0, 10.0);
    const double vFresh = volumeOf(fresh->shape());
    auto result = process::Executor::execute(replayable, fresh, process::ExecuteOptions{true});
    ASSERT_NE(result.workpiece, nullptr);
    ASSERT_FALSE(result.workpiece->shape().IsNull());
    EXPECT_NEAR(vFresh - volumeOf(result.workpiece->shape()), vFresh - vBuilt,
                (vFresh - vBuilt) * 0.10)
        << "the recovered bolt circle must regenerate the same removed volume";
}

// ─── 6c. END-TO-END replay for a SIDE ring: a recovered non-Z bolt circle must
// survive to the plan as ONE pattern step and regenerate the holes (not vanish).
// This drives the whole RE pipeline (recognize → dedupe → subsumption → Executor),
// which the isolated recognize/volume tests do not exercise. ─────────────────
TEST(SkillCircularPattern, ForeignSideBoltCircleReplaysThroughExecutor)
{
    auto stock = skill::createCuboidStock(40.0, 60.0, 60.0);
    skill::circular_pattern::Input in;
    in.axis_origin_xyz  = { 40.0, 30.0, 30.0 };   // +X face, ring centred on it
    in.axis_dir_xyz     = { 1.0, 0.0, 0.0 };
    in.hole_dia_mm      = 5.0;
    in.hole_depth_mm    = 6.0;
    in.radial_offset_mm = 18.0;
    in.count            = 5;
    in.total_angle_deg  = 360.0;
    auto built = skill::circular_pattern::apply(*stock, in);
    const double vBuilt = volumeOf(built.workpiece->shape());

    skill::Workpiece foreign(built.workpiece->shape());
    const process::ProcessPlan plan = re::inferProcessPlan(foreign, 0.7);

    int patternSteps = 0, holeSteps = 0;
    for (const auto& s : plan.steps()) {
        if (s.skill_id == "circular_pattern" || s.skill_id == "bolt_circle_pattern")
            ++patternSteps;
        if (s.skill_id == "drill_hole" || s.skill_id == "drill_through_hole" ||
            s.skill_id == "bore_cylindrical" || s.skill_id == "mill_circular_pocket")
            ++holeSteps;
    }
    // The side ring must reach the plan as SOME regenerable hole geometry — either
    // one pattern step (preferred) or its individual holes — never vanish entirely.
    EXPECT_GE(patternSteps + holeSteps, 1)
        << "the side bolt circle must not be wiped from the plan (holes must survive)";
    EXPECT_EQ(patternSteps, 1) << "the side ring survives as one editable pattern step";

    // Replay → the recovered plan must remove ~the same volume (holes regenerated).
    process::ProcessPlan replayable;
    for (const auto& step : plan.steps()) replayable.append(re::liftRecoveredStep(step));
    auto fresh = skill::createCuboidStock(40.0, 60.0, 60.0);
    const double vFresh = volumeOf(fresh->shape());
    auto result = process::Executor::execute(replayable, fresh, process::ExecuteOptions{true});
    ASSERT_NE(result.workpiece, nullptr);
    ASSERT_FALSE(result.workpiece->shape().IsNull());
    EXPECT_NEAR(vFresh - volumeOf(result.workpiece->shape()), vFresh - vBuilt,
                (vFresh - vBuilt) * 0.15)
        << "the recovered side bolt circle must regenerate ~the same removed volume";
}

// ─── 7. NO false pattern: an irregular cluster (not on one circle, not evenly
// spaced) must NOT be read as a bolt circle. ─────────────────────────────────
TEST(SkillCircularPattern, IrregularClusterIsNotAPattern)
{
    auto stock = skill::createCylindricalStock(80.0, 10.0);
    // Three same-diameter holes at WILDLY different radii from centre — fails
    // the 5% radial-deviation gate, so it is not a bolt circle.
    const double rr[3] = { 8.0, 20.0, 33.0 };
    std::shared_ptr<skill::Workpiece> wp = stock;
    for (double rad : rr) {
        skill::drill_hole::Input h;
        h.position_x_mm = rad; h.position_y_mm = 0.0;
        h.diameter_mm = 4.0;   h.depth_mm = 5.0;
        wp = skill::drill_hole::apply(*wp, h).workpiece;
    }
    skill::Workpiece foreign(wp->shape());
    auto cands = skill::circular_pattern::recognize(foreign);
    EXPECT_TRUE(cands.empty())
        << "holes at different radii must not be a circular pattern";
}

// ─── 7b. THROUGH-hole ring: both hole ends are open (both probes OUT), so the
// recogniser takes the canonicalised-axis fallback.  The ring must still be
// recognised with a consistent entry on ONE of the two faces and round-trip. ──
TEST(SkillCircularPattern, ThroughHoleRingRecognisedConsistently)
{
    // 10mm plate; hole_depth = 10 → every hole is a through hole.
    auto stock = skill::createCylindricalStock(80.0, 10.0);
    skill::circular_pattern::Input in;
    in.axis_origin_xyz  = { 0.0, 0.0, 10.0 };
    in.axis_dir_xyz     = { 0.0, 0.0, 1.0 };
    in.hole_dia_mm      = 5.0;
    in.hole_depth_mm    = 10.0;          // == plate thickness → through
    in.radial_offset_mm = 30.0;
    in.count            = 6;
    in.total_angle_deg  = 360.0;
    auto built = skill::circular_pattern::apply(*stock, in);
    const double vBuilt = volumeOf(built.workpiece->shape());

    skill::Workpiece foreign(built.workpiece->shape());
    auto cands = skill::circular_pattern::recognize(foreign);
    ASSERT_GE(cands.size(), 1u) << "a through-hole bolt circle must be recognised";
    const auto& p = cands[0].recovered_params;
    EXPECT_EQ(p.value("count", 0), 6);
    EXPECT_NEAR(p.value("radial_offset_mm", 0.0), 30.0, 1.0);
    EXPECT_NEAR(p.value("hole_depth_mm", 0.0), 10.0, 0.8) << "full through depth";
    // The entry plane must be ONE of the two real faces (Z=0 or Z=10) — never a
    // mid-plate mean of mixed picks (the pre-fix sign-dependent failure mode).
    ASSERT_TRUE(p.contains("axis_origin_xyz"));
    const double oz = p["axis_origin_xyz"][2].get<double>();
    EXPECT_TRUE(std::abs(oz - 10.0) < 0.8 || std::abs(oz - 0.0) < 0.8)
        << "entry Z=" << oz << " must sit on a real face, not mid-plate";

    // Round-trip: regenerating from the recovered params removes the same volume.
    skill::circular_pattern::Input re;
    re.axis_origin_xyz  = { p["axis_origin_xyz"][0], p["axis_origin_xyz"][1],
                            p["axis_origin_xyz"][2] };
    re.axis_dir_xyz     = { p["axis_dir_xyz"][0], p["axis_dir_xyz"][1],
                            p["axis_dir_xyz"][2] };
    re.hole_dia_mm      = p.value("hole_dia_mm", 0.0);
    re.hole_depth_mm    = p.value("hole_depth_mm", 0.0);
    re.radial_offset_mm = p.value("radial_offset_mm", 0.0);
    re.count            = p.value("count", 0);
    re.total_angle_deg  = p.value("total_angle_deg", 360.0);
    auto fresh = skill::createCylindricalStock(80.0, 10.0);
    const double vFresh = volumeOf(fresh->shape());
    auto regen = skill::circular_pattern::apply(*fresh, re);
    EXPECT_NEAR(vFresh - volumeOf(regen.workpiece->shape()), vFresh - vBuilt,
                (vFresh - vBuilt) * 0.10)
        << "the recovered through ring must regenerate the same removed volume";
}

// ─── 8. SIDE bolt circle (axis +X, not Z): a radial ring of holes bored into a
// case SIDE must be recognised with the correct NON-Z axis and round-trip.  This
// is the axis-agnostic generalisation of the recogniser (was Z-only). ──────────
TEST(SkillCircularPattern, RecognizesSideBoltCircleNonZAxis)
{
    // A tall block; bore a 5-hole ring into its +X face, ring plane ⟂ X.
    auto stock = skill::createCuboidStock(40.0, 60.0, 60.0);  // X[0,40] Y[0,60] Z[0,60]
    skill::circular_pattern::Input in;
    in.axis_origin_xyz  = { 40.0, 30.0, 30.0 };   // +X face, ring centred on it
    in.axis_dir_xyz     = { 1.0, 0.0, 0.0 };      // holes bored along -X into the body
    in.hole_dia_mm      = 5.0;
    in.hole_depth_mm    = 6.0;
    in.radial_offset_mm = 18.0;
    in.count            = 5;
    in.total_angle_deg  = 360.0;
    auto built = skill::circular_pattern::apply(*stock, in);

    skill::Workpiece foreign(built.workpiece->shape());   // NO history → geometry path
    auto cands = skill::circular_pattern::recognize(foreign);
    ASSERT_GE(cands.size(), 1u) << "a side bolt circle must be recognised";
    const auto& p = cands[0].recovered_params;

    EXPECT_EQ(p.value("count", 0), 5) << "all 5 side holes counted";
    EXPECT_NEAR(p.value("radial_offset_mm", 0.0), 18.0, 1.0) << "ring radius recovered";
    EXPECT_NEAR(p.value("hole_depth_mm", 0.0), 6.0, 0.6)
        << "depth recovered along the hole axis, not the world-Z span";

    // The recovered axis must be ±X (NOT the old hard-coded +Z).
    ASSERT_TRUE(p.contains("axis_dir_xyz"));
    const auto& ax = p["axis_dir_xyz"];
    EXPECT_NEAR(std::abs(ax[0].get<double>()), 1.0, 1e-3) << "axis is ±X";
    EXPECT_NEAR(ax[1].get<double>(), 0.0, 1e-3);
    EXPECT_NEAR(ax[2].get<double>(), 0.0, 1e-3);

    // The axis origin must lie on the +X entry face (X≈40), not at X=0.
    ASSERT_TRUE(p.contains("axis_origin_xyz"));
    EXPECT_NEAR(p["axis_origin_xyz"][0].get<double>(), 40.0, 0.6)
        << "axis origin X must be the real entry plane";

    // hole_centers must be full 3-D points (the dedupe/CAM consumers read x,y,z).
    ASSERT_TRUE(p.contains("hole_centers"));
    ASSERT_EQ(p["hole_centers"].size(), 5u);
    EXPECT_EQ(p["hole_centers"][0].size(), 3u) << "3-D centres for a non-Z ring";
}

// ─── 8b. SIDE bolt circle round-trips by removed volume through recognize→apply
// (the recovered non-Z axis must regenerate the same holes, in the same place). ─
TEST(SkillCircularPattern, SideBoltCircleRoundTripsByVolume)
{
    auto stock = skill::createCuboidStock(40.0, 60.0, 60.0);
    skill::circular_pattern::Input in;
    in.axis_origin_xyz  = { 40.0, 30.0, 30.0 };
    in.axis_dir_xyz     = { 1.0, 0.0, 0.0 };
    in.hole_dia_mm      = 5.0;
    in.hole_depth_mm    = 6.0;
    in.radial_offset_mm = 18.0;
    in.count            = 5;
    in.total_angle_deg  = 360.0;
    auto built = skill::circular_pattern::apply(*stock, in);
    const double vBuilt = volumeOf(built.workpiece->shape());

    skill::Workpiece foreign(built.workpiece->shape());
    auto cands = skill::circular_pattern::recognize(foreign);
    ASSERT_GE(cands.size(), 1u);

    // Regenerate from the recovered params on FRESH stock → same removed volume.
    skill::circular_pattern::Input re;
    const auto& p = cands[0].recovered_params;
    re.axis_origin_xyz  = { p["axis_origin_xyz"][0], p["axis_origin_xyz"][1],
                            p["axis_origin_xyz"][2] };
    re.axis_dir_xyz     = { p["axis_dir_xyz"][0], p["axis_dir_xyz"][1],
                            p["axis_dir_xyz"][2] };
    re.hole_dia_mm      = p.value("hole_dia_mm", 0.0);
    re.hole_depth_mm    = p.value("hole_depth_mm", 0.0);
    re.radial_offset_mm = p.value("radial_offset_mm", 0.0);
    re.count            = p.value("count", 0);
    re.total_angle_deg  = p.value("total_angle_deg", 360.0);
    auto fresh = skill::createCuboidStock(40.0, 60.0, 60.0);
    auto regen = skill::circular_pattern::apply(*fresh, re);

    const double vFresh = volumeOf(fresh->shape());
    EXPECT_NEAR(vFresh - volumeOf(regen.workpiece->shape()), vFresh - vBuilt,
                (vFresh - vBuilt) * 0.10)
        << "the recovered side bolt circle regenerates the same removed volume";
}
