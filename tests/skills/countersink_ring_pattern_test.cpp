// @lat: [[process/test-strategy#skill round-trip]]
//
// countersink_ring_pattern — a ring of N identical countersinks (flat-head
// screw seat ring).  This mirrors the counterbore ring's measured worst case:
// the grammar recovered the cone ring and pilot ring as separate
// bolt_circle_pattern steps whose replay left every seat cone missing
// (~-43.7 % removed-volume error) while dedupe blocked the individual
// countersinks.  These tests pin the fix: one compound step that owns the
// whole ring and replays at full volume.

#include <gtest/gtest.h>

#include "cam/ToolpathPlan.hpp"
#include "process/Executor.hpp"
#include "process/ProcessPlan.hpp"
#include "re/Recognizer.hpp"
#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/countersink.hpp"
#include "skills/countersink_ring_pattern.hpp"
#include "skills/drill_hole.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <algorithm>
#include <cmath>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::countersink_ring_pattern::Input ringInput()
{
    skill::countersink_ring_pattern::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.count              = 6;
    in.bolt_circle_dia_mm = 60.0;     // R30
    in.center_x_mm        = 50.0;
    in.center_y_mm        = 50.0;
    in.axis_dir           = gp_Dir(0, 0, -1);
    in.pilot_dia_mm       = 5.0;
    in.pilot_depth_mm     = 10.0;
    in.cone_top_dia_mm    = 10.0;
    in.cone_angle_deg     = 90.0;     // metric flat-head (ISO 7721)
    return in;
}

// Analytic removed volume of ONE countersink, matching countersink::apply's
// cutter construction exactly: a pilot cylinder (d5 to depth 10 below the
// entry plane) FUSED with a cone frustum from the entry plane (r 5 → 2.5 over
// cone_depth = (10 − 5)/2 / tan(90°/2) = 2.5).  The fused union is
//   cylinder + frustum − overlap,
// and the overlap is the pilot core inside the cone span (π·r_p²·cone_depth):
//   π·2.5²·10 + (π·2.5/3)(5² + 5·2.5 + 2.5²) − π·2.5²·2.5 ≈ 261.80 mm³
double oneCountersinkVolume()
{
    const double pilotR    = 2.5;
    const double coneTopR  = 5.0;
    const double pilotDep  = 10.0;
    const double coneDep   = 2.5;    // (10 − 5)/2 / tan(45°)
    const double cylVol    = M_PI * pilotR * pilotR * pilotDep;
    const double frustVol  = (M_PI * coneDep / 3.0) *
        (coneTopR * coneTopR + coneTopR * pilotR + pilotR * pilotR);
    const double coneCore  = M_PI * pilotR * pilotR * coneDep;
    return cylVol + frustVol - coneCore;
}
}  // namespace

// ─── 1. apply removes the full cone-plus-pilot ring volume ───────────────────
TEST(SkillCountersinkRingPattern, ApplyRemovesRingVolume)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    const double vStock = volumeOf(stock->shape());

    auto out = skill::countersink_ring_pattern::apply(*stock, ringInput());
    ASSERT_NE(out.workpiece, nullptr);
    const double removed = vStock - volumeOf(out.workpiece->shape());

    // 6 x (pilot d5 x 10 + cone annulus above it) = 6 x 261.80 ≈ 1570.8
    const double expect = 6.0 * oneCountersinkVolume();
    EXPECT_NEAR(removed, expect, expect * 0.02)
        << "the ring must remove the FULL countersink volume (cone + pilot below)";
}

// ─── 2. THE regression: a foreign countersunk ring must reach the plan as ONE
// compound step (not cone/pilot bolt-circles + blocked countersinks) and
// replay at full volume (the ~-43.7 % cone-short error fixed). ────────────────
TEST(SkillCountersinkRingPattern, ForeignRingReplaysAsOneStepFullVolume)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    auto built = skill::countersink_ring_pattern::apply(*stock, ringInput());
    const double vBuilt = volumeOf(built.workpiece->shape());

    skill::Workpiece foreign(built.workpiece->shape());   // NO history
    const process::ProcessPlan plan = re::inferProcessPlan(foreign, 0.7);

    int ringSteps = 0, plainRings = 0, looseCsinks = 0, looseHoles = 0;
    for (const auto& s : plan.steps()) {
        if (s.skill_id == "countersink_ring_pattern") ++ringSteps;
        if (s.skill_id == "bolt_circle_pattern" || s.skill_id == "circular_pattern")
            ++plainRings;
        if (s.skill_id == "countersink") ++looseCsinks;
        if (s.skill_id == "drill_hole" || s.skill_id == "drill_through_hole" ||
            s.skill_id == "bore_cylindrical" || s.skill_id == "mill_circular_pocket")
            ++looseHoles;
    }
    EXPECT_EQ(ringSteps, 1)  << "exactly one countersink-ring compound step";
    EXPECT_EQ(plainRings, 0) << "the cone/pilot bolt-circle duplicates are subsumed";
    EXPECT_EQ(looseCsinks, 0) << "member countersinks are subsumed by the ring";
    EXPECT_EQ(looseHoles, 0)  << "no loose hole atoms leak";

    // Replay on fresh stock: the FULL volume regenerates (was ~-43.7 % short).
    process::ProcessPlan replayable;
    for (const auto& step : plan.steps()) replayable.append(re::liftRecoveredStep(step));
    auto fresh = skill::createCuboidStock(100.0, 100.0, 20.0);
    const double vFresh = volumeOf(fresh->shape());
    auto result = process::Executor::execute(replayable, fresh, process::ExecuteOptions{true});
    ASSERT_NE(result.workpiece, nullptr);
    ASSERT_FALSE(result.workpiece->shape().IsNull());
    EXPECT_NEAR(vFresh - volumeOf(result.workpiece->shape()), vFresh - vBuilt,
                (vFresh - vBuilt) * 0.05)
        << "the recovered ring must regenerate the FULL removed volume";
}

// ─── 3. LOOSE-fit ring (±0.2 mm centre jitter — realistic foreign/rebuilt STEP
// noise): the circle fit tolerates the residual and still emits ONE ring. ────
TEST(SkillCountersinkRingPattern, LooseFitRingStillOneStep)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    // Build 6 INDIVIDUAL countersinks at jittered ring positions.
    const double jit[6][2] = { { 0.2, 0.0 }, { -0.15, 0.1 }, { 0.0, -0.2 },
                               { 0.1, 0.15 }, { -0.2, -0.1 }, { 0.15, -0.05 } };
    std::shared_ptr<skill::Workpiece> wp = stock;
    for (int i = 0; i < 6; ++i) {
        const double ang = i * 60.0 * M_PI / 180.0;
        skill::countersink::Input cin;
        cin.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        cin.position_x_mm   = 50.0 + 30.0 * std::cos(ang) + jit[i][0];
        cin.position_y_mm   = 50.0 + 30.0 * std::sin(ang) + jit[i][1];
        cin.axis_dir        = gp_Dir(0, 0, -1);
        cin.pilot_dia_mm    = 5.0;
        cin.pilot_depth_mm  = 10.0;
        cin.cone_top_dia_mm = 10.0;
        cin.cone_angle_deg  = 90.0;
        wp = skill::countersink::apply(*wp, cin).workpiece;
    }
    const double vBuilt = volumeOf(wp->shape());

    skill::Workpiece foreign(wp->shape());
    const process::ProcessPlan plan = re::inferProcessPlan(foreign, 0.7);
    int ringSteps = 0;
    for (const auto& s : plan.steps())
        if (s.skill_id == "countersink_ring_pattern") ++ringSteps;
    EXPECT_EQ(ringSteps, 1)
        << "a jittered (loose-fit) ring must still collapse to one compound step";

    // Volume parity within a jitter-sized tolerance.
    process::ProcessPlan replayable;
    for (const auto& step : plan.steps()) replayable.append(re::liftRecoveredStep(step));
    auto fresh = skill::createCuboidStock(100.0, 100.0, 20.0);
    const double vFresh = volumeOf(fresh->shape());
    auto result = process::Executor::execute(replayable, fresh, process::ExecuteOptions{true});
    ASSERT_NE(result.workpiece, nullptr);
    EXPECT_NEAR(vFresh - volumeOf(result.workpiece->shape()), vFresh - vBuilt,
                (vFresh - vBuilt) * 0.05)
        << "the loose-fit ring must still regenerate the full removed volume";
}

// ─── 4. NO false ring on plain drills: a 6-hole plain bolt circle carries no
// countersink atoms, so no countersink_ring_pattern may appear. ──────────────
TEST(SkillCountersinkRingPattern, NoFalseRingOnPlainDrillCircle)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    std::shared_ptr<skill::Workpiece> wp = stock;
    for (int i = 0; i < 6; ++i) {
        const double ang = i * 60.0 * M_PI / 180.0;
        skill::drill_hole::Input h;
        h.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        h.position_x_mm = 50.0 + 30.0 * std::cos(ang);
        h.position_y_mm = 50.0 + 30.0 * std::sin(ang);
        h.axis_dir      = gp_Dir(0, 0, -1);
        h.diameter_mm   = 5.0;
        h.depth_mm      = 10.0;
        wp = skill::drill_hole::apply(*wp, h).workpiece;
    }
    skill::Workpiece foreign(wp->shape());
    const process::ProcessPlan plan = re::inferProcessPlan(foreign, 0.7);
    for (const auto& s : plan.steps())
        EXPECT_NE(s.skill_id, std::string("countersink_ring_pattern"))
            << "plain drills must never be read as a countersunk ring";
}

// ─── 5. DFM validation rejects malformed input ───────────────────────────────
TEST(SkillCountersinkRingPattern, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);

    auto bad = ringInput(); bad.count = 2;                 // < 3 instances
    EXPECT_FALSE(skill::countersink_ring_pattern::validate(*stock, bad).passed);

    bad = ringInput(); bad.cone_top_dia_mm = 4.0;          // cone top <= pilot
    EXPECT_FALSE(skill::countersink_ring_pattern::validate(*stock, bad).passed);

    bad = ringInput(); bad.cone_angle_deg = 30.0;          // outside [45, 120]
    EXPECT_FALSE(skill::countersink_ring_pattern::validate(*stock, bad).passed);

    bad = ringInput(); bad.axis_dir = gp_Dir(1, 0, 0);     // non-Z axis
    EXPECT_FALSE(skill::countersink_ring_pattern::validate(*stock, bad).passed);
}

// ─── 6. UNEVEN angles on one circle are NOT a ring: three concyclic
// countersinks at 0°/90°/180° must be refused by the angular-evenness gate
// (any 3 points are concyclic — replaying them at uniform 120° spacing would
// cut holes where none exist).  The member countersinks survive as the correct
// explanation instead. ───────────────────────────────────────────────────────
TEST(SkillCountersinkRingPattern, UnevenRingIsNotAPattern)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    const double angles_deg[3] = { 0.0, 90.0, 180.0 };   // uneven spacing
    std::shared_ptr<skill::Workpiece> wp = stock;
    for (int i = 0; i < 3; ++i) {
        const double ang = angles_deg[i] * M_PI / 180.0;
        skill::countersink::Input cin;
        cin.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        cin.position_x_mm   = 50.0 + 30.0 * std::cos(ang);
        cin.position_y_mm   = 50.0 + 30.0 * std::sin(ang);
        cin.axis_dir        = gp_Dir(0, 0, -1);
        cin.pilot_dia_mm    = 5.0;
        cin.pilot_depth_mm  = 10.0;
        cin.cone_top_dia_mm = 10.0;
        cin.cone_angle_deg  = 90.0;
        wp = skill::countersink::apply(*wp, cin).workpiece;
    }

    skill::Workpiece foreign(wp->shape());
    const process::ProcessPlan plan = re::inferProcessPlan(foreign, 0.7);

    int ringSteps = 0, looseCsinks = 0;
    for (const auto& s : plan.steps()) {
        if (s.skill_id == "countersink_ring_pattern") ++ringSteps;
        if (s.skill_id == "countersink") ++looseCsinks;
    }
    EXPECT_EQ(ringSteps, 0)
        << "uneven concyclic countersinks must NOT be forced into a uniform ring";
    EXPECT_GE(looseCsinks, 1)
        << "the individual countersinks must survive as the explanation";
}

// ─── 7. TWO rings sharing one entry face coexist: the shared top entry-plane
// face id must not sit in a ring's dedupe union, or the first ring blocks the
// second (strict-superset arbitration would eat it). ─────────────────────────
TEST(SkillCountersinkRingPattern, TwoRingsOnOneFaceCoexist)
{
    auto stock = skill::createCuboidStock(140.0, 140.0, 20.0);

    auto inA = ringInput();
    inA.bolt_circle_dia_mm = 50.0;
    inA.center_x_mm        = 45.0;
    inA.center_y_mm        = 45.0;
    auto inB = inA;
    inB.center_x_mm = 95.0;
    inB.center_y_mm = 95.0;
    // Outermost cone-top extent = 25 + 5 = 30 from each centre; centres are
    // 50·√2 ≈ 70.7 apart, so the rings are well clear of each other.

    auto midway = skill::countersink_ring_pattern::apply(*stock, inA);
    ASSERT_NE(midway.workpiece, nullptr);
    auto built = skill::countersink_ring_pattern::apply(*midway.workpiece, inB);
    ASSERT_NE(built.workpiece, nullptr);
    const double vBuilt = volumeOf(built.workpiece->shape());

    skill::Workpiece foreign(built.workpiece->shape());
    const process::ProcessPlan plan = re::inferProcessPlan(foreign, 0.7);

    int ringSteps = 0;
    for (const auto& s : plan.steps())
        if (s.skill_id == "countersink_ring_pattern") ++ringSteps;
    EXPECT_EQ(ringSteps, 2)
        << "two rings on one shared entry face must BOTH reach the plan";

    // Replay both on fresh stock: total removed volume parity.
    process::ProcessPlan replayable;
    for (const auto& step : plan.steps()) replayable.append(re::liftRecoveredStep(step));
    auto fresh = skill::createCuboidStock(140.0, 140.0, 20.0);
    const double vFresh = volumeOf(fresh->shape());
    auto result = process::Executor::execute(replayable, fresh, process::ExecuteOptions{true});
    ASSERT_NE(result.workpiece, nullptr);
    ASSERT_FALSE(result.workpiece->shape().IsNull());
    EXPECT_NEAR(vFresh - volumeOf(result.workpiece->shape()), vFresh - vBuilt,
                (vFresh - vBuilt) * 0.05)
        << "both rings must regenerate their full removed volume";
}

// ─── 8. TIGHT pitch circle: PCD only just above the cone-top diameter (12.5 vs
// 10).  validate allows any PCD whose adjacent cones clear (chord 12.5·sin60° ≈
// 10.83 > 10), but a rr < coneTopDia-style gate would silently reject
// PCD < 2×cone_top — the recovery gate must match validate. ──────────────────
TEST(SkillCountersinkRingPattern, TightPitchCircleStillRecovered)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    auto in = ringInput();
    in.count              = 3;
    in.bolt_circle_dia_mm = 12.5;   // R6.25 — barely above cone top dia 10
    auto built = skill::countersink_ring_pattern::apply(*stock, in);
    ASSERT_NE(built.workpiece, nullptr);

    skill::Workpiece foreign(built.workpiece->shape());
    const process::ProcessPlan plan = re::inferProcessPlan(foreign, 0.7);

    int ringSteps = 0;
    for (const auto& s : plan.steps())
        if (s.skill_id == "countersink_ring_pattern") ++ringSteps;
    EXPECT_EQ(ringSteps, 1)
        << "a tight-PCD ring the skill can BUILD must also be RECOVERED";
}

// ─── 9. DFM validation rejects a sub-manufacturable pilot (DFM-002 mirror):
// pilot d0.5 must fail validate up front, not throw inside apply. ────────────
TEST(SkillCountersinkRingPattern, ValidateRejectsSubManufacturablePilot)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    auto bad = ringInput();
    bad.pilot_dia_mm = 0.5;
    EXPECT_FALSE(skill::countersink_ring_pattern::validate(*stock, bad).passed)
        << "a sub-manufacturable pilot must fail validate, not throw in apply";
}

// ─── 10. A REPLAYED ring is machinable: the compound signature stamped by the
// Executor must yield a real CAM toolpath (segments, both plunge depths), not
// the dispatcher's empty-toolpath-plus-warning fallback. ─────────────────────
TEST(SkillCountersinkRingPattern, ReplayedRingProducesToolpaths)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    auto built = skill::countersink_ring_pattern::apply(*stock, ringInput());
    ASSERT_NE(built.workpiece, nullptr);

    skill::Workpiece foreign(built.workpiece->shape());
    const process::ProcessPlan plan = re::inferProcessPlan(foreign, 0.7);

    process::ProcessPlan replayable;
    for (const auto& step : plan.steps()) replayable.append(re::liftRecoveredStep(step));
    auto fresh = skill::createCuboidStock(100.0, 100.0, 20.0);
    auto result = process::Executor::execute(replayable, fresh, process::ExecuteOptions{true});
    ASSERT_NE(result.workpiece, nullptr);
    ASSERT_FALSE(result.workpiece->shape().IsNull());

    const cam::ToolpathReport report = cam::planAndVerify(*result.workpiece);
    const cam::Toolpath* ringTp = nullptr;
    for (const auto& tp : report.toolpaths)
        if (tp.feature_skill_id == "countersink_ring_pattern" && !tp.segments.empty())
            ringTp = &tp;
    ASSERT_NE(ringTp, nullptr)
        << "the replayed ring signature must produce a non-empty toolpath";

    // Stock spans Z in [0, 20] → the entry plane is Z = 20.  Assert BOTH
    // plunges at their EXACT tip depths (a min-Z check alone is subsumed by
    // the deeper pilot and could not catch a chamfer-depth/tan error):
    //   pilot tip    z = 20 - pilot_depth(10)                         = 10.0
    //   chamfer TIP  z = 20 - [coneDep(2.5) + (pilot/2)/tan(45°)(2.5)] = 15.0
    //     (tip-datum: a pointed countersink's tip travels past the frustum
    //      height so the mouth opens to the full cone_top diameter)
    bool sawPilotTip = false, sawChamferTip = false;
    double zLo = 1e9;
    for (const auto& s : ringTp->segments) {
        zLo = std::min(zLo, s.end_point.Z());
        if (s.move == cam::PathSegment::Move::Linear) {
            if (std::abs(s.end_point.Z() - 10.0) < 0.2) sawPilotTip = true;
            if (std::abs(s.end_point.Z() - 15.0) < 0.2) sawChamferTip = true;
        }
    }
    EXPECT_TRUE(sawPilotTip)
        << "a feed plunge must land at the pilot tip Z=10";
    EXPECT_TRUE(sawChamferTip)
        << "a feed plunge must land at the pointed-chamfer TIP Z=15 "
           "(coneDep + (pilot/2)/tan(half)) — not the frustum height Z=17.5";
    EXPECT_LT(zLo, 20.0 - 9.9)
        << "the pilot plunge must reach full depth below the entry plane";

    // And the G-code carries a real feed (plunge) move for the ring.
    const std::string g = cam::toGCode(*ringTp);
    EXPECT_NE(g.find("G1"), std::string::npos)
        << "the ring toolpath G-code must contain a feed (plunge) move";
}
