// @lat: [[process/test-strategy#skill round-trip]]
//
// counterbore_ring_pattern — a ring of N identical counterbores (screw-head
// seat ring).  This was the RE pipeline's measured worst case: the grammar
// recovered the pilot ring and seat ring as TWO bolt_circle_pattern steps whose
// replay left every pilot seat-depth short (~18 % removed-volume error) while
// dedupe blocked the individual counterbores.  These tests pin the fix: one
// compound step that owns the whole ring and replays at full volume.

#include <gtest/gtest.h>

#include "cam/ToolpathPlan.hpp"
#include "process/Executor.hpp"
#include "process/ProcessPlan.hpp"
#include "re/Recognizer.hpp"
#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/counterbore.hpp"
#include "skills/counterbore_ring_pattern.hpp"
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

skill::counterbore_ring_pattern::Input ringInput()
{
    skill::counterbore_ring_pattern::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.count              = 6;
    in.bolt_circle_dia_mm = 60.0;     // R30
    in.center_x_mm        = 50.0;
    in.center_y_mm        = 50.0;
    in.axis_dir           = gp_Dir(0, 0, -1);
    in.pilot_dia_mm       = 5.0;
    in.pilot_depth_mm     = 10.0;
    in.seat_dia_mm        = 9.0;
    in.seat_depth_mm      = 3.0;
    return in;
}
}  // namespace

// ─── 1. apply removes the full two-diameter ring volume ─────────────────────
TEST(SkillCounterboreRingPattern, ApplyRemovesRingVolume)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    const double vStock = volumeOf(stock->shape());

    auto out = skill::counterbore_ring_pattern::apply(*stock, ringInput());
    ASSERT_NE(out.workpiece, nullptr);
    const double removed = vStock - volumeOf(out.workpiece->shape());

    // 6 x (seat d9 x 3  +  pilot d5 x (10-3)) = 6 x (190.85 + 137.44) ≈ 1969.8
    const double expect = 6.0 * (M_PI * 4.5 * 4.5 * 3.0 + M_PI * 2.5 * 2.5 * 7.0);
    EXPECT_NEAR(removed, expect, expect * 0.02)
        << "the ring must remove the FULL counterbore volume (seat + pilot below)";
}

// ─── 2. THE regression: a foreign counterbored ring must reach the plan as ONE
// compound step (not two seat/pilot bolt-circles + blocked counterbores) and
// replay at full volume (the ~18 % pilot-short error fixed). ─────────────────
TEST(SkillCounterboreRingPattern, ForeignRingReplaysAsOneStepFullVolume)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    auto built = skill::counterbore_ring_pattern::apply(*stock, ringInput());
    const double vBuilt = volumeOf(built.workpiece->shape());

    skill::Workpiece foreign(built.workpiece->shape());   // NO history
    const process::ProcessPlan plan = re::inferProcessPlan(foreign, 0.7);

    int ringSteps = 0, plainRings = 0, looseCbores = 0, looseHoles = 0;
    for (const auto& s : plan.steps()) {
        if (s.skill_id == "counterbore_ring_pattern") ++ringSteps;
        if (s.skill_id == "bolt_circle_pattern" || s.skill_id == "circular_pattern")
            ++plainRings;
        if (s.skill_id == "counterbore") ++looseCbores;
        if (s.skill_id == "drill_hole" || s.skill_id == "drill_through_hole" ||
            s.skill_id == "bore_cylindrical" || s.skill_id == "mill_circular_pocket")
            ++looseHoles;
    }
    EXPECT_EQ(ringSteps, 1)  << "exactly one counterbore-ring compound step";
    EXPECT_EQ(plainRings, 0) << "the seat/pilot bolt-circle duplicates are subsumed";
    EXPECT_EQ(looseCbores, 0) << "member counterbores are subsumed by the ring";
    EXPECT_EQ(looseHoles, 0)  << "no loose hole atoms leak";

    // Replay on fresh stock: the FULL volume regenerates (was ~18 % short).
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
TEST(SkillCounterboreRingPattern, LooseFitRingStillOneStep)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    // Build 6 INDIVIDUAL counterbores at jittered ring positions.
    const double jit[6][2] = { { 0.2, 0.0 }, { -0.15, 0.1 }, { 0.0, -0.2 },
                               { 0.1, 0.15 }, { -0.2, -0.1 }, { 0.15, -0.05 } };
    std::shared_ptr<skill::Workpiece> wp = stock;
    for (int i = 0; i < 6; ++i) {
        const double ang = i * 60.0 * M_PI / 180.0;
        skill::counterbore::Input cin;
        cin.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        cin.position_x_mm  = 50.0 + 30.0 * std::cos(ang) + jit[i][0];
        cin.position_y_mm  = 50.0 + 30.0 * std::sin(ang) + jit[i][1];
        cin.axis_dir       = gp_Dir(0, 0, -1);
        cin.pilot_dia_mm   = 5.0;
        cin.pilot_depth_mm = 10.0;
        cin.seat_dia_mm    = 9.0;
        cin.seat_depth_mm  = 3.0;
        wp = skill::counterbore::apply(*wp, cin).workpiece;
    }
    const double vBuilt = volumeOf(wp->shape());

    skill::Workpiece foreign(wp->shape());
    const process::ProcessPlan plan = re::inferProcessPlan(foreign, 0.7);
    int ringSteps = 0;
    for (const auto& s : plan.steps())
        if (s.skill_id == "counterbore_ring_pattern") ++ringSteps;
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
// counterbore atoms, so no counterbore_ring_pattern may appear. ──────────────
TEST(SkillCounterboreRingPattern, NoFalseRingOnPlainDrillCircle)
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
        EXPECT_NE(s.skill_id, std::string("counterbore_ring_pattern"))
            << "plain drills must never be read as a counterbored ring";
}

// ─── 5. DFM validation rejects malformed input ───────────────────────────────
TEST(SkillCounterboreRingPattern, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);

    auto bad = ringInput(); bad.count = 2;                 // < 3 instances
    EXPECT_FALSE(skill::counterbore_ring_pattern::validate(*stock, bad).passed);

    bad = ringInput(); bad.seat_dia_mm = 4.0;              // seat <= pilot
    EXPECT_FALSE(skill::counterbore_ring_pattern::validate(*stock, bad).passed);

    bad = ringInput(); bad.seat_depth_mm = 12.0;           // seat deeper than pilot
    EXPECT_FALSE(skill::counterbore_ring_pattern::validate(*stock, bad).passed);

    bad = ringInput(); bad.axis_dir = gp_Dir(1, 0, 0);     // non-Z axis
    EXPECT_FALSE(skill::counterbore_ring_pattern::validate(*stock, bad).passed);
}

// ─── 6. UNEVEN angles on one circle are NOT a ring: three concyclic
// counterbores at 0°/90°/180° must be refused by the angular-evenness gate
// (any 3 points are concyclic — replaying them at uniform 120° spacing would
// cut holes where none exist).  The member counterbores survive as the correct
// explanation instead. ───────────────────────────────────────────────────────
TEST(SkillCounterboreRingPattern, UnevenRingIsNotAPattern)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    const double angles_deg[3] = { 0.0, 90.0, 180.0 };   // uneven spacing
    std::shared_ptr<skill::Workpiece> wp = stock;
    for (int i = 0; i < 3; ++i) {
        const double ang = angles_deg[i] * M_PI / 180.0;
        skill::counterbore::Input cin;
        cin.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        cin.position_x_mm  = 50.0 + 30.0 * std::cos(ang);
        cin.position_y_mm  = 50.0 + 30.0 * std::sin(ang);
        cin.axis_dir       = gp_Dir(0, 0, -1);
        cin.pilot_dia_mm   = 5.0;
        cin.pilot_depth_mm = 10.0;
        cin.seat_dia_mm    = 9.0;
        cin.seat_depth_mm  = 3.0;
        wp = skill::counterbore::apply(*wp, cin).workpiece;
    }

    skill::Workpiece foreign(wp->shape());
    const process::ProcessPlan plan = re::inferProcessPlan(foreign, 0.7);

    int ringSteps = 0, looseCbores = 0;
    for (const auto& s : plan.steps()) {
        if (s.skill_id == "counterbore_ring_pattern") ++ringSteps;
        if (s.skill_id == "counterbore") ++looseCbores;
    }
    EXPECT_EQ(ringSteps, 0)
        << "uneven concyclic counterbores must NOT be forced into a uniform ring";
    EXPECT_GE(looseCbores, 1)
        << "the individual counterbores must survive as the explanation";
}

// ─── 7. TWO rings sharing one entry face coexist: the shared top entry-plane
// face id must not sit in a ring's dedupe union, or the first ring blocks the
// second (strict-superset arbitration would eat it). ─────────────────────────
TEST(SkillCounterboreRingPattern, TwoRingsOnOneFaceCoexist)
{
    auto stock = skill::createCuboidStock(140.0, 140.0, 20.0);

    auto inA = ringInput();
    inA.bolt_circle_dia_mm = 50.0;
    inA.center_x_mm        = 45.0;
    inA.center_y_mm        = 45.0;
    auto inB = inA;
    inB.center_x_mm = 95.0;
    inB.center_y_mm = 95.0;
    // Outermost seat extent = 25 + 4.5 = 29.5 from each centre; centres are
    // 50·√2 ≈ 70.7 apart, so the rings are well clear of each other.

    auto midway = skill::counterbore_ring_pattern::apply(*stock, inA);
    ASSERT_NE(midway.workpiece, nullptr);
    auto built = skill::counterbore_ring_pattern::apply(*midway.workpiece, inB);
    ASSERT_NE(built.workpiece, nullptr);
    const double vBuilt = volumeOf(built.workpiece->shape());

    skill::Workpiece foreign(built.workpiece->shape());
    const process::ProcessPlan plan = re::inferProcessPlan(foreign, 0.7);

    int ringSteps = 0;
    for (const auto& s : plan.steps())
        if (s.skill_id == "counterbore_ring_pattern") ++ringSteps;
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

// ─── 8. TIGHT pitch circle: PCD only just above the seat diameter (12 vs 9).
// validate allows any PCD whose adjacent seats clear (chord 12·sin60° ≈ 10.39 >
// 9), but the grammar's old rr < seatDia gate silently rejected PCD < 2×seat —
// the recovery gate must match validate. ─────────────────────────────────────
TEST(SkillCounterboreRingPattern, TightPitchCircleStillRecovered)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    auto in = ringInput();
    in.count              = 3;
    in.bolt_circle_dia_mm = 12.0;   // R6 — barely above seat dia 9
    auto built = skill::counterbore_ring_pattern::apply(*stock, in);
    ASSERT_NE(built.workpiece, nullptr);

    skill::Workpiece foreign(built.workpiece->shape());
    const process::ProcessPlan plan = re::inferProcessPlan(foreign, 0.7);

    int ringSteps = 0;
    for (const auto& s : plan.steps())
        if (s.skill_id == "counterbore_ring_pattern") ++ringSteps;
    EXPECT_EQ(ringSteps, 1)
        << "a tight-PCD ring the skill can BUILD must also be RECOVERED";
}

// ─── 9. DFM validation rejects a sub-manufacturable pilot (DFM-002 mirror):
// pilot d0.5 previously passed validate but threw inside apply. ──────────────
TEST(SkillCounterboreRingPattern, ValidateRejectsSubManufacturablePilot)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    auto bad = ringInput();
    bad.pilot_dia_mm = 0.5;
    EXPECT_FALSE(skill::counterbore_ring_pattern::validate(*stock, bad).passed)
        << "a sub-manufacturable pilot must fail validate, not throw in apply";
}

// ─── 10. A REPLAYED ring is machinable: the compound signature stamped by the
// Executor must yield a real CAM toolpath (segments, both plunge depths), not
// the dispatcher's empty-toolpath-plus-warning fallback. ─────────────────────
TEST(SkillCounterboreRingPattern, ReplayedRingProducesToolpaths)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    auto built = skill::counterbore_ring_pattern::apply(*stock, ringInput());
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
        if (tp.feature_skill_id == "counterbore_ring_pattern" && !tp.segments.empty())
            ringTp = &tp;
    ASSERT_NE(ringTp, nullptr)
        << "the replayed ring signature must produce a non-empty toolpath";

    // Stock spans Z in [0, 20] → the entry plane is Z = 20.  The path must
    // plunge through the seat depth (3) AND the pilot depth (10) below it.
    double zLo = 1e9;
    for (const auto& s : ringTp->segments) zLo = std::min(zLo, s.end_point.Z());
    EXPECT_LT(zLo, 20.0 - 2.9)
        << "the seat plunge must descend below the entry plane (Z~17)";
    EXPECT_LT(zLo, 20.0 - 9.9)
        << "the pilot plunge must reach full depth below the entry plane (Z~10)";

    // And the G-code carries a real feed (plunge) move for the ring.
    const std::string g = cam::toGCode(*ringTp);
    EXPECT_NE(g.find("G1"), std::string::npos)
        << "the ring toolpath G-code must contain a feed (plunge) move";
}
