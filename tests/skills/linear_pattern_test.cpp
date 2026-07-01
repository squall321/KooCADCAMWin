// @lat: [[process/test-strategy#skill round-trip]]
//
// linear_pattern — count_x × count_y grid of cylindrical cuts.
//
// Cases (5):
//   1. 3x1 pattern removes ~3 × π r² × depth volume on a flat plate.
//   2. 2x2 grid removes ~4 × per-hole volume.
//   3. DFM rejects pitch ≤ hole_dia.
//   4. DFM rejects count_x = 0.
//   5. recognize finds the pattern (history replay).

#include <gtest/gtest.h>

#include "re/Recognizer.hpp"
#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_hole.hpp"
#include "skills/linear_pattern.hpp"

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

// ─── 1. 3-hole linear row volume ───────────────────────────────────────────
TEST(SkillLinearPattern, ThreeInRowVolume)
{
    auto stock = skill::createCuboidStock(100.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::linear_pattern::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.hole_dia_mm   = 4.0;
    in.hole_depth_mm = 5.0;
    in.count_x       = 3;
    in.pitch_x_mm    = 15.0;
    in.count_y       = 1;
    in.pitch_y_mm    = 0.0;
    in.start_x_mm    = -15.0;        // -15, 0, +15 from face center
    in.start_y_mm    = 0.0;

    auto out = skill::linear_pattern::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());
    const double removed  = volBefore - volAfter;
    const double per      = M_PI * 2.0 * 2.0 * 5.0;
    const double expected = 3.0 * per;
    EXPECT_NEAR(removed, expected, expected * 0.15)
        << "3 × π r² × depth should match within 15%";

    EXPECT_EQ(out.signature.pattern["instance_count"].get<int>(), 3);
    EXPECT_TRUE(out.signature.pattern["is_pattern"].get<bool>());
}

// ─── 2. 2x2 grid volume ────────────────────────────────────────────────────
TEST(SkillLinearPattern, TwoByTwoGridVolume)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::linear_pattern::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.hole_dia_mm   = 3.0;
    in.hole_depth_mm = 4.0;
    in.count_x       = 2;
    in.pitch_x_mm    = 20.0;
    in.count_y       = 2;
    in.pitch_y_mm    = 20.0;
    in.start_x_mm    = -10.0;
    in.start_y_mm    = -10.0;

    auto out = skill::linear_pattern::apply(*stock, in);
    const double removed  = volBefore - volumeOf(out.workpiece->shape());
    const double per      = M_PI * 1.5 * 1.5 * 4.0;
    const double expected = 4.0 * per;
    EXPECT_NEAR(removed, expected, expected * 0.15);

    EXPECT_EQ(out.signature.pattern["instance_count"].get<int>(), 4);
}

// ─── 3. DFM rejects overlapping pitch ──────────────────────────────────────
TEST(SkillLinearPattern, ValidateRejectsPitchTooSmall)
{
    auto stock = skill::createCuboidStock(100.0, 50.0, 10.0);
    skill::linear_pattern::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.hole_dia_mm   = 5.0;
    in.hole_depth_mm = 4.0;
    in.count_x       = 3;
    in.pitch_x_mm    = 4.0;          // <= dia — overlap
    auto r = skill::linear_pattern::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings) if (f.code == "DFM-PATT-1") found = true;
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::linear_pattern::apply(*stock, in), skill::SkillError);
}

// ─── 4. DFM rejects count_x = 0 ────────────────────────────────────────────
TEST(SkillLinearPattern, ValidateRejectsZeroCount)
{
    auto stock = skill::createCuboidStock(100.0, 50.0, 10.0);
    skill::linear_pattern::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.hole_dia_mm   = 3.0;
    in.hole_depth_mm = 4.0;
    in.count_x       = 0;
    in.pitch_x_mm    = 15.0;
    auto r = skill::linear_pattern::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 5. Recognize finds the pattern (history replay) ───────────────────────
TEST(SkillLinearPattern, RecognizeFindsPattern)
{
    auto stock = skill::createCuboidStock(100.0, 50.0, 10.0);
    skill::linear_pattern::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.hole_dia_mm   = 4.0;
    in.hole_depth_mm = 5.0;
    in.count_x       = 3;
    in.pitch_x_mm    = 15.0;
    in.start_x_mm    = -15.0;

    auto out = skill::linear_pattern::apply(*stock, in);
    auto cands = skill::linear_pattern::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("linear_pattern"));
    EXPECT_GT(cands[0].confidence, 0.5);
}

// ─── 6. GEOMETRIC recognition (no history): a real 4-hole row is recovered as
// ONE pattern with the right count and pitch, from geometry alone. ───────────
TEST(SkillLinearPattern, RecognizeFromForeignGeometry)
{
    // Build a 4-hole row, then strip history by wrapping the raw shape.
    auto stock = skill::createCuboidStock(120.0, 40.0, 10.0);
    skill::linear_pattern::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.hole_dia_mm   = 4.0;
    in.hole_depth_mm = 5.0;
    in.count_x       = 4;
    in.pitch_x_mm    = 12.0;
    in.start_x_mm    = -18.0;       // -18,-6,+6,+18
    auto built = skill::linear_pattern::apply(*stock, in);

    skill::Workpiece foreign(built.workpiece->shape());   // NO feature history
    auto cands = skill::linear_pattern::recognize(foreign);
    ASSERT_GE(cands.size(), 1u) << "a 4-hole row must be recognised geometrically";
    const auto& p = cands[0].recovered_params;
    EXPECT_EQ(p.value("count_x", 0), 4) << "all 4 holes counted";
    EXPECT_NEAR(p.value("pitch_x_mm", 0.0), 12.0, 0.5) << "pitch recovered";
    EXPECT_NEAR(p.value("hole_dia_mm", 0.0), 4.0, 0.1) << "diameter recovered";
    ASSERT_TRUE(p.contains("hole_centers"));
    EXPECT_EQ(p["hole_centers"].size(), 4u) << "centres emitted for subsumption";
}

// ─── 7. NO false pattern: two unrelated holes must NOT be read as a pattern. ──
TEST(SkillLinearPattern, TwoHolesAreNotAPattern)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 10.0);
    skill::drill_hole::Input h1;
    h1.position_x_mm = -20.0; h1.position_y_mm = -15.0;
    h1.diameter_mm = 5.0; h1.depth_mm = 6.0;
    auto w1 = skill::drill_hole::apply(*stock, h1);
    skill::drill_hole::Input h2;
    h2.position_x_mm = 18.0; h2.position_y_mm = 22.0;   // unrelated location
    h2.diameter_mm = 5.0; h2.depth_mm = 6.0;
    auto w2 = skill::drill_hole::apply(*w1.workpiece, h2);

    skill::Workpiece foreign(w2.workpiece->shape());
    auto cands = skill::linear_pattern::recognize(foreign);
    EXPECT_TRUE(cands.empty())
        << "two same-diameter but unrelated holes must not be a linear pattern";
}

// ─── 8. SUBSUMPTION: a recognised row replaces its individual drills in the
// inferred plan — one editable pattern step, not N drills. ───────────────────
TEST(SkillLinearPattern, PatternSubsumesIndividualDrills)
{
    auto stock = skill::createCuboidStock(120.0, 40.0, 10.0);
    skill::linear_pattern::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.hole_dia_mm   = 4.0;
    in.hole_depth_mm = 5.0;
    in.count_x       = 4;
    in.pitch_x_mm    = 12.0;
    in.start_x_mm    = -18.0;
    auto built = skill::linear_pattern::apply(*stock, in);

    skill::Workpiece foreign(built.workpiece->shape());
    const process::ProcessPlan plan = re::inferProcessPlan(foreign, 0.7);

    int patternSteps = 0, drillSteps = 0;
    for (const auto& s : plan.steps()) {
        if (s.skill_id == "linear_pattern") ++patternSteps;
        if (s.skill_id == "drill_hole" || s.skill_id == "drill_through_hole")
            ++drillSteps;
    }
    EXPECT_EQ(patternSteps, 1) << "the row is one pattern step";
    EXPECT_EQ(drillSteps, 0)
        << "the pattern must SUBSUME the individual drills (no leftover drill steps)";
}

// ─── 9. SIDE grille (radial axis): a row bored into the +X face, stepping along
// Y (axis = -X — a watch/phone speaker grille), is now recovered as ONE pattern
// about its real axis, and regenerates in place from the recovered world grid. ─
TEST(SkillLinearPattern, SideBoredRowRecognisesAndRegenerates)
{
    // Stock corner at origin: x in [0,40], y in [0,120], z in [0,30].  Build the
    // side grille with linear_pattern's OWN world path (axis -X into the +X face):
    // a 4-hole row stepping along Y, centred on the +X face at z=15.  This drives
    // both apply()'s arbitrary-axis placement and recognize()'s arbitrary-axis
    // recovery — drill_hole itself does not support a Z-free (side) axis.
    auto stock = skill::createCuboidStock(40.0, 120.0, 30.0);
    const double vStock = volumeOf(stock->shape());

    skill::linear_pattern::Input build;
    build.use_world     = true;
    build.world_ox_mm   = 40.0;                 // on the +X face
    build.world_oy_mm   = 42.0;                 // first hole y
    build.world_oz_mm   = 15.0;                 // mid-height
    build.world_nx      = 1.0;                  // OUTWARD normal = +X (cut goes -X)
    build.world_ny      = 0.0;
    build.world_nz      = 0.0;
    build.hole_dia_mm   = 2.0;                  // a realistic small grille hole (<= 3mm)
    build.hole_depth_mm = 6.0;
    build.count_x       = 4;                    // 4 holes along the face-local u axis
    build.pitch_x_mm    = 8.0;
    build.count_y       = 1;
    build.start_x_mm    = 0.0;
    build.start_y_mm    = 0.0;
    auto builtWp = skill::linear_pattern::apply(*stock, build).workpiece;
    const double vBored = volumeOf(builtWp->shape());
    ASSERT_LT(vBored, vStock) << "the side grille must remove material";

    skill::Workpiece foreign(builtWp->shape());
    const auto cands = skill::linear_pattern::recognize(foreign);
    ASSERT_GE(cands.size(), 1u) << "a side grille must now be recovered as a pattern";
    const auto& p = cands[0].recovered_params;
    EXPECT_EQ(std::max(p.value("count_x", 0), p.value("count_y", 0)), 4)
        << "all 4 side holes counted";
    EXPECT_NEAR(p.value("hole_dia_mm", 0.0), 2.0, 0.1);
    ASSERT_TRUE(p.value("use_world", false)) << "side grille carries a world placement";
    // Recovered axis must be ~ +/-X (the bore axis), not +/-Z.
    EXPECT_NEAR(std::abs(p.value("world_nx", 0.0)), 1.0, 1e-3) << "axis is +/-X";
    EXPECT_NEAR(p.value("world_nz", 9.0), 0.0, 1e-3);
    // Depth MUST be recovered (>0) — a 0 depth would abort the Executor replay
    // via validate()'s DFM-INPUT gate.
    EXPECT_NEAR(p.value("hole_depth_mm", 0.0), 6.0, 0.5) << "hole depth recovered from cyl length";

    // REGENERATE from the recovered world grid onto a fresh stock, USING THE
    // RECOVERED DEPTH (not a hard-coded value) → same removed volume in place.
    skill::linear_pattern::Input in;
    in.use_world     = true;
    in.hole_dia_mm   = p.value("hole_dia_mm", 0.0);
    in.hole_depth_mm = p.value("hole_depth_mm", 0.0);
    in.count_x       = p.value("count_x", 1);
    in.pitch_x_mm    = p.value("pitch_x_mm", 0.0);
    in.count_y       = p.value("count_y", 1);
    in.pitch_y_mm    = p.value("pitch_y_mm", 0.0);
    in.start_x_mm    = p.value("start_x_mm", 0.0);
    in.start_y_mm    = p.value("start_y_mm", 0.0);
    in.world_ox_mm   = p.value("world_ox_mm", 0.0);
    in.world_oy_mm   = p.value("world_oy_mm", 0.0);
    in.world_oz_mm   = p.value("world_oz_mm", 0.0);
    in.world_nx      = p.value("world_nx", 0.0);
    in.world_ny      = p.value("world_ny", 0.0);
    in.world_nz      = p.value("world_nz", 1.0);

    auto regen = skill::linear_pattern::apply(*skill::createCuboidStock(40.0, 120.0, 30.0), in);
    ASSERT_FALSE(regen.workpiece->shape().IsNull());
    const double vRegen = volumeOf(regen.workpiece->shape());
    EXPECT_NEAR(vStock - vRegen, vStock - vBored, (vStock - vBored) * 0.10)
        << "the recovered side grille must regenerate the same removed volume";
}

// ─── 9b. SIDE-AXIS SIZE gate: a row of LARGE side bores (dowel pins / oil
// galleries — 8 mm) is functionally NOT a grille and must NOT be over-unified
// into one editable pattern (which would then subsume the real bores).  Small
// grille holes on a side face still recover (test 9); large ones do not. ───────
TEST(SkillLinearPattern, LargeSideBoreRowIsNotAGrille)
{
    auto stock = skill::createCuboidStock(40.0, 120.0, 30.0);
    skill::linear_pattern::Input build;
    build.use_world     = true;
    build.world_ox_mm   = 40.0; build.world_oy_mm = 42.0; build.world_oz_mm = 15.0;
    build.world_nx      = 1.0;  build.world_ny = 0.0; build.world_nz = 0.0;
    build.hole_dia_mm   = 8.0;                  // large structural bore (> 3 mm)
    build.hole_depth_mm = 10.0;
    build.count_x       = 4;
    build.pitch_x_mm    = 16.0;
    build.count_y       = 1;
    auto builtWp = skill::linear_pattern::apply(*stock, build).workpiece;

    skill::Workpiece foreign(builtWp->shape());
    const auto cands = skill::linear_pattern::recognize(foreign);
    EXPECT_TRUE(cands.empty())
        << "a large-diameter side bore row must not be recognised as a grille pattern";
}

// ─── 10. n=3 is below the geometric floor: three collinear, evenly-spaced
// same-diameter holes are NOT enough to call a pattern (2 gaps is near-vacuous);
// the floor is >= 4 so a chance trio doesn't subsume real drills. ────────────
TEST(SkillLinearPattern, ThreeCollinearHolesAreNotAPattern)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);
    std::shared_ptr<skill::Workpiece> wp = stock;
    for (int i = 0; i < 3; ++i) {
        skill::drill_hole::Input h;
        h.position_x_mm = -20.0 + i * 20.0;     // -20, 0, +20 (evenly spaced)
        h.position_y_mm = 0.0;
        h.diameter_mm   = 4.0; h.depth_mm = 6.0;
        wp = skill::drill_hole::apply(*wp, h).workpiece;
    }
    skill::Workpiece foreign(wp->shape());
    auto cands = skill::linear_pattern::recognize(foreign);
    EXPECT_TRUE(cands.empty())
        << "three collinear holes are below the >=4 pattern floor";

    // And inferProcessPlan must keep the 3 drills (no pattern subsumes them).
    const process::ProcessPlan plan = re::inferProcessPlan(foreign, 0.7);
    int patternSteps = 0;
    for (const auto& s : plan.steps())
        if (s.skill_id == "linear_pattern") ++patternSteps;
    EXPECT_EQ(patternSteps, 0) << "no pattern step for only 3 holes";
}
