// @lat: [[process/test-strategy#auto_boss_under_hole]]
//
// auto_boss_under_hole — REAL geometric verification.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/auto_boss_under_hole.hpp"

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
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

skill::auto_boss_under_hole::Input goodInput()
{
    skill::auto_boss_under_hole::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 25.0;
    in.position_y_mm  = 25.0;
    in.start_z_mm     = 30.0;   // 20mm above the stock top (z=10), needs pillar
    in.axis_dir       = gp_Dir(0, 0, -1);
    in.boss_dia_mm    = 10.0;   // > seat M3 (6) + 2 mm
    in.screw_spec     = "M3";
    in.total_depth_mm = 8.0;
    return in;
}

}  // namespace

// ─── T1: apply produces non-null shape with REAL volume delta ─────────────
TEST(SkillAutoBossUnderHole, ApplyAddsPillarThenCuts)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());

    const double v0 = volumeOf(stock->shape());

    auto in  = goodInput();
    auto out = skill::auto_boss_under_hole::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Pillar height = start_z_mm − zMax = 30 − 10 = 20 mm (boss synthesised).
    const double pillarH = in.start_z_mm - 10.0;
    const double rBoss   = in.boss_dia_mm / 2.0;
    const double vPillar = M_PI * rBoss * rBoss * pillarH;

    // Seat M3: dia 6, depth 3.4.  Pilot M3: dia 3.4 over (8 − 3.4) = 4.6 mm.
    const double rSeat   = 3.0;
    const double rPilot  = 1.7;
    const double vSeat   = M_PI * rSeat  * rSeat  * 3.4;
    const double vPilot  = M_PI * rPilot * rPilot * (in.total_depth_mm - 3.4);

    const double expectedDelta = vPillar - (vSeat + vPilot);
    const double actualDelta   = volumeOf(out.workpiece->shape()) - v0;

    EXPECT_NEAR(actualDelta, expectedDelta,
                std::abs(expectedDelta) * 0.20)
        << "actual=" << actualDelta << " expected≈" << expectedDelta;
}

// ─── T2: validate rejects unknown spec; apply throws ──────────────────────
TEST(SkillAutoBossUnderHole, RejectsUnknownScrewSpec)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto in    = goodInput();
    in.screw_spec = "M99";

    auto r = skill::auto_boss_under_hole::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BOSS-SIZE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::auto_boss_under_hole::apply(*stock, in),
                 skill::SkillError);
}

// ─── T3: signature carries spec key + is_compound = true ──────────────────
TEST(SkillAutoBossUnderHole, SignatureCarriesSpecAndCompound)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto in    = goodInput();
    auto out   = skill::auto_boss_under_hole::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("auto_boss_under_hole"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["screw_spec"].get<std::string>(), "M3");
    EXPECT_GE(out.signature.pattern["subfeature_count"].get<int>(), 2);
}

// ─── T4: recognize returns ≥1 candidate with right spec key ───────────────
TEST(SkillAutoBossUnderHole, RecognizeFindsTheBoss)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto in    = goodInput();
    auto out   = skill::auto_boss_under_hole::apply(*stock, in);

    auto cands = skill::auto_boss_under_hole::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    bool foundM3 = false;
    for (const auto& c : cands) {
        if (c.skill_id != "auto_boss_under_hole") continue;
        if (c.recovered_params.contains("screw_spec") &&
            c.recovered_params["screw_spec"].get<std::string>() == "M3") {
            foundM3 = true;
            EXPECT_GT(c.confidence, 0.5);
        }
    }
    EXPECT_TRUE(foundM3);
}

// ─── T5: context detection — when XY hits solid (in-bbox start), no pillar
TEST(SkillAutoBossUnderHole, ContextDetectsSolidUnderPosition)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 30.0);
    auto in    = goodInput();
    // Start INSIDE the bbox top — no pillar needed.
    in.start_z_mm = 30.0;       // = zMax exactly, distance = 0
    auto out = skill::auto_boss_under_hole::apply(*stock, in);

    EXPECT_FALSE(out.signature.pattern["pillar_synthesized"].get<bool>())
        << "should NOT synthesize pillar when solid is immediately below";
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 2);
}
