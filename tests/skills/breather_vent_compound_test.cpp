// @lat: [[process/test-strategy#breather_vent_compound]]
//
// breather_vent_compound — COMPOUND, SPEC-DRIVEN feature.
//   3-step concentric chain: through hole + counterbore + O-ring groove.
//
// Test cases:
//   1. apply removes ≈ π × r_through² × wall + π × (r_cb² − r_through²) ×
//      cb_depth + π × (r_groove² − r_cb²) × groove_width.
//   2. validate rejects unknown spec + counterbore+groove ≥ wall thickness
//      (context auto-detect failure) + apply throws.
//   3. signature carries spec_key + subfeature_count == 3.
//   4. recognize returns the candidate with the right spec_key (> 50 % conf).
//   5. spec table correctness: each BV-Mn entry has the expected diameters
//      AND auto-context-detect: same spec on a THIN wall fails DFM-BV-WALL.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/breather_vent_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. apply removes spec'd volume ─────────────────────────────────────
TEST(SkillBreatherVent, ApplyRemovesCompoundVolume)
{
    // BV-M12 needs cb_depth + groove_depth = 3 + 0.7 = 3.7 < (wall − 0.5).
    // Use a 10 mm-thick wall for headroom.
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double v0 = volumeOf(stock->shape());

    skill::breather_vent_compound::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.spec_key      = "BV-M12";
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.axis_dir      = gp_Dir(0, 0, -1);

    auto out = skill::breather_vent_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    // BV-M12: through_dia=6, cb_dia=12, cb_depth=3, groove_dia=14,
    //         groove_width=1.0
    const double rThru = 3.0;
    const double rCb   = 6.0;
    const double rGr   = 7.0;
    const double wall  = 10.0;
    const double cbDepth = 3.0;
    const double gWidth = 1.0;
    const double approx =
        M_PI * rThru * rThru * wall +
        M_PI * (rCb * rCb - rThru * rThru) * cbDepth +
        M_PI * (rGr * rGr - rCb * rCb) * gWidth;
    const double removed = v0 - v1;
    EXPECT_NEAR(removed, approx, approx * 0.05)
        << "expected ≈ " << approx << " mm³, got " << removed;

    EXPECT_EQ(out.signature.skill_id,
              std::string("breather_vent_compound"));
}

// ─── 2. validate rejects bad inputs (incl. context auto-detect) ─────────
TEST(SkillBreatherVent, ValidateRejectsBadInputs)
{
    // (a) Unknown spec
    {
        auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
        skill::breather_vent_compound::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.spec_key   = "BV-M99";
        in.position_x_mm = 20.0;
        in.position_y_mm = 20.0;
        auto r = skill::breather_vent_compound::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool sp = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-BV-SP") { sp = true; break; }
        EXPECT_TRUE(sp);
        EXPECT_THROW(skill::breather_vent_compound::apply(*stock, in),
                     skill::SkillError);
    }

    // (b) Context auto-detect — THIN wall (3 mm) cannot host BV-M16
    //     (cb_depth 4 + groove_depth 1 = 5 mm > 3 − 0.5).
    {
        auto stock = skill::createCuboidStock(40.0, 40.0, 3.0);
        skill::breather_vent_compound::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.spec_key   = "BV-M16";    // requires ≥ 5.5 mm wall
        in.position_x_mm = 20.0;
        in.position_y_mm = 20.0;
        auto r = skill::breather_vent_compound::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool wall = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-BV-WALL") { wall = true; break; }
        EXPECT_TRUE(wall) << "context auto-detect (wall thickness) must fail";
        EXPECT_THROW(skill::breather_vent_compound::apply(*stock, in),
                     skill::SkillError);
    }
}

// ─── 3. signature carries spec key + 3 subfeatures ──────────────────────
TEST(SkillBreatherVent, SignatureCarriesSpec)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::breather_vent_compound::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.spec_key      = "BV-M8";
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    auto out = skill::breather_vent_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("breather_vent_compound"));
    EXPECT_EQ(out.signature.pattern.at("is_compound").get<bool>(), true);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
    EXPECT_EQ(out.signature.pattern.at("spec_key").get<std::string>(),
              std::string("BV-M8"));
    EXPECT_NEAR(out.signature.pattern.at("through_dia_mm").get<double>(),
                4.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("counterbore_dia_mm").get<double>(),
                8.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("groove_dia_mm").get<double>(),
                10.0, 1e-6);
}

// ─── 4. recognize returns candidate ─────────────────────────────────────
TEST(SkillBreatherVent, RecognizeRecoversSpec)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::breather_vent_compound::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.spec_key      = "BV-M12";
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    auto out = skill::breather_vent_compound::apply(*stock, in);

    auto cands = skill::breather_vent_compound::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_GT(cands.front().confidence, 0.5);
    EXPECT_EQ(cands.front().recovered_params["spec_key"].get<std::string>(),
              std::string("BV-M12"));
    EXPECT_NEAR(cands.front().recovered_params["through_dia_mm"].get<double>(),
                6.0, 1e-6);
}

// ─── 5. spec table correctness AND context-aware DFM ────────────────────
TEST(SkillBreatherVent, SpecTableAndContextDetect)
{
    auto m8  = skill::breather_vent_compound::breatherSpecFor("BV-M8");
    auto m12 = skill::breather_vent_compound::breatherSpecFor("BV-M12");
    auto m16 = skill::breather_vent_compound::breatherSpecFor("BV-M16");
    auto m20 = skill::breather_vent_compound::breatherSpecFor("BV-M20");
    auto bad = skill::breather_vent_compound::breatherSpecFor("BV-M99");

    EXPECT_NEAR(m8.through_dia_mm,        4.0, 1e-9);
    EXPECT_NEAR(m8.counterbore_dia_mm,    8.0, 1e-9);
    EXPECT_NEAR(m8.counterbore_depth_mm,  2.0, 1e-9);
    EXPECT_NEAR(m8.groove_dia_mm,        10.0, 1e-9);
    EXPECT_NEAR(m8.groove_width_mm,       0.8, 1e-9);

    EXPECT_NEAR(m12.through_dia_mm,       6.0, 1e-9);
    EXPECT_NEAR(m16.counterbore_dia_mm,  16.0, 1e-9);
    EXPECT_NEAR(m20.groove_dia_mm,       22.0, 1e-9);

    EXPECT_NEAR(bad.through_dia_mm, 0.0, 1e-9);

    // Context-aware: the SAME spec key passes on a thick wall and fails on
    // a thin wall.  Demonstrates the auto-detect wall-thickness logic.
    {
        auto thick = skill::createCuboidStock(40.0, 40.0, 10.0);
        skill::breather_vent_compound::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.spec_key   = "BV-M16";
        in.position_x_mm = 20.0;
        in.position_y_mm = 20.0;
        auto r = skill::breather_vent_compound::validate(*thick, in);
        EXPECT_TRUE(r.passed) << "10 mm wall is plenty for BV-M16";
    }
    {
        auto thin = skill::createCuboidStock(40.0, 40.0, 3.0);
        skill::breather_vent_compound::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.spec_key   = "BV-M16";
        in.position_x_mm = 20.0;
        in.position_y_mm = 20.0;
        auto r = skill::breather_vent_compound::validate(*thin, in);
        EXPECT_FALSE(r.passed) << "3 mm wall cannot host BV-M16";
    }
}
