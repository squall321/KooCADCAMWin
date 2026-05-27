// @lat: [[process/test-strategy#compound macro]]
//
// drill_and_tap — compound macro: drill_hole + (synthetic tap metadata).
//
// Test cases:
//   1. apply produces a hole whose diameter matches the pilot table.
//   2. validate rejects unknown thread size.
//   3. signature carries thread_size and tap_depth metadata.
//   4. recognize reverse-resolves thread_size from a standard pilot.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_and_tap.hpp"

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

// ─── 1. M3 → 2.5 mm pilot, drilled to (tap + extra) depth ─────────────────
TEST(SkillDrillAndTap, ApplyDrillsCorrectPilotDiameter)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::drill_and_tap::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm        = 25.0;
    in.position_y_mm        = 25.0;
    in.axis_dir             = gp_Dir(0, 0, -1);
    in.thread_size          = "M3";
    in.tap_depth_mm         = 5.0;
    in.pilot_extra_depth_mm = 1.0;

    auto out = skill::drill_and_tap::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume removed ≈ π × (2.5/2)² × 6.0 mm.
    const double pilotDia = 2.5;
    const double depth    = 6.0;
    const double approx   = M_PI * (pilotDia/2.0) * (pilotDia/2.0) * depth;
    const double diff     = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.05);

    // Signature has the right id and metadata.
    EXPECT_EQ(out.signature.skill_id, std::string("drill_and_tap"));
    EXPECT_EQ(out.signature.pattern.at("thread_size").get<std::string>(),
              std::string("M3"));
    EXPECT_NEAR(out.signature.pattern.at("pilot_dia_mm").get<double>(), 2.5, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("tap_depth_mm").get<double>(), 5.0, 1e-6);
}

// ─── 2. Pilot table lookup helpers work in both directions ────────────────
TEST(SkillDrillAndTap, ThreadSizeTableLookup)
{
    EXPECT_NEAR(skill::drill_and_tap::pilotDiameterFor("M3"), 2.5, 1e-6);
    EXPECT_NEAR(skill::drill_and_tap::pilotDiameterFor("M5"), 4.2, 1e-6);
    EXPECT_EQ  (skill::drill_and_tap::pilotDiameterFor("M99"), 0.0);

    EXPECT_EQ(skill::drill_and_tap::threadSizeFor(2.50, 0.05), std::string("M3"));
    EXPECT_EQ(skill::drill_and_tap::threadSizeFor(4.20, 0.05), std::string("M5"));
    EXPECT_EQ(skill::drill_and_tap::threadSizeFor(7.00, 0.05), std::string());
}

// ─── 3. Validate rejects unknown thread size ──────────────────────────────
TEST(SkillDrillAndTap, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::drill_and_tap::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.thread_size   = "M99";
    in.tap_depth_mm  = 5.0;

    auto r = skill::drill_and_tap::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-TAP-SIZE") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::drill_and_tap::apply(*stock, in), skill::SkillError);
}

// ─── 4. Validate rejects non-positive tap_depth ───────────────────────────
TEST(SkillDrillAndTap, ValidateRejectsZeroTapDepth)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::drill_and_tap::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.thread_size   = "M3";
    in.tap_depth_mm  = 0.0;

    auto r = skill::drill_and_tap::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 5. Recognize reverse-resolves thread size ────────────────────────────
TEST(SkillDrillAndTap, RecognizeRecoversThreadSize)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::drill_and_tap::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.thread_size   = "M4";    // pilot = 3.3
    in.tap_depth_mm  = 5.0;
    in.pilot_extra_depth_mm = 1.0;
    auto out = skill::drill_and_tap::apply(*stock, in);

    auto cands = skill::drill_and_tap::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    bool matched = false;
    for (const auto& c : cands) {
        if (c.recovered_params["thread_size"].get<std::string>() == "M4") {
            EXPECT_NEAR(c.recovered_params["pilot_dia_mm"].get<double>(), 3.3, 1e-2);
            matched = true;
            break;
        }
    }
    EXPECT_TRUE(matched);
}
