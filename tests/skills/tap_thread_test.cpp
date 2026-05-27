// @lat: [[process/test-strategy#skill round-trip]]
//
// tap_thread skill — metadata-only approximation tests:
//   1. Pilot hole drilled by drill_hole, then tap_thread::apply records
//      a signature without geometry change.
//   2. DFM gates on insufficient depth and wrong pitch.
//   3. recognize() replays the signature from feature history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_hole.hpp"
#include "skills/tap_thread.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. Apply: no geometry change, signature recorded ─────────────────────
TEST(SkillTapThread, ApplyIsMetadataOnly)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Drill the pilot hole for an M3 tap (tap-drill = 2.5 mm).
    skill::drill_hole::Input drill;
    drill.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    drill.position_x_mm = 25.0;
    drill.position_y_mm = 25.0;
    drill.diameter_mm   = 2.5;
    drill.depth_mm      = 6.0;
    auto pilot = skill::drill_hole::apply(*stock, drill);

    const double volBefore = volumeOf(pilot.workpiece->shape());

    skill::tap_thread::Input in;
    in.existing_hole_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(25.0, 25.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.thread_size     = "M3";
    in.pitch_mm        = 0.5;
    in.thread_depth_mm = 4.0;

    auto out = skill::tap_thread::apply(*pilot.workpiece, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume must NOT change (metadata-only).
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-6);

    // History includes the prior drill_hole + the new tap_thread.
    EXPECT_EQ(out.workpiece->features().size(), 2u);
    EXPECT_EQ(out.signature.skill_id, std::string("tap_thread"));
    EXPECT_EQ(out.signature.pattern["geometry"], "metadata_only");
}

// ── 2. DFM rejects too-shallow thread ────────────────────────────────────
TEST(SkillTapThread, ValidateRejectsShallowThread)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::tap_thread::Input in;
    in.existing_hole_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.thread_size     = "M3";
    in.pitch_mm        = 0.5;
    in.thread_depth_mm = 0.5;  // < 1.5 × pitch = 0.75

    auto r = skill::tap_thread::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool engageFound = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-TAP-ENGAGE") { engageFound = true; break; }
    EXPECT_TRUE(engageFound);
    EXPECT_THROW(skill::tap_thread::apply(*stock, in), skill::SkillError);
}

// ── 3. Recognize replays signature from feature history ──────────────────
TEST(SkillTapThread, RecognizeReplaysSignature)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::drill_hole::Input drill;
    drill.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    drill.position_x_mm = 25.0;
    drill.position_y_mm = 25.0;
    drill.diameter_mm   = 3.3;     // M4 pilot
    drill.depth_mm      = 8.0;
    auto pilot = skill::drill_hole::apply(*stock, drill);

    skill::tap_thread::Input in;
    in.existing_hole_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(25.0, 25.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.thread_size     = "M4";
    in.pitch_mm        = 0.7;
    in.thread_depth_mm = 5.0;

    auto out = skill::tap_thread::apply(*pilot.workpiece, in);
    auto cands = skill::tap_thread::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("tap_thread"));
    EXPECT_EQ(cands[0].recovered_params["thread_size"], "M4");
    EXPECT_NEAR(cands[0].recovered_params["pitch_mm"].get<double>(), 0.7, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["thread_depth_mm"].get<double>(), 5.0, 1e-6);
    EXPECT_GT(cands[0].confidence, 0.9);
}

// ── 4. Tap-drill chart lookup ────────────────────────────────────────────
TEST(SkillTapThread, TapDrillChartLookup)
{
    EXPECT_NEAR(skill::tap_thread::tapDrillDiameter("M3"), 2.5, 1e-6);
    EXPECT_NEAR(skill::tap_thread::tapDrillDiameter("M4"), 3.3, 1e-6);
    EXPECT_NEAR(skill::tap_thread::tapDrillDiameter("M6"), 5.0, 1e-6);
    EXPECT_NEAR(skill::tap_thread::tapDrillDiameter("M99"), 0.0, 1e-6);
}
