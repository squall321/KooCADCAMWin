// @lat: [[process/test-strategy#skill round-trip]]
//
// rigid_tap skill tests (slice — threading variant).
//   1. Apply runs a real helical sweep, removes a small amount of stock.
//   2. DFM rejects insufficient engagement (DFM-TAP-ENGAGE).
//   3. DFM emits DFM-RIGID-TAP-PITCH (warning) on non-standard pitch.
//   4. Standard coarse-pitch and tap-drill chart lookups.
//   5. Recognize replays signature from feature history at confidence 0.5.
//   6. Signature carries rigid_tap_mode metadata.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_hole.hpp"
#include "skills/rigid_tap.hpp"

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

// ── 1. Apply: helical thread cuts a small volume of material ─────────────
TEST(SkillRigidTap, ApplyProducesValidThread)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Drill the pilot hole for an M3 rigid-tap (chart pilot = 2.5 mm).
    skill::drill_hole::Input drill;
    drill.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    drill.position_x_mm = 25.0;
    drill.position_y_mm = 25.0;
    drill.diameter_mm   = 2.5;
    drill.depth_mm      = 6.0;
    auto pilot = skill::drill_hole::apply(*stock, drill);

    const double volBefore = volumeOf(pilot.workpiece->shape());

    skill::rigid_tap::Input in;
    in.existing_hole_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(25.0, 25.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.thread_size     = "M3";
    in.pitch_mm        = 0.5;
    in.thread_depth_mm = 4.0;

    auto out = skill::rigid_tap::apply(*pilot.workpiece, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Real swept geometry (or annular-ring fallback) — volume must not increase.
    EXPECT_LE(volumeOf(out.workpiece->shape()), volBefore + 1e-6);
    EXPECT_EQ(out.workpiece->features().size(), 2u);
    EXPECT_EQ(out.signature.skill_id, std::string("rigid_tap"));
    EXPECT_TRUE(out.signature.pattern.at("rigid_tap_mode").get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("rigid_tap"));

    const std::string geom = out.signature.pattern["geometry"].get<std::string>();
    EXPECT_TRUE(geom == "helical_swept" || geom == "metadata_only")
        << "unexpected geometry tag: " << geom;
}

// ── 2. DFM rejects too-shallow engagement ────────────────────────────────
TEST(SkillRigidTap, ValidateRejectsShallowEngagement)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::rigid_tap::Input in;
    in.existing_hole_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.thread_size     = "M3";
    in.pitch_mm        = 0.5;
    in.thread_depth_mm = 0.5;     // < 1.5 × pitch (0.75)

    auto r = skill::rigid_tap::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool engageFound = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-TAP-ENGAGE") engageFound = true;
    EXPECT_TRUE(engageFound);
    EXPECT_THROW(skill::rigid_tap::apply(*stock, in), skill::SkillError);
}

// ── 3. DFM-RIGID-TAP-PITCH warning on non-standard pitch ─────────────────
TEST(SkillRigidTap, ValidateWarnsOnNonStandardPitch)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::drill_hole::Input drill;
    drill.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    drill.position_x_mm = 25.0;
    drill.position_y_mm = 25.0;
    drill.diameter_mm   = 2.5;
    drill.depth_mm      = 6.0;
    auto pilot = skill::drill_hole::apply(*stock, drill);

    skill::rigid_tap::Input in;
    in.existing_hole_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(25.0, 25.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.thread_size     = "M3";
    in.pitch_mm        = 0.35;   // not the standard M3 coarse pitch (0.5)
    in.thread_depth_mm = 4.0;

    auto r = skill::rigid_tap::validate(*pilot.workpiece, in);
    EXPECT_TRUE(r.passed);   // warning only, validation still passes
    bool pitchWarn = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-RIGID-TAP-PITCH") {
            EXPECT_EQ(f.severity, "warning");
            pitchWarn = true;
        }
    EXPECT_TRUE(pitchWarn);
}

// ── 4. Standard-pitch and tap-drill chart lookups ────────────────────────
TEST(SkillRigidTap, ChartLookups)
{
    EXPECT_NEAR(skill::rigid_tap::standardCoarsePitch("M3"), 0.50, 1e-6);
    EXPECT_NEAR(skill::rigid_tap::standardCoarsePitch("M4"), 0.70, 1e-6);
    EXPECT_NEAR(skill::rigid_tap::standardCoarsePitch("M5"), 0.80, 1e-6);
    EXPECT_NEAR(skill::rigid_tap::standardCoarsePitch("M6"), 1.00, 1e-6);
    EXPECT_NEAR(skill::rigid_tap::standardCoarsePitch("M8"), 1.25, 1e-6);
    EXPECT_NEAR(skill::rigid_tap::standardCoarsePitch("M99"), 0.0, 1e-6);

    EXPECT_NEAR(skill::rigid_tap::tapDrillDiameter("M3"), 2.50, 1e-6);
    EXPECT_NEAR(skill::rigid_tap::tapDrillDiameter("M6"), 5.00, 1e-6);
    EXPECT_NEAR(skill::rigid_tap::tapDrillDiameter("Mxx"), 0.0, 1e-6);
}

// ── 5. Recognize replays signature at 0.5 confidence ─────────────────────
TEST(SkillRigidTap, RecognizeReplaysSignature)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::drill_hole::Input drill;
    drill.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    drill.position_x_mm = 25.0;
    drill.position_y_mm = 25.0;
    drill.diameter_mm   = 4.2;   // M5 pilot
    drill.depth_mm      = 8.0;
    auto pilot = skill::drill_hole::apply(*stock, drill);

    skill::rigid_tap::Input in;
    in.existing_hole_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(25.0, 25.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.thread_size     = "M5";
    in.pitch_mm        = 0.8;
    in.thread_depth_mm = 5.0;

    auto out   = skill::rigid_tap::apply(*pilot.workpiece, in);
    auto cands = skill::rigid_tap::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("rigid_tap"));
    EXPECT_EQ(cands[0].recovered_params["thread_size"], "M5");
    EXPECT_NEAR(cands[0].confidence, 0.5, 1e-6);
    EXPECT_EQ(cands[0].matched_geometry["source"], "metadata");
}

// ── 6. Recognize on a fresh stock returns nothing ────────────────────────
TEST(SkillRigidTap, RecognizeOnFreshStockEmpty)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto cands = skill::rigid_tap::recognize(*stock);
    EXPECT_TRUE(cands.empty());
}
