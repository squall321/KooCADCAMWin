// @lat: [[process/test-strategy#skill round-trip]]
//
// mold_rib — thin reinforcing wall added perpendicular to a planar face.
//
// Cases (6):
//   1. apply on cuboid stock fuses a rib and ADDS volume.
//   2. DFM rejects thickness < 0.6 mm.
//   3. DFM warns when height/thickness > 3 (stability).
//   4. DFM warns when draft_angle < 0.5 deg.
//   5. recognize replays the feature history.
//   6. Diagonal orientation works (non-axis-aligned start→end).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/mold_rib.hpp"

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

// ─── 1. Apply adds volume ─────────────────────────────────────────────────
TEST(SkillMoldRib, ApplyAddsRibVolume)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const double stockVol = volumeOf(stock->shape());

    skill::mold_rib::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.start_x_mm      = 10.0; in.start_y_mm = 15.0;
    in.end_x_mm        = 30.0; in.end_y_mm   = 15.0;
    in.height_mm       = 3.0;
    in.thickness_mm    = 1.5;
    in.draft_angle_deg = 1.0;

    auto out = skill::mold_rib::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double newVol = volumeOf(out.workpiece->shape());
    // Approximate added volume = length × thickness × height (with slight
    // taper shrinkage from draft, so it should be slightly less).
    const double approxAdded = 20.0 * 1.5 * 3.0;  // 90 mm³
    const double added = newVol - stockVol;
    EXPECT_GT(added, 0.0);
    EXPECT_NEAR(added, approxAdded, approxAdded * 0.20);   // 20% tolerance for draft

    EXPECT_EQ(out.signature.skill_id, std::string("mold_rib"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. DFM rejects too-thin rib ──────────────────────────────────────────
TEST(SkillMoldRib, ValidateRejectsSubMinThickness)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::mold_rib::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm   = 5.0; in.start_y_mm = 15.0;
    in.end_x_mm     = 35.0; in.end_y_mm  = 15.0;
    in.height_mm    = 2.0;
    in.thickness_mm = 0.4;       // < 0.6 mm
    in.draft_angle_deg = 1.0;

    auto r = skill::mold_rib::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-RIB-MIN-THICK") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);

    EXPECT_THROW(skill::mold_rib::apply(*stock, in), skill::SkillError);
}

// ─── 3. DFM warns on stability (height > 3× thickness) ────────────────────
TEST(SkillMoldRib, ValidateWarnsOnTallSlimRib)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);

    skill::mold_rib::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm   = 5.0; in.start_y_mm = 15.0;
    in.end_x_mm     = 35.0; in.end_y_mm  = 15.0;
    in.height_mm    = 8.0;          // 8 / 1.0 = 8 > 3
    in.thickness_mm = 1.0;
    in.draft_angle_deg = 1.0;

    auto r = skill::mold_rib::validate(*stock, in);
    EXPECT_TRUE(r.passed);  // warning, not error
    bool sawWarn = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-RIB-STAB" && f.severity == "warning") { sawWarn = true; break; }
    EXPECT_TRUE(sawWarn);
}

// ─── 4. DFM warns on too-low draft ────────────────────────────────────────
TEST(SkillMoldRib, ValidateWarnsOnLowDraft)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::mold_rib::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm      = 5.0; in.start_y_mm = 15.0;
    in.end_x_mm        = 35.0; in.end_y_mm  = 15.0;
    in.height_mm       = 2.0;
    in.thickness_mm    = 1.0;
    in.draft_angle_deg = 0.1;      // < 0.5

    auto r = skill::mold_rib::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    bool sawWarn = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-RIB-DRAFT") { sawWarn = true; break; }
    EXPECT_TRUE(sawWarn);
}

// ─── 5. Recognize replays parameters from history ─────────────────────────
TEST(SkillMoldRib, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 12.0);

    skill::mold_rib::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm      = 12.0; in.start_y_mm = 20.0;
    in.end_x_mm        = 38.0; in.end_y_mm   = 20.0;
    in.height_mm       = 3.0;
    in.thickness_mm    = 1.2;
    in.draft_angle_deg = 1.5;

    auto out = skill::mold_rib::apply(*stock, in);
    auto cands = skill::mold_rib::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    const auto& c = cands.front();
    EXPECT_GT(c.confidence, 0.7);
    EXPECT_NEAR(c.recovered_params["thickness_mm"].get<double>(), 1.2, 0.05);
    EXPECT_NEAR(c.recovered_params["height_mm"].get<double>(), 3.0, 0.05);
    EXPECT_NEAR(c.recovered_params["start_x_mm"].get<double>(), 12.0, 0.01);
    EXPECT_NEAR(c.recovered_params["end_x_mm"].get<double>(), 38.0, 0.01);
}

// ─── 6. Diagonal rib (non axis-aligned) ───────────────────────────────────
TEST(SkillMoldRib, DiagonalRib)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::mold_rib::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm      = 10.0; in.start_y_mm = 10.0;
    in.end_x_mm        = 40.0; in.end_y_mm   = 40.0;  // 45°
    in.height_mm       = 2.0;
    in.thickness_mm    = 1.0;
    in.draft_angle_deg = 1.0;

    auto out = skill::mold_rib::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_GT(volumeOf(out.workpiece->shape()),
              volumeOf(stock->shape()));
}
