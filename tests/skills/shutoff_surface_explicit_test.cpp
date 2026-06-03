// @lat: [[process/test-strategy#skill round-trip]]
//
// shutoff_surface_explicit — adds a thin ribbon of material that seals an
// undercut where mold halves meet.
//
// Cases (5):
//   1. ApplyAddsVolume        — polyline + pull_dir → added ribbon volume.
//   2. ValidateRejectsTinyClearance.
//   3. ValidateRejectsShortPolyline.
//   4. SignatureRecordsCompoundKind.
//   5. RecognizeReplaysHistory.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/shutoff_surface_explicit.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. Ribbon adds volume ────────────────────────────────────────────────
TEST(SkillShutoffSurfaceExplicit, ApplyAddsVolume)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::shutoff_surface_explicit::Input in;
    // Edge polyline lies near the top of the stock; pull direction is +Z so
    // the ribbon extrudes upward from the polyline by `shutoff_face_size_mm`.
    in.undercut_edge_polyline = {
        gp_Pnt(5.0, 15.0, 10.0),
        gp_Pnt(20.0, 15.0, 10.0),
        gp_Pnt(35.0, 15.0, 10.0),
    };
    in.pull_dir_x = 0.0;
    in.pull_dir_y = 0.0;
    in.pull_dir_z = 1.0;
    in.shutoff_face_size_mm = 2.0;

    auto out = skill::shutoff_surface_explicit::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double volAfter = volumeOf(out.workpiece->shape());

    EXPECT_GT(volAfter, volBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("shutoff_surface_explicit"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. DFM rejects clearance < 0.5 mm ───────────────────────────────────
TEST(SkillShutoffSurfaceExplicit, ValidateRejectsTinyClearance)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::shutoff_surface_explicit::Input in;
    in.undercut_edge_polyline = {
        gp_Pnt(5.0, 15.0, 10.0),
        gp_Pnt(35.0, 15.0, 10.0),
    };
    in.pull_dir_z = 1.0;
    in.shutoff_face_size_mm = 0.3;       // < 0.5 mm

    auto r = skill::shutoff_surface_explicit::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SHUTOFF-CLEARANCE") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);

    EXPECT_THROW(skill::shutoff_surface_explicit::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. DFM rejects short polyline ───────────────────────────────────────
TEST(SkillShutoffSurfaceExplicit, ValidateRejectsShortPolyline)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::shutoff_surface_explicit::Input in;
    in.undercut_edge_polyline = { gp_Pnt(5.0, 15.0, 10.0) };   // 1 point
    in.pull_dir_z = 1.0;
    in.shutoff_face_size_mm = 2.0;

    auto r = skill::shutoff_surface_explicit::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Signature records compound + kind ────────────────────────────────
TEST(SkillShutoffSurfaceExplicit, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::shutoff_surface_explicit::Input in;
    in.undercut_edge_polyline = {
        gp_Pnt(5.0, 15.0, 10.0),
        gp_Pnt(35.0, 15.0, 10.0),
    };
    in.pull_dir_z = 1.0;
    in.shutoff_face_size_mm = 2.0;

    auto out = skill::shutoff_surface_explicit::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("shutoff_surface_explicit"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["mold_feature_type"].get<std::string>(),
              std::string("shutoff_surface"));
    // Added volume → derived_volume_removed should be negative.
    EXPECT_LT(out.signature.pattern["derived_volume_removed"].get<double>(),
              0.0);
}

// ─── 5. Recognize replays history ────────────────────────────────────────
TEST(SkillShutoffSurfaceExplicit, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::shutoff_surface_explicit::Input in;
    in.undercut_edge_polyline = {
        gp_Pnt(5.0, 15.0, 10.0),
        gp_Pnt(35.0, 15.0, 10.0),
    };
    in.pull_dir_z = 1.0;
    in.shutoff_face_size_mm = 2.5;
    auto out = skill::shutoff_surface_explicit::apply(*stock, in);

    auto cands = skill::shutoff_surface_explicit::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_GE(cands.front().confidence, 0.9);
    EXPECT_NEAR(
        cands.front().recovered_params["shutoff_face_size_mm"].get<double>(),
        2.5, 0.01);
}
