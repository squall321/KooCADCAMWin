// @lat: [[process/test-strategy#skill round-trip]]
//
// sweep_surface — circular profile swept along a polyline path.
//
// Cases (6):
//   1. ApplyProducesNonNull shape.
//   2. ValidateRejectsBadInput — single waypoint.
//   3. SignatureRecordsKind — pattern.kind == "sweep_surface".
//   4. RecognizeMetadataReplay — history replay confidence 1.0.
//   5. ValidateRejectsCoincidentPoints.
//   6. ApplyRecordsPathLengthAndWaypointCount.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/sweep_surface.hpp"

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

// ─── 1. ApplyProducesNonNull shape ────────────────────────────────────────
TEST(SkillSweepSurface, ApplyProducesNonNull)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::sweep_surface::Input in;
    in.path_waypoints = {
        gp_Pnt(10.0, 10.0, 15.0),
        gp_Pnt(30.0, 10.0, 15.0),
        gp_Pnt(30.0, 20.0, 15.0),
    };
    in.profile_radius_mm = 1.0;

    auto out = skill::sweep_surface::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    // Sweep should add volume.
    EXPECT_GT(volumeOf(out.workpiece->shape()), volumeOf(stock->shape()) - 1e-6);
}

// ─── 2. ValidateRejectsBadInput (single waypoint) ─────────────────────────
TEST(SkillSweepSurface, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::sweep_surface::Input in;
    in.path_waypoints = { gp_Pnt(10, 10, 15) };
    in.profile_radius_mm = 1.0;

    auto r = skill::sweep_surface::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);

    EXPECT_THROW(skill::sweep_surface::apply(*stock, in), skill::SkillError);
}

// ─── 3. SignatureRecordsKind ──────────────────────────────────────────────
TEST(SkillSweepSurface, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::sweep_surface::Input in;
    in.path_waypoints = {
        gp_Pnt(10, 10, 15),
        gp_Pnt(30, 10, 15),
    };
    in.profile_radius_mm = 1.0;

    auto out = skill::sweep_surface::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("sweep_surface"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("sweep_surface"));
}

// ─── 4. RecognizeMetadataReplay (confidence 1.0) ──────────────────────────
TEST(SkillSweepSurface, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::sweep_surface::Input in;
    in.path_waypoints = {
        gp_Pnt(10, 10, 15),
        gp_Pnt(30, 10, 15),
        gp_Pnt(30, 20, 15),
    };
    in.profile_radius_mm = 1.5;

    auto out = skill::sweep_surface::apply(*stock, in);
    auto cands = skill::sweep_surface::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("sweep_surface"));
    EXPECT_DOUBLE_EQ(c.confidence, 1.0);
    EXPECT_NEAR(c.recovered_params["profile_radius_mm"].get<double>(),
                1.5, 1e-6);
}

// ─── 5. ValidateRejectsCoincidentPoints ───────────────────────────────────
TEST(SkillSweepSurface, ValidateRejectsCoincidentPoints)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::sweep_surface::Input in;
    in.path_waypoints = {
        gp_Pnt(10, 10, 15),
        gp_Pnt(10, 10, 15),    // coincident with previous
        gp_Pnt(30, 10, 15),
    };
    in.profile_radius_mm = 1.0;

    auto r = skill::sweep_surface::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SWEEP-COLL") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}

// ─── 6. ApplyRecordsPathLengthAndWaypointCount ────────────────────────────
TEST(SkillSweepSurface, ApplyRecordsPathLengthAndWaypointCount)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::sweep_surface::Input in;
    in.path_waypoints = {
        gp_Pnt(10.0, 10.0, 15.0),
        gp_Pnt(30.0, 10.0, 15.0),    // +20 mm
        gp_Pnt(30.0, 20.0, 15.0),    // +10 mm
    };
    in.profile_radius_mm = 1.0;

    auto out = skill::sweep_surface::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["path_waypoint_count"].get<int>(), 3);
    EXPECT_NEAR(out.signature.pattern["path_length_mm"].get<double>(),
                30.0, 1e-3);
}
