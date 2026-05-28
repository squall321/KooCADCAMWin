// @lat: [[process/test-strategy#skill round-trip]]
//
// undercut_relief — small cylindrical pocket at a parting-line corner for
// side-action engagement.
//
// Cases:
//   1. ApplyRemovesMaterial : a 3 mm relief removes ~ π r² × depth.
//   2. ValidateRejectsBadInput : dia outside [0.5, 6.0] mm → DFM-UC-DIA.
//   3. SignatureRecordsKind + dia + depth.
//   4. RecognizeMetadataReplay : recognize() replays from history.
//   5. RejectsOutOfPlaneAxis (specific) : axis_xy with Z != 0 → DFM-UC-AXIS.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/undercut_relief.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

}  // namespace

// ─── 1. Apply removes material ────────────────────────────────────────────
TEST(SkillUndercutRelief, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);
    const double stockVol = volumeOf(stock->shape());

    skill::undercut_relief::Input in;
    // Corner at the +X side of the stock, axis pointing -X (cuts INTO the
    // block from the side at x = 40).
    in.corner_x_mm = 40.0;
    in.corner_y_mm = 15.0;
    in.corner_z_mm = 10.0;
    in.axis_xy     = gp_Dir(-1.0, 0.0, 0.0);
    in.dia_mm      = 3.0;
    in.depth_mm    = 5.0;

    auto out = skill::undercut_relief::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double removed = stockVol - volumeOf(out.workpiece->shape());
    const double r = 1.5;
    const double approx = M_PI * r * r * 5.0;
    EXPECT_GT(removed, 0.0);
    EXPECT_NEAR(removed, approx, approx * 0.25);
}

// ─── 2. Validate rejects bad input (oversized dia) ────────────────────────
TEST(SkillUndercutRelief, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);

    skill::undercut_relief::Input in;
    in.corner_x_mm = 40.0; in.corner_y_mm = 15.0; in.corner_z_mm = 10.0;
    in.axis_xy     = gp_Dir(-1.0, 0.0, 0.0);
    in.dia_mm      = 10.0;       // > 6.0
    in.depth_mm    = 3.0;

    auto r = skill::undercut_relief::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-UC-DIA") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);

    EXPECT_THROW(skill::undercut_relief::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillUndercutRelief, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);

    skill::undercut_relief::Input in;
    in.corner_x_mm = 40.0; in.corner_y_mm = 15.0; in.corner_z_mm = 10.0;
    in.axis_xy     = gp_Dir(-1.0, 0.0, 0.0);
    in.dia_mm      = 3.5;
    in.depth_mm    = 4.0;

    auto out = skill::undercut_relief::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("undercut_relief"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("undercut_relief"));
    EXPECT_NEAR(out.signature.pattern["dia_mm"].get<double>(),   3.5, 1e-9);
    EXPECT_NEAR(out.signature.pattern["depth_mm"].get<double>(), 4.0, 1e-9);
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillUndercutRelief, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);

    skill::undercut_relief::Input in;
    in.corner_x_mm = 40.0; in.corner_y_mm = 12.0; in.corner_z_mm = 8.0;
    in.axis_xy     = gp_Dir(-1.0, 0.0, 0.0);
    in.dia_mm      = 2.5;
    in.depth_mm    = 3.0;

    auto out = skill::undercut_relief::apply(*stock, in);
    auto cands = skill::undercut_relief::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    const auto& c = cands.front();
    EXPECT_GT(c.confidence, 0.7);
    EXPECT_NEAR(c.recovered_params["dia_mm"].get<double>(),   2.5, 1e-9);
    EXPECT_NEAR(c.recovered_params["depth_mm"].get<double>(), 3.0, 1e-9);
    EXPECT_NEAR(c.recovered_params["corner_x_mm"].get<double>(), 40.0, 1e-9);
}

// ─── 5. Rejects out-of-plane axis (specific) ──────────────────────────────
TEST(SkillUndercutRelief, RejectsOutOfPlaneAxis)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);

    skill::undercut_relief::Input in;
    in.corner_x_mm = 40.0; in.corner_y_mm = 15.0; in.corner_z_mm = 10.0;
    in.axis_xy     = gp_Dir(0.0, 0.0, 1.0);   // pointing +Z (out of XY)
    in.dia_mm      = 3.0;
    in.depth_mm    = 3.0;

    auto r = skill::undercut_relief::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-UC-AXIS") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}
