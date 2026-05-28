// @lat: [[process/test-strategy#skill round-trip]]
//
// honing_op — precision bore finish via reciprocating abrasive stones.
//
// Tests (5):
//   1. apply() enlarges the bore by removal_mm (default 0.002 mm).
//   2. DFM rejects out-of-range removal (< 0.0005, > 0.02) AND bore < 3 mm.
//   3. apply() throws when datum doesn't resolve to a cylinder.
//   4. apply() carries hone tooling + cross_hatch_angle_deg metadata.
//   5. Composed: drill_hole → internal_grind → honing_op (3-step finish).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_hole.hpp"
#include "skills/honing_op.hpp"
#include "skills/internal_grind.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Cylinder.hxx>

#include <algorithm>
#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

double largestCylinderRadius(const skill::Workpiece& wp)
{
    double best = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        best = std::max(best, surf.Cylinder().Radius());
    }
    return best;
}

std::shared_ptr<skill::Workpiece> stockWithHole(double dia, double depth)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::drill_hole::Input din;
    din.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    din.position_x_mm = 25.0;
    din.position_y_mm = 25.0;
    din.diameter_mm   = dia;
    din.depth_mm      = depth;
    auto out = skill::drill_hole::apply(*stock, din);
    return out.workpiece;
}

}  // namespace

// ─── 1. Apply: enlarges bore by removal_mm ────────────────────────────────
TEST(SkillHoningOp, ApplyEnlargesBoreByRemoval)
{
    auto wp = stockWithHole(8.0, 15.0);
    const double rBefore = largestCylinderRadius(*wp);
    EXPECT_NEAR(rBefore, 4.0, 1e-3);

    skill::honing_op::Input in;
    in.bore_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(25, 25, 10), gp_Dir(0, 0, -1)) };
    in.removal_mm = 0.005;
    in.cross_hatch_angle_deg = 45.0;

    auto out = skill::honing_op::apply(*wp, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double rAfter = largestCylinderRadius(*out.workpiece);
    // Hone removal is small but measurable.
    EXPECT_NEAR(rAfter, rBefore + 0.005, 5e-3);

    EXPECT_EQ(out.signature.skill_id, std::string("honing_op"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. DFM rejects out-of-range removal AND too-small bore ───────────────
TEST(SkillHoningOp, ValidateDFM)
{
    // Out-of-range removal
    {
        auto wp = stockWithHole(8.0, 15.0);
        skill::honing_op::Input in;
        in.bore_datum = skill::FaceCylinderByAxis{
            gp_Ax1(gp_Pnt(25, 25, 10), gp_Dir(0, 0, -1)) };

        in.removal_mm = 0.0001;          // < 0.0005
        auto r = skill::honing_op::validate(*wp, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::honing_op::apply(*wp, in), skill::SkillError);

        in.removal_mm = 0.05;            // > 0.02
        r = skill::honing_op::validate(*wp, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::honing_op::apply(*wp, in), skill::SkillError);
    }

    // Bore smaller than 3 mm dia
    {
        auto wp = stockWithHole(2.0, 6.0);
        skill::honing_op::Input in;
        in.bore_datum = skill::FaceCylinderByAxis{
            gp_Ax1(gp_Pnt(25, 25, 10), gp_Dir(0, 0, -1)) };
        in.removal_mm = 0.002;
        auto r = skill::honing_op::validate(*wp, in);
        EXPECT_FALSE(r.passed);
        bool foundDia = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-HN-DIA") { foundDia = true; break; }
        EXPECT_TRUE(foundDia);
        EXPECT_THROW(skill::honing_op::apply(*wp, in), skill::SkillError);
    }
}

// ─── 3. apply() throws when datum doesn't resolve to a cylinder ───────────
TEST(SkillHoningOp, ApplyThrowsOnNoCylinder)
{
    auto cuboid = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::honing_op::Input in;
    in.bore_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(25, 25, 5), gp_Dir(0, 0, -1)) };
    in.removal_mm = 0.002;

    auto r = skill::honing_op::validate(*cuboid, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::honing_op::apply(*cuboid, in), skill::SkillError);
}

// ─── 4. Apply carries hone metadata + cross_hatch_angle_deg ───────────────
TEST(SkillHoningOp, ApplyCarriesHoneMetadata)
{
    auto wp = stockWithHole(8.0, 15.0);

    skill::honing_op::Input in;
    in.bore_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(25, 25, 10), gp_Dir(0, 0, -1)) };
    in.removal_mm = 0.002;
    in.cross_hatch_angle_deg = 30.0;

    auto out = skill::honing_op::apply(*wp, in);

    EXPECT_EQ(out.signature.tooling.tool_type, std::string("honing_stone"));
    EXPECT_EQ(out.signature.tooling.tool_material, std::string("silicon_carbide"));
    EXPECT_EQ(out.signature.tooling.flute_count, 0);
    EXPECT_LT(out.signature.tooling.cutting_speed_sfm, 1000.0);   // hones are slow
    EXPECT_NEAR(
        out.signature.pattern["cross_hatch_angle_deg"].get<double>(), 30.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("honing_op"));

    // recognize() returns empty (metadata-only differentiation).
    auto cands = skill::honing_op::recognize(*out.workpiece);
    EXPECT_TRUE(cands.empty());
}

// ─── 5. Composed: drill → internal_grind → honing_op ──────────────────────
TEST(SkillHoningOp, ComposedThreeStepFinish)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    // Drill at 9.85 mm
    skill::drill_hole::Input din;
    din.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    din.position_x_mm = 25.0;
    din.position_y_mm = 25.0;
    din.diameter_mm   = 9.85;
    din.depth_mm      = 15.0;
    auto drilled = skill::drill_hole::apply(*stock, din);
    EXPECT_NEAR(largestCylinderRadius(*drilled.workpiece), 4.925, 1e-3);

    // Internal grind 0.07 mm → 9.99 mm
    skill::internal_grind::Input gin;
    gin.cylindrical_face_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(25, 25, 10), gp_Dir(0, 0, -1)) };
    gin.removal_mm = 0.07;
    auto ground = skill::internal_grind::apply(*drilled.workpiece, gin);
    EXPECT_NEAR(largestCylinderRadius(*ground.workpiece), 4.995, 1e-2);

    // Hone 0.005 mm → 10.00 mm
    skill::honing_op::Input hin;
    hin.bore_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(25, 25, 10), gp_Dir(0, 0, -1)) };
    hin.removal_mm = 0.005;
    auto honed = skill::honing_op::apply(*ground.workpiece, hin);
    EXPECT_NEAR(largestCylinderRadius(*honed.workpiece), 5.0, 1e-2);

    EXPECT_EQ(honed.signature.skill_id, std::string("honing_op"));
}
