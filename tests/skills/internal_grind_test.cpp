// @lat: [[process/test-strategy#skill round-trip]]
//
// internal_grind — INTERNAL precision grind on an existing bore.
//
// Tests (5):
//   1. apply() enlarges the bore radius by removal_mm.
//   2. DFM rejects out-of-range removal_mm AND bores smaller than 5 mm dia.
//   3. apply() throws when datum doesn't resolve to a cylinder.
//   4. apply() carries grinding metadata (CBN, internal_ID).
//   5. Composed: drill_hole then internal_grind gives the expected final dia.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_hole.hpp"
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

// ─── 1. Apply: enlarges the bore by removal_mm ────────────────────────────
TEST(SkillInternalGrind, ApplyEnlargesBore)
{
    auto wp = stockWithHole(6.0, 12.0);   // 6 mm dia bore
    const double rBefore = largestCylinderRadius(*wp);
    EXPECT_NEAR(rBefore, 3.0, 1e-3);

    skill::internal_grind::Input in;
    in.cylindrical_face_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(25, 25, 10), gp_Dir(0, 0, -1)) };
    in.removal_mm = 0.05;

    auto out = skill::internal_grind::apply(*wp, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double rAfter = largestCylinderRadius(*out.workpiece);
    EXPECT_NEAR(rAfter, rBefore + 0.05, 1e-3);

    EXPECT_EQ(out.signature.skill_id, std::string("internal_grind"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. DFM rejects out-of-range removal AND too-small bore ───────────────
TEST(SkillInternalGrind, ValidateDFM)
{
    // Out-of-range removal
    {
        auto wp = stockWithHole(6.0, 12.0);
        skill::internal_grind::Input in;
        in.cylindrical_face_datum = skill::FaceCylinderByAxis{
            gp_Ax1(gp_Pnt(25, 25, 10), gp_Dir(0, 0, -1)) };
        in.removal_mm = 0.001;
        auto r = skill::internal_grind::validate(*wp, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::internal_grind::apply(*wp, in), skill::SkillError);

        in.removal_mm = 0.5;
        r = skill::internal_grind::validate(*wp, in);
        EXPECT_FALSE(r.passed);
    }

    // Bore smaller than 5 mm dia
    {
        auto wp = stockWithHole(3.0, 8.0);   // 3 mm dia bore
        skill::internal_grind::Input in;
        in.cylindrical_face_datum = skill::FaceCylinderByAxis{
            gp_Ax1(gp_Pnt(25, 25, 10), gp_Dir(0, 0, -1)) };
        in.removal_mm = 0.02;
        auto r = skill::internal_grind::validate(*wp, in);
        EXPECT_FALSE(r.passed);
        bool foundDia = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-IG-DIA") { foundDia = true; break; }
        EXPECT_TRUE(foundDia);
        EXPECT_THROW(skill::internal_grind::apply(*wp, in), skill::SkillError);
    }
}

// ─── 3. apply() throws when datum doesn't resolve to a cylinder ───────────
TEST(SkillInternalGrind, ApplyThrowsOnNoCylinder)
{
    auto cuboid = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::internal_grind::Input in;
    in.cylindrical_face_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(25, 25, 5), gp_Dir(0, 0, -1)) };
    in.removal_mm = 0.02;

    auto r = skill::internal_grind::validate(*cuboid, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::internal_grind::apply(*cuboid, in), skill::SkillError);
}

// ─── 4. Apply carries grinding metadata (CBN, internal_ID) ────────────────
TEST(SkillInternalGrind, ApplyCarriesGrindingMetadata)
{
    auto wp = stockWithHole(8.0, 15.0);

    skill::internal_grind::Input in;
    in.cylindrical_face_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(25, 25, 10), gp_Dir(0, 0, -1)) };
    in.removal_mm = 0.04;
    in.surface_finish = "ra_0.2";

    auto out = skill::internal_grind::apply(*wp, in);

    EXPECT_EQ(out.signature.tooling.tool_type, std::string("grinding_quill"));
    EXPECT_EQ(out.signature.tooling.tool_material,
              std::string("cubic_boron_nitride"));
    EXPECT_EQ(out.signature.tooling.flute_count, 0);
    EXPECT_EQ(out.signature.tooling.extra["grind_type"].get<std::string>(),
              std::string("internal_ID"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("internal_grind"));
    EXPECT_NEAR(out.signature.pattern["removal_mm"].get<double>(), 0.04, 1e-9);

    // recognize() returns empty (metadata-only differentiation).
    auto cands = skill::internal_grind::recognize(*out.workpiece);
    EXPECT_TRUE(cands.empty());
}

// ─── 5. Composed: drill_hole then internal_grind ──────────────────────────
TEST(SkillInternalGrind, ComposedWorkflowProducesFinalDiameter)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    // Drill an undersize pilot hole at 6.85 mm (under-bore for precision 7 mm).
    skill::drill_hole::Input din;
    din.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    din.position_x_mm = 25.0;
    din.position_y_mm = 25.0;
    din.diameter_mm   = 6.85;
    din.depth_mm      = 12.0;
    auto drilled = skill::drill_hole::apply(*stock, din);
    EXPECT_NEAR(largestCylinderRadius(*drilled.workpiece), 3.425, 1e-3);

    // Internal-grind 0.075 mm radial → 7.00 mm final dia.
    skill::internal_grind::Input gin;
    gin.cylindrical_face_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(25, 25, 10), gp_Dir(0, 0, -1)) };
    gin.removal_mm = 0.075;
    auto ground = skill::internal_grind::apply(*drilled.workpiece, gin);

    EXPECT_NEAR(largestCylinderRadius(*ground.workpiece), 3.5, 1e-3);
    EXPECT_EQ(ground.workpiece->features().size(), 1u);
}
