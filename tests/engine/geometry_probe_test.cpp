// @lat: [[process/test-strategy#geometry probe]]
//
// Unit tests for probe::GeometryProbe — product-agnostic DFM geometry
// measurements (breakthrough plan B5.2-B5.3): planar-planar wedge angle,
// shape-wide minimum dihedral, and hole-to-outer-wall clearance.

#include <gtest/gtest.h>

#include "engine/primitives/Cuts.hpp"
#include "engine/probe/GeometryProbe.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cmath>

namespace probe = koocadcam::engine::probe;
namespace pr    = koocadcam::engine::prim;

namespace {

// Find the first PLANAR face of `s` whose outward normal is within ~2.5°
// of `want` (dot > 0.999).  Null face when none matches.
TopoDS_Face faceWithNormal(const TopoDS_Shape& s, const gp_Dir& want)
{
    for (TopExp_Explorer ex(s, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face& f = TopoDS::Face(ex.Current());
        BRepAdaptor_Surface surf(f);
        if (surf.GetType() != GeomAbs_Plane) continue;
        const double uMid = (surf.FirstUParameter() + surf.LastUParameter()) / 2.0;
        const double vMid = (surf.FirstVParameter() + surf.LastVParameter()) / 2.0;
        gp_Pnt p;
        gp_Vec du, dv;
        surf.D1(uMid, vMid, p, du, dv);
        gp_Vec n = du.Crossed(dv);
        if (n.Magnitude() < 1e-9) continue;
        n.Normalize();
        if (f.Orientation() == TopAbs_REVERSED) n.Reverse();
        if (n.Dot(gp_Vec(want)) > 0.999) return f;
    }
    return {};
}

// Find the first face of `s` whose underlying surface has the given type.
TopoDS_Face faceOfType(const TopoDS_Shape& s, GeomAbs_SurfaceType type)
{
    for (TopExp_Explorer ex(s, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face& f = TopoDS::Face(ex.Current());
        if (BRepAdaptor_Surface(f).GetType() == type) return f;
    }
    return {};
}

}  // namespace

// ── 1. Box: two adjacent planar faces meet at a 90° wedge ────────────────
TEST(GeometryProbe, PlanarPlanarWedgeIsNinety)
{
    const TopoDS_Shape box = BRepPrimAPI_MakeBox(20.0, 30.0, 10.0).Shape();
    const TopoDS_Face fXmin = faceWithNormal(box, gp_Dir(-1.0, 0.0, 0.0));
    const TopoDS_Face fYmin = faceWithNormal(box, gp_Dir(0.0, -1.0, 0.0));
    ASSERT_FALSE(fXmin.IsNull());
    ASSERT_FALSE(fYmin.IsNull());
    EXPECT_NEAR(probe::dihedralWedgeDeg(fXmin, fYmin), 90.0, 0.5);
}

// ── 2. Any curved face in the pair → NaN (curved dihedrals are Phase 2) ──
TEST(GeometryProbe, CurvedFacePairGivesNaN)
{
    const TopoDS_Shape cyl = BRepPrimAPI_MakeCylinder(5.0, 10.0).Shape();
    const TopoDS_Face lateral = faceOfType(cyl, GeomAbs_Cylinder);
    const TopoDS_Face cap     = faceOfType(cyl, GeomAbs_Plane);
    ASSERT_FALSE(lateral.IsNull());
    ASSERT_FALSE(cap.IsNull());
    EXPECT_TRUE(std::isnan(probe::dihedralWedgeDeg(lateral, cap)));
    EXPECT_TRUE(std::isnan(probe::dihedralWedgeDeg(cap, lateral)));
}

// ── 3. Shape-wide minimum: every adjacent face pair of a box is 90° ──────
TEST(GeometryProbe, MinDihedralOfBoxIsNinety)
{
    const TopoDS_Shape box = BRepPrimAPI_MakeBox(20.0, 30.0, 10.0).Shape();
    EXPECT_NEAR(probe::minDihedralAngleDeg(box), 90.0, 0.5);
}

// ── 4. Ø6 through-hole at (10,30) in a 60×60×10 plate: hole wall to the
//      x=0 outer wall is 10 − 3 = 7 mm.  A planar probe face → +inf. ─────
TEST(GeometryProbe, HoleToOuterWallDistance)
{
    const TopoDS_Shape plate = BRepPrimAPI_MakeBox(60.0, 60.0, 10.0).Shape();
    // Drill overshoots both plate faces (z = -1 .. 11) → clean through-hole.
    const gp_Ax2 axis(gp_Pnt(10.0, 30.0, -1.0), gp_Dir(0.0, 0.0, 1.0));
    const TopoDS_Shape drill = BRepPrimAPI_MakeCylinder(axis, 3.0, 12.0).Shape();
    const TopoDS_Shape part  = pr::cut(plate, drill);
    ASSERT_FALSE(part.IsNull());

    const TopoDS_Face hole = faceOfType(part, GeomAbs_Cylinder);
    ASSERT_FALSE(hole.IsNull());
    EXPECT_NEAR(probe::holeToOuterMinDistance(part, hole), 7.0, 0.3);

    // Non-cylindrical probe face → rule N/A → +inf.
    const TopoDS_Face planar = faceOfType(part, GeomAbs_Plane);
    ASSERT_FALSE(planar.IsNull());
    EXPECT_TRUE(std::isinf(probe::holeToOuterMinDistance(part, planar)));
}
