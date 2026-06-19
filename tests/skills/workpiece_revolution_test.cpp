// @lat: [[engine/skills#Workpiece]]
//
// Workpiece surface-of-revolution detection (prerequisite for geometric
// revolve recognition).  isFaceRevolution flags cylinder/cone/torus/sphere/
// surface-of-revolution faces; faceRevolutionAxis recovers their axis.  A
// lathe/revolve part's curved faces all share one axis — the signal the
// geometric revolve-RE path will group on.

#include <gtest/gtest.h>

#include "skills/Workpiece.hpp"

#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <cmath>

using namespace koocadcam;

// A cylinder has one revolution side face whose axis is the build axis (+Z).
TEST(WorkpieceRevolution, CylinderSideFaceIsRevolutionAboutAxis)
{
    const TopoDS_Shape cyl =
        BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 5.0, 10.0).Shape();
    skill::Workpiece wp(cyl);

    int revFaces = 0;
    bool sawAxialZ = false;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceRevolution(i)) continue;
        ++revFaces;
        const auto ax = wp.faceRevolutionAxis(i);
        ASSERT_TRUE(ax.has_value()) << "a cylinder face must report an axis";
        if (std::abs(std::abs(ax->Direction().Z()) - 1.0) < 1e-6) sawAxialZ = true;
    }
    EXPECT_GE(revFaces, 1) << "cylinder must have a revolution side face";
    EXPECT_TRUE(sawAxialZ) << "the recovered axis must be +Z";
}

// A cone's lateral face is a revolution surface about the same +Z axis.
TEST(WorkpieceRevolution, ConeLateralFaceIsRevolution)
{
    const TopoDS_Shape cone =
        BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 6.0, 2.0, 8.0).Shape();
    skill::Workpiece wp(cone);

    bool foundConeAxis = false;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceRevolution(i)) continue;
        const auto ax = wp.faceRevolutionAxis(i);
        if (ax && std::abs(std::abs(ax->Direction().Z()) - 1.0) < 1e-6) foundConeAxis = true;
    }
    EXPECT_TRUE(foundConeAxis) << "cone lateral face must be a +Z revolution surface";
}

// A planar face is NOT a revolution surface and reports no axis.
TEST(WorkpieceRevolution, PlanarFaceIsNotRevolution)
{
    const TopoDS_Shape cyl =
        BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 5.0, 10.0).Shape();
    skill::Workpiece wp(cyl);

    int planarChecked = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (wp.isFacePlanar(i)) {
            ++planarChecked;
            EXPECT_FALSE(wp.isFaceRevolution(i)) << "a planar cap is not a revolution face";
            EXPECT_FALSE(wp.faceRevolutionAxis(i).has_value());
        }
    }
    EXPECT_GE(planarChecked, 1) << "a cylinder has planar end caps to check";
}
