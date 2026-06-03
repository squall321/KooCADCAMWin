// Diagnostic probe (temporary)
#include <gtest/gtest.h>
#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <iostream>

using namespace koocadcam;
namespace pr = engine::prim;

namespace { double V(const TopoDS_Shape& s){ GProp_GProps p; BRepGProp::VolumeProperties(s,p); return p.Mass(); } }

TEST(Probe, AnnularRingAlone) {
    auto stock = skill::createCylindricalStock(40.0, 8.0);
    std::cout << "stock vol = " << V(stock->shape()) << "\n";
    const gp_Ax2 ax(gp_Pnt(0,0,8.05), gp_Dir(0,0,-1));
    auto ring = pr::annularRing(ax, 19.0, 16.0, 1.55);
    std::cout << "ring vol = " << V(ring) << "\n";
    auto cut = pr::cut(stock->shape(), ring);
    std::cout << "after-cut vol = " << V(cut) << "\n";
}

TEST(Probe, AnnularRingPlusOneNotch) {
    auto stock = skill::createCylindricalStock(40.0, 8.0);
    const gp_Ax2 ax(gp_Pnt(0,0,8.05), gp_Dir(0,0,-1));
    auto ring = pr::annularRing(ax, 19.0, 16.0, 1.55);
    const gp_Ax2 nax(gp_Pnt(19, 0, 8.05), gp_Dir(0,0,-1));
    auto notch = pr::cylinder(nax, 0.3, 0.55);
    std::cout << "notch vol = " << V(notch) << "\n";
    std::vector<TopoDS_Shape> cutters = { ring, notch };
    auto cut = pr::cutMany(stock->shape(), cutters);
    std::cout << "after-cut vol (1 notch) = " << V(cut) << "\n";
}
