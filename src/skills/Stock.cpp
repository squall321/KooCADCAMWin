// @lat: [[engine/skills#Stock]]

#include "Stock.hpp"

#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

namespace koocadcam::skill {

namespace pr = koocadcam::engine::prim;

std::shared_ptr<Workpiece> createCuboidStock(double width_mm, double height_mm,
                                              double depth_mm,
                                              const std::string& material)
{
    const TopoDS_Shape shape = pr::box(
        gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp::DZ()),
        width_mm, height_mm, depth_mm);
    return std::make_shared<Workpiece>(shape, material);
}

std::shared_ptr<Workpiece> createCylindricalStock(double diameter_mm,
                                                    double thickness_mm,
                                                    const std::string& material)
{
    const TopoDS_Shape shape = pr::cylinder(
        gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp::DZ()),
        diameter_mm / 2.0, thickness_mm);
    return std::make_shared<Workpiece>(shape, material);
}

}  // namespace koocadcam::skill
