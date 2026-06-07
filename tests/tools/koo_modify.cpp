// @lat: [[engine/reverse-route#modify-cli]]
//
// koo_modify — recover a hole from a foreign STEP, change its diameter ON THE
// ORIGINAL geometry, and write the modified STEP.  Unlike re-executing a
// recovered plan (which keeps only recognized features), this preserves the
// whole imported part and edits ONE feature in place: it enlarges the chosen
// hole by cutting a larger co-axial cylinder along the hole's recovered axis.
//
//   Usage: koo_modify <in.step> <out.step> <tx> <ty> <tz> <new_dia_mm> [tol=15]
//
// Picks the recognized drill_hole / bore_cylindrical / counterbore whose entry
// is nearest (tx,ty,tz), reports its measured diameter, and enlarges it.

#include "io/StepIO.hpp"
#include "re/Recognizer.hpp"
#include "skills/Workpiece.hpp"

#include "engine/primitives/Tools.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

using namespace koocadcam;
namespace pr = koocadcam::engine::prim;

namespace {
double vol(const TopoDS_Shape& s)
{
    GProp_GProps g; BRepGProp::VolumeProperties(s, g); return g.Mass();
}
double num(const nlohmann::json& p, const char* k) {
    return (p.contains(k) && p[k].is_number()) ? p[k].get<double>() : 0.0;
}
}

int main(int argc, char* argv[])
{
    if (argc < 7) {
        std::cerr << "Usage: koo_modify <in.step> <out.step> <tx> <ty> <tz> <new_dia_mm> [tol=15]\n";
        return 2;
    }
    const std::string inPath = argv[1], outPath = argv[2];
    const double tx = std::atof(argv[3]), ty = std::atof(argv[4]), tz = std::atof(argv[5]);
    const double newDia = std::atof(argv[6]);
    const double tol = (argc >= 8) ? std::atof(argv[7]) : 15.0;

    std::string err;
    auto shape = io::StepIO::read(inPath, err);
    if (!shape) { std::cerr << "read failed: " << err << "\n"; return 1; }
    skill::Workpiece wp(*shape);

    double xMin,yMin,zMin,xMax,yMax,zMax;
    wp.boundingBox(xMin,yMin,zMin,xMax,yMax,zMax);
    const double diag = std::sqrt((xMax-xMin)*(xMax-xMin)+(yMax-yMin)*(yMax-yMin)+(zMax-zMin)*(zMax-zMin));

    // Find the recognized hole nearest the target.
    auto cands = re::dedupe(re::analyze(wp));
    const nlohmann::json* best = nullptr;
    std::string bestSkill;
    double bestD = tol;
    for (const auto& c : cands) {
        if (c.skill_id != "drill_hole" && c.skill_id != "bore_cylindrical" &&
            c.skill_id != "counterbore") continue;
        const auto& p = c.recovered_params;
        const double d = std::sqrt(std::pow(num(p,"position_x_mm")-tx,2) +
                                   std::pow(num(p,"position_y_mm")-ty,2) +
                                   std::pow(num(p,"position_z_mm")-tz,2));
        if (d <= bestD) { bestD = d; best = &p; bestSkill = c.skill_id; }
    }
    if (!best) { std::cerr << "no recognized hole within " << tol << "mm of target\n"; return 1; }

    const double oldDia = (best->contains("seat_dia_mm")) ? num(*best,"seat_dia_mm")
                                                          : num(*best,"diameter_mm");
    gp_Dir axisDir(0,0,-1);
    if (best->contains("axis_dir") && (*best)["axis_dir"].is_array() && (*best)["axis_dir"].size()==3) {
        auto a = (*best)["axis_dir"];
        const double ax=a[0].get<double>(), ay=a[1].get<double>(), az=a[2].get<double>();
        if (std::sqrt(ax*ax+ay*ay+az*az) > 1e-9) axisDir = gp_Dir(ax,ay,az);
    }
    const gp_Pnt entry(num(*best,"position_x_mm"), num(*best,"position_y_mm"), num(*best,"position_z_mm"));

    std::cout << "Matched " << bestSkill << " at (" << entry.X() << ", " << entry.Y()
              << ", " << entry.Z() << ")  dia=" << oldDia << " mm  (dist " << bestD << ")\n";
    if (newDia <= oldDia) {
        std::cerr << "koo_modify demo enlarges only; new_dia (" << newDia
                  << ") must exceed measured dia (" << oldDia << ")\n";
        return 1;
    }

    // Cut a larger co-axial cylinder spanning the part along the hole's axis.
    const gp_Pnt start(entry.X() - axisDir.X()*0.1*diag,
                       entry.Y() - axisDir.Y()*0.1*diag,
                       entry.Z() - axisDir.Z()*0.1*diag);
    const TopoDS_Shape cutter = pr::cylinder(gp_Ax2(start, axisDir), newDia/2.0, diag*1.2);
    const TopoDS_Shape result = pr::cut(*shape, cutter);

    const double v0 = vol(*shape), v1 = vol(result);
    skill::Workpiece wpNew(result);
    std::cout << "Enlarged hole " << oldDia << " -> " << newDia << " mm\n";
    std::cout << "faces " << wp.faceCount() << " -> " << wpNew.faceCount()
              << "   volume " << v0 << " -> " << v1
              << "   removed " << (v0 - v1) << " mm3\n";

    if (!io::StepIO::write(result, outPath, err)) {
        std::cerr << "write failed: " << err << "\n"; return 1;
    }
    std::cout << "wrote " << outPath << "\n";
    return 0;
}
