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

#include <BRep_Builder.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

using namespace koocadcam;
namespace pr = koocadcam::engine::prim;

namespace {

// Fuse `tool` into ONLY the solid(s) of `shape` whose AABB contains `p`, leaving
// every other solid untouched, then re-assemble the compound.  Fusing a tool
// into a whole multi-solid STEP assembly at once is fragile in OCCT; isolating
// the affected solid makes "shrink" (add material) work on real assemblies.
// If `shape` is a single solid, this is just one fuse.
TopoDS_Shape fuseIntoContainingSolid(const TopoDS_Shape& shape,
                                     const TopoDS_Shape& tool, const gp_Pnt& p)
{
    BRep_Builder bb;
    TopoDS_Compound out;
    bb.MakeCompound(out);
    bool fusedAny = false;
    int solids = 0;
    for (TopExp_Explorer ex(shape, TopAbs_SOLID); ex.More(); ex.Next()) {
        ++solids;
        const TopoDS_Solid& s = TopoDS::Solid(ex.Current());
        Bnd_Box box; BRepBndLib::Add(s, box);
        double xm,ym,zm,xx,yx,zx; box.Get(xm,ym,zm,xx,yx,zx);
        const double m = 1.0;  // small inflation
        const bool contains = p.X() >= xm-m && p.X() <= xx+m &&
                              p.Y() >= ym-m && p.Y() <= yx+m &&
                              p.Z() >= zm-m && p.Z() <= zx+m;
        if (contains) { bb.Add(out, pr::fuse(s, tool)); fusedAny = true; }
        else          { bb.Add(out, s); }
    }
    if (solids == 0) return pr::fuse(shape, tool);  // no solids → fuse whole
    if (!fusedAny)   return pr::fuse(shape, tool);  // point outside all → fallback
    return out;
}
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
    const skill::RecognizedFeature* bestC = nullptr;
    double bestD = tol;
    for (const auto& c : cands) {
        if (c.skill_id != "drill_hole" && c.skill_id != "bore_cylindrical" &&
            c.skill_id != "counterbore") continue;
        const auto& p = c.recovered_params;
        const double d = std::sqrt(std::pow(num(p,"position_x_mm")-tx,2) +
                                   std::pow(num(p,"position_y_mm")-ty,2) +
                                   std::pow(num(p,"position_z_mm")-tz,2));
        if (d <= bestD) { bestD = d; bestC = &c; }
    }
    if (!bestC) { std::cerr << "no recognized hole within " << tol << "mm of target\n"; return 1; }

    const nlohmann::json* best = &bestC->recovered_params;
    const std::string bestSkill = bestC->skill_id;
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
    if (std::abs(newDia - oldDia) < 1e-6) {
        std::cerr << "new_dia equals measured dia — nothing to change\n";
        return 1;
    }

    // Co-axial tool spanning the part along the hole's recovered axis.
    const gp_Pnt start(entry.X() - axisDir.X()*0.1*diag,
                       entry.Y() - axisDir.Y()*0.1*diag,
                       entry.Z() - axisDir.Z()*0.1*diag);
    const gp_Ax2 ax(start, axisDir);

    TopoDS_Shape result;
    try {
        if (newDia > oldDia) {
            // Enlarge: cut a larger co-axial cylinder.
            result = pr::cut(*shape, pr::cylinder(ax, newDia/2.0, diag*1.2));
        } else {
            // Shrink: fuse a co-axial annulus filling the ring (newR .. oldR),
            // leaving a smaller Ø(new_dia) hole.  Two robustness measures:
            //  - radial OVERLAP into the existing wall (oldR + overlap) so the
            //    union has no coincident cylindrical face (exact-radius fuse fails);
            //  - axial extent limited to the HOLE's length (entry → far_center,
            //    +0.5mm each side) so we don't fuse a giant tube protruding from
            //    the part — only the hole bore is filled.
            const double overlap = std::max(1.0, oldDia * 0.01);
            double len = diag * 1.2;            // fallback if far_center is absent
            gp_Pnt aStart = start;
            const auto& mg = bestC->matched_geometry;
            if (mg.is_object() && mg.contains("far_center") && mg["far_center"].is_array()
                && mg["far_center"].size() == 3) {
                const auto& fc = mg["far_center"];
                const gp_Pnt farP(fc[0].get<double>(), fc[1].get<double>(), fc[2].get<double>());
                len = entry.Distance(farP) + 1.0;
                aStart = gp_Pnt(entry.X() - axisDir.X()*0.5,
                                entry.Y() - axisDir.Y()*0.5,
                                entry.Z() - axisDir.Z()*0.5);
            }
            const TopoDS_Shape annulus = pr::annularRing(gp_Ax2(aStart, axisDir),
                                                         oldDia/2.0 + overlap, newDia/2.0, len);
            // Fuse only into the solid that holds this hole (robust on assemblies).
            result = fuseIntoContainingSolid(*shape, annulus, entry);
        }
    } catch (const Standard_Failure&) {
        std::cerr << "modify failed (OCCT boolean): "
                  << (newDia > oldDia ? "enlarge" : "shrink")
                  << " not feasible on this geometry — adding material into a "
                     "multi-solid assembly (shrink) is fragile; enlarge "
                     "(material removal) is robust\n";
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "modify failed: " << e.what() << "\n";
        return 1;
    }
    if (result.IsNull()) { std::cerr << "modify produced a null shape\n"; return 1; }

    const double v0 = vol(*shape), v1 = vol(result);
    skill::Workpiece wpNew(result);
    std::cout << (newDia > oldDia ? "Enlarged" : "Shrank") << " hole "
              << oldDia << " -> " << newDia << " mm\n";
    std::cout << "faces " << wp.faceCount() << " -> " << wpNew.faceCount()
              << "   volume " << v0 << " -> " << v1
              << "   delta " << (v1 - v0) << " mm3\n";

    if (!io::StepIO::write(result, outPath, err)) {
        std::cerr << "write failed: " << err << "\n"; return 1;
    }
    std::cout << "wrote " << outPath << "\n";
    return 0;
}
