// @lat: [[engine/reverse-route#modify-cli]]
//
// koo_modify — recover a hole from a foreign STEP and edit it ON THE
// ORIGINAL geometry: resize and/or MOVE it, preserving the rest of the part.
//
//   Usage: koo_modify <in.step> <out.step> <tx> <ty> <tz> <new_dia_mm>
//                     [tol=15] [--move <nx> <ny>]
//
// Picks the recognized single-cylinder hole (drill_hole / bore_cylindrical /
// mill_circular_pocket / drill_through_hole) nearest (tx,ty,tz), then edits
// it through edit::editHole (plan B4.1): BRepAlgoAPI_Defeaturing removes the
// hole's faces and HEALS the solid — restoring the material — and the new
// hole is cut fresh at the requested diameter/position.  Enlarge, shrink and
// move are therefore all MATERIAL-REMOVAL operations; no fill geometry, no
// fuse, no far_center heuristic (the old fuse path once synthesized a
// 1046 mm fill tube from a mis-recovered length — that failure mode no
// longer exists).  Works on multi-solid assemblies for the same reason.

#include "edit/FeatureEditor.hpp"
#include "io/StepIO.hpp"
#include "re/Recognizer.hpp"
#include "skills/Workpiece.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

using namespace koocadcam;

namespace {

double vol(const TopoDS_Shape& s)
{
    GProp_GProps g;
    BRepGProp::VolumeProperties(s, g);
    return g.Mass();
}

double num(const nlohmann::json& p, const char* k)
{
    return (p.contains(k) && p[k].is_number()) ? p[k].get<double>() : 0.0;
}

bool isEditableHole(const std::string& id)
{
    return id == "drill_hole" || id == "bore_cylindrical" ||
           id == "mill_circular_pocket" || id == "drill_through_hole";
}

}  // namespace

int main(int argc, char* argv[])
{
    if (argc < 7) {
        std::cerr << "Usage: koo_modify <in.step> <out.step> <tx> <ty> <tz> "
                     "<new_dia_mm> [tol=15] [--move <nx> <ny>]\n";
        return 2;
    }
    const std::string inPath = argv[1], outPath = argv[2];
    const double tx = std::atof(argv[3]), ty = std::atof(argv[4]),
                 tz = std::atof(argv[5]);
    const double newDia = std::atof(argv[6]);

    double tol     = 15.0;
    bool   hasMove = false;
    double nx = 0.0, ny = 0.0;
    for (int a = 7; a < argc; ++a) {
        const std::string arg = argv[a];
        if (arg == "--move" && a + 2 < argc) {
            nx      = std::atof(argv[++a]);
            ny      = std::atof(argv[++a]);
            hasMove = true;
        } else if (!arg.empty() && arg[0] != '-') {
            tol = std::atof(arg.c_str());
        } else {
            std::cerr << "unknown option: " << arg << "\n";
            return 2;
        }
    }

    std::string err;
    auto shape = io::StepIO::read(inPath, err);
    if (!shape) { std::cerr << "read failed: " << err << "\n"; return 1; }
    skill::Workpiece wp(*shape);

    // Find the recognized editable hole nearest the target.
    auto cands = re::dedupe(re::analyze(wp));
    const skill::RecognizedFeature* bestC = nullptr;
    double bestD = tol;
    for (const auto& c : cands) {
        if (!isEditableHole(c.skill_id)) continue;
        const auto& p = c.recovered_params;
        const double d = std::sqrt(std::pow(num(p, "position_x_mm") - tx, 2) +
                                   std::pow(num(p, "position_y_mm") - ty, 2) +
                                   std::pow(num(p, "position_z_mm") - tz, 2));
        if (d <= bestD) { bestD = d; bestC = &c; }
    }
    if (!bestC) {
        std::cerr << "no recognized single-cylinder hole within " << tol
                  << "mm of target (counterbore-family editing lands with "
                     "plan B4.2)\n";
        return 1;
    }

    auto oldHole = edit::holeSpecFromRecovered(bestC->recovered_params);
    if (!oldHole) {
        std::cerr << "recovered params incomplete — cannot edit\n";
        return 1;
    }

    edit::HoleSpec newHole = *oldHole;
    newHole.diameter_mm    = newDia;
    if (hasMove) {
        newHole.entry = gp_Pnt(nx, ny, oldHole->entry.Z());
    }

    std::cout << "Matched " << bestC->skill_id << " at ("
              << oldHole->entry.X() << ", " << oldHole->entry.Y() << ", "
              << oldHole->entry.Z() << ")  dia=" << oldHole->diameter_mm
              << " mm  (dist " << bestD << ")\n";

    const bool sameDia = std::abs(newDia - oldHole->diameter_mm) < 1e-6;
    if (sameDia && !hasMove) {
        std::cerr << "new_dia equals measured dia and no --move — nothing "
                     "to change\n";
        return 1;
    }

    const TopoDS_Shape result = edit::editHole(*shape, *oldHole, newHole, err);
    if (result.IsNull()) {
        std::cerr << "edit failed: " << err << "\n";
        return 1;
    }

    const double v0 = vol(*shape), v1 = vol(result);
    skill::Workpiece wpNew(result);
    std::cout << "Edited hole: dia " << oldHole->diameter_mm << " -> "
              << newDia << " mm";
    if (hasMove)
        std::cout << ", moved (" << oldHole->entry.X() << ", "
                  << oldHole->entry.Y() << ") -> (" << nx << ", " << ny << ")";
    std::cout << "\nfaces " << wp.faceCount() << " -> " << wpNew.faceCount()
              << "   volume " << v0 << " -> " << v1 << "   delta "
              << (v1 - v0) << " mm3\n";

    if (!io::StepIO::write(result, outPath, err)) {
        std::cerr << "write failed: " << err << "\n";
        return 1;
    }
    std::cout << "wrote " << outPath << "\n";
    return 0;
}
