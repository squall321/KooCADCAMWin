// @lat: [[engine/skills#flange]]

#include "flange.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::flange {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double sheetThicknessOf(const Workpiece& wp)
{
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    return std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.flange_width_mm <= 0.0) {
        r.add("DFM-F-WIDTH", "error", "flange_width_mm must be > 0");
    }
    if (in.bend_radius_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "bend_radius_mm must be > 0");
    }

    const double t = sheetThicknessOf(wp);
    if (t > 5.0) {
        r.add("DFM-F-SHEET", "error",
              "flange: stock thickness " + std::to_string(t) +
              " > 5 mm — not sheet metal");
    }
    if (t > 0.0 && in.bend_radius_mm < 0.5 * t) {
        r.add("DFM-F-MIN-R", "error",
              "flange bend radius " + std::to_string(in.bend_radius_mm) +
              " mm < 0.5 × sheet_thickness (" + std::to_string(0.5 * t) +
              " mm)");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "flange DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double t       = sheetThicknessOf(wp);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    // Determine the strip cutter box and the bend axis based on edge_selector.
    // The strip is the band of the sheet within flange_width_mm of the chosen
    // edge.  The bend axis lies along the INNER boundary of that strip, at
    // the sheet's bottom Z (for an upward flange).
    //
    // After cutting, we rotate the strip by 90° around the bend axis.
    double stripXMin = xMin, stripXMax = xMax;
    double stripYMin = yMin, stripYMax = yMax;
    gp_Pnt bendAxisOrigin;
    gp_Dir bendAxisDir;
    double edgeLength = 0.0;

    switch (in.edge_selector) {
        case EdgeSide::XMin:
            stripXMax = xMin + in.flange_width_mm;
            bendAxisOrigin = gp_Pnt(stripXMax, yMin, zMin);
            bendAxisDir    = gp_Dir(0.0, 1.0, 0.0);
            edgeLength = yMax - yMin;
            break;
        case EdgeSide::XMax:
            stripXMin = xMax - in.flange_width_mm;
            bendAxisOrigin = gp_Pnt(stripXMin, yMin, zMin);
            bendAxisDir    = gp_Dir(0.0, 1.0, 0.0);
            edgeLength = yMax - yMin;
            break;
        case EdgeSide::YMin:
            stripYMax = yMin + in.flange_width_mm;
            bendAxisOrigin = gp_Pnt(xMin, stripYMax, zMin);
            bendAxisDir    = gp_Dir(1.0, 0.0, 0.0);
            edgeLength = xMax - xMin;
            break;
        case EdgeSide::YMax:
            stripYMin = yMax - in.flange_width_mm;
            bendAxisOrigin = gp_Pnt(xMin, stripYMin, zMin);
            bendAxisDir    = gp_Dir(1.0, 0.0, 0.0);
            edgeLength = xMax - xMin;
            break;
    }

    // 1. Cut the sheet into "main" (everything not in the strip) and "strip".
    //    For a cuboid-sheet stock, the strip is just the small box at the
    //    chosen edge.  Build BOTH parts as their own primitives (rather
    //    than computing one via Boolean), which is more robust.
    const TopoDS_Shape stripBox = pr::box(
        gp_Ax2(gp_Pnt(stripXMin, stripYMin, zMin), gp::DZ()),
        stripXMax - stripXMin, stripYMax - stripYMin, zMax - zMin);
    const TopoDS_Shape main_part = pr::cut(wp.shape(), stripBox);
    // strip_part: intersect sheet with stripBox.  For a cuboid sheet
    // entirely containing the strip box, the intersection equals stripBox.
    const TopoDS_Shape strip_part = stripBox;
    (void)bboxDiag;  // bboxDiag kept for potential future use

    // 2. Rotate the strip 90° around the bend axis.
    const gp_Ax1 bendAxis(bendAxisOrigin, bendAxisDir);
    // The rotation sign depends on which edge: for x_max we want the strip
    // to rotate so its outer edge goes up (+Z).  Using right-hand rule
    // around bendAxisDir = +Y at the inner boundary (lower-X side) of an
    // x_max flange, a rotation of +90° (CCW about +Y) moves +X → +Z, which
    // sends the outer strip edge upward.  For x_min the bend axis is on the
    // outer (lower-X) side of the strip, so we need -90° to fold upward.
    //
    // Generalized: positive angle works when the strip lies on the +(axis ×
    // strip-outward) side.  We compute a sign automatically.
    double angleSign = 1.0;
    switch (in.edge_selector) {
        // The bend axis lies at sheet bottom Z, along ±Y or ±X.  Right-hand
        // rule: rotation around +Y by +θ sends +X to -Z (downward) and
        // -X to +Z (upward).  To fold upward we therefore use:
        //   XMin (strip on -X side) :  +θ  (sends -X to +Z)
        //   XMax (strip on +X side) :  -θ  (sends +X to +Z)
        //   YMin (strip on -Y side, axis=+X): -θ around +X sends -Y to +Z
        //   YMax (strip on +Y side, axis=+X): +θ around +X sends +Y to +Z
        case EdgeSide::XMin: angleSign =  1.0; break;
        case EdgeSide::XMax: angleSign = -1.0; break;
        case EdgeSide::YMin: angleSign = -1.0; break;
        case EdgeSide::YMax: angleSign =  1.0; break;
    }
    gp_Trsf rot;
    rot.SetRotation(bendAxis, angleSign * M_PI / 2.0);
    BRepBuilderAPI_Transform xf(strip_part, rot, /*Copy=*/false);
    if (!xf.IsDone()) throw SkillError("flange: rotation failed");
    const TopoDS_Shape strip_rotated = xf.Shape();

    // 3. Build a quarter cylindrical bend zone.  We use a FULL annular
    //    ring (inner radius = bend_radius_mm, outer = bend_radius_mm + t)
    //    centred on the bend axis; the fuse with main + strip trims it.
    TopoDS_Shape bendWedge;
    try {
        gp_Dir bdir = bendAxisDir;
        gp_Dir xDir;
        // Build a perpendicular direction for gp_Ax2's X axis.
        if (std::abs(bdir.Dot(gp::DX())) < 0.9) xDir = gp::DX();
        else xDir = gp::DY();
        // Orthogonalize.
        gp_Vec v(xDir);
        gp_Vec b(bdir);
        v = v - b.Multiplied(v.Dot(b));
        if (v.Magnitude() < 1e-6) {
            v = gp_Vec(gp::DZ());
            v = v - b.Multiplied(v.Dot(b));
        }
        v.Normalize();
        xDir = gp_Dir(v);
        gp_Pnt origin = bendAxisOrigin;
        // Retract slightly along axis to ensure full overlap with strip.
        origin.SetX(origin.X() - bdir.X() * 0.1);
        origin.SetY(origin.Y() - bdir.Y() * 0.1);
        origin.SetZ(origin.Z() - bdir.Z() * 0.1);
        const gp_Ax2 ax(origin, bdir, xDir);
        const double H = edgeLength + 0.2;
        BRepPrimAPI_MakeCylinder outerC(ax, in.bend_radius_mm + t, H);
        outerC.Build();
        if (!outerC.IsDone()) throw Standard_Failure("flange: outer cyl failed");
        BRepPrimAPI_MakeCylinder innerC(ax, in.bend_radius_mm, H);
        innerC.Build();
        if (!innerC.IsDone()) throw Standard_Failure("flange: inner cyl failed");
        bendWedge = pr::cut(outerC.Shape(), innerC.Shape());
    } catch (const Standard_Failure& ex) {
        spdlog::warn("flange: bend-wedge construction failed: {}", ex.what());
        bendWedge = TopoDS_Shape();
    }

    // 4. Fuse.
    TopoDS_Shape result = pr::fuse(main_part, strip_rotated);
    if (!bendWedge.IsNull()) {
        try {
            result = pr::fuse(result, bendWedge);
        } catch (const Standard_Failure& ex) {
            spdlog::warn("flange: bend-wedge fuse failed: {}", ex.what());
        }
    }

    // Signature.
    const double flangeFlatArea = in.flange_width_mm * edgeLength;
    json params = {
        { "edge_selector",   edgeSideToString(in.edge_selector) },
        { "flange_width_mm", in.flange_width_mm },
        { "bend_radius_mm",  in.bend_radius_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "flat_face_count",       2 },
        { "cylindrical_bend_face", 1 },
        { "bend_radius_mm",        in.bend_radius_mm },
        { "flange_flat_area_mm2",  flangeFlatArea },
        { "edge_length_mm",        edgeLength },
        { "flange_width_mm",       in.flange_width_mm },
        { "sheet_thickness_mm",    t },
        { "bend_axis_dir",         { bendAxisDir.X(), bendAxisDir.Y(), bendAxisDir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "press_brake";
    tooling.tool_dia_mm       = in.bend_radius_mm * 2.0;
    tooling.tool_length_mm    = edgeLength;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 2.0 + edgeLength * 0.05;
    tooling.extra["machining_constraint"] =
        "Press-brake flange — limited by V-die opening; min flange length ≥ "
        "0.5 × V-die width";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::flange applied: side={} width={} R={} faces {}→{}",
                  edgeSideToString(in.edge_selector),
                  in.flange_width_mm, in.bend_radius_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// A flange is recognised by:
//   - 1 horizontal-axis cylindrical bend face (axis lies in XY)
//   - 2 adjacent planar faces, one of which has normal ~ ±Z (the main
//     sheet) and the other has normal ~ horizontal (the flange flat)
//
// Distinguish from a generic `bend` by the flange flat being MUCH smaller
// than the main flat (the strip width is a small fraction of the sheet).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const double t = sheetThicknessOf(wp);

    // Collect horizontal-axis cylindrical face candidates first; the bend
    // wedge leaves two (inner & outer), and we want only the inner one
    // (= bend_radius).
    struct Cand { int fIdx; double radius; gp_Ax1 axis; };
    std::vector<Cand> rawCands;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        BRepAdaptor_Surface surf(wp.face(fIdx));
        const gp_Cylinder cyl = surf.Cylinder();
        const gp_Ax1 axis = cyl.Axis();
        const gp_Dir adir = axis.Direction();
        if (std::abs(adir.Z()) > 0.3) continue;
        rawCands.push_back({ fIdx, cyl.Radius(), axis });
    }
    // Group by axis (same direction + nearby location), keep minimum radius.
    std::vector<bool> grouped(rawCands.size(), false);
    std::vector<Cand> filtered;
    for (size_t i = 0; i < rawCands.size(); ++i) {
        if (grouped[i]) continue;
        Cand best = rawCands[i];
        grouped[i] = true;
        for (size_t j = i + 1; j < rawCands.size(); ++j) {
            if (grouped[j]) continue;
            const gp_Dir d1 = best.axis.Direction();
            const gp_Dir d2 = rawCands[j].axis.Direction();
            if (std::abs(std::abs(d1.Dot(d2)) - 1.0) > 1e-3) continue;
            if (best.axis.Location().Distance(rawCands[j].axis.Location()) > 5.0)
                continue;
            grouped[j] = true;
            if (rawCands[j].radius < best.radius) best = rawCands[j];
        }
        filtered.push_back(best);
    }

    for (const auto& fc : filtered) {
        const int fIdx       = fc.fIdx;
        const double radius  = fc.radius;
        const gp_Ax1 axis    = fc.axis;
        const gp_Dir adir    = axis.Direction();

        // Collect ALL workpiece planar faces and find the LARGEST one with
        // ~Z normal (the main sheet flat) and the LARGEST one with horizontal
        // normal (the flange flat).  This is robust against the bend wedge
        // not being topologically connected to the slabs.
        int mainFlat = -1, flangeFlat = -1;
        double mainArea = -1.0, flangeArea = -1.0;
        for (int g = 0; g < wp.faceCount(); ++g) {
            if (g == fIdx) continue;
            if (!wp.isFacePlanar(g)) continue;
            gp_Dir n;
            try { n = wp.faceNormal(g); } catch (...) { continue; }
            const double a = wp.faceArea(g);
            if (std::abs(n.Z()) > 0.9) {
                if (a > mainArea) { mainArea = a; mainFlat = g; }
            } else if (std::abs(n.Z()) < 0.1) {
                if (a > flangeArea) { flangeArea = a; flangeFlat = g; }
            }
        }
        if (mainFlat < 0 || flangeFlat < 0) continue;

        const double smallArea = flangeArea;
        const double largeArea = mainArea;

        // Flange ratio: small / large must be < 0.5 (otherwise it's a
        // symmetric bend, not a flange).
        if (largeArea < 1e-6) continue;
        const double areaRatio = smallArea / largeArea;
        if (areaRatio > 0.5) continue;

        // Recover edge_selector from bend axis location/direction.
        // For axis ~ along +Y at x = xMin → edge_selector = x_min, etc.
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        std::string edgeSel = "x_min";
        if (std::abs(adir.Y()) > 0.9) {
            // Bend axis along Y → vertical XY-edge.
            if (axis.Location().X() < (xMin + xMax) * 0.5) edgeSel = "x_min";
            else edgeSel = "x_max";
        } else if (std::abs(adir.X()) > 0.9) {
            if (axis.Location().Y() < (yMin + yMax) * 0.5) edgeSel = "y_min";
            else edgeSel = "y_max";
        }

        // edge_length = larger bbox dim along the bend axis direction.
        double edgeLen = (std::abs(adir.Y()) > 0.5) ? (yMax - yMin) : (xMax - xMin);
        const double flange_width = (edgeLen > 1e-6) ? (smallArea / edgeLen) : 0.0;

        json recovered = {
            { "edge_selector",   edgeSel },
            { "flange_width_mm", flange_width },
            { "bend_radius_mm",  radius },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "main_flat_id",        mainFlat },
            { "flange_flat_id",      flangeFlat },
            { "main_flat_area",      largeArea },
            { "flange_flat_area",    smallArea },
            { "sheet_thickness_mm",  t },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.80, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::flange
