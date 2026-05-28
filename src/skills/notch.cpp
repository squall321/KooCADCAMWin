// @lat: [[engine/skills#notch]]

#include "notch.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::notch {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double sheetThicknessOf(const Workpiece& wp)
{
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    return std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
}

// Build a triangular-prism cutter (footprint = isoceles triangle with mouth
// = `width_mm` and apex `depth_mm` inward from the edge).  Origin is the
// midpoint of the mouth on the edge.  Cutter extends from sheet bottom z
// to sheet top z (full through).
//
// `tangent` is unit vector along the edge (mouth direction); `inward` is
// unit vector perpendicular pointing INTO the sheet.
TopoDS_Shape buildTriangleCutter(const gp_Pnt& mouthMid,
                                  const gp_Vec& tangent,
                                  const gp_Vec& inward,
                                  double width_mm, double depth_mm,
                                  double zBot, double zTop)
{
    const double halfW = width_mm * 0.5;
    // Three corner points at sheet bottom z; we extrude upward by (zTop - zBot).
    gp_Pnt p1(mouthMid.X() - halfW * tangent.X(),
              mouthMid.Y() - halfW * tangent.Y(),
              zBot - 0.05);
    gp_Pnt p2(mouthMid.X() + halfW * tangent.X(),
              mouthMid.Y() + halfW * tangent.Y(),
              zBot - 0.05);
    gp_Pnt p3(mouthMid.X() + depth_mm * inward.X(),
              mouthMid.Y() + depth_mm * inward.Y(),
              zBot - 0.05);
    BRepBuilderAPI_MakeEdge e1(p1, p2);
    BRepBuilderAPI_MakeEdge e2(p2, p3);
    BRepBuilderAPI_MakeEdge e3(p3, p1);
    if (!e1.IsDone() || !e2.IsDone() || !e3.IsDone())
        throw Standard_Failure("notch: triangle edge build failed");
    BRepBuilderAPI_MakeWire wire(e1.Edge(), e2.Edge(), e3.Edge());
    if (!wire.IsDone()) throw Standard_Failure("notch: triangle wire build failed");
    BRepBuilderAPI_MakeFace face(wire.Wire(), /*OnlyPlane=*/true);
    if (!face.IsDone()) throw Standard_Failure("notch: triangle face build failed");
    const gp_Vec extrude(0.0, 0.0, (zTop - zBot) + 0.1);
    BRepPrimAPI_MakePrism prism(face.Face(), extrude);
    prism.Build();
    if (!prism.IsDone()) throw Standard_Failure("notch: triangle prism build failed");
    return prism.Shape();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.width_mm <= 0.0)
        r.add("DFM-INPUT", "error", "notch width_mm must be > 0");
    if (in.depth_mm <= 0.0)
        r.add("DFM-INPUT", "error", "notch depth_mm must be > 0");
    if (in.position_mm < 0.0)
        r.add("DFM-INPUT", "error", "notch position_mm must be >= 0");

    const double t = sheetThicknessOf(wp);
    if (t > 5.0) {
        r.add("DFM-N-SHEET", "error",
              "notch: stock thickness " + std::to_string(t) +
              " > 5 mm — not sheet metal");
    }
    if (in.width_mm > 0.0 && t > 0.0 && in.width_mm < 1.5 * t) {
        r.add("DFM-N-MIN-W", "error",
              "notch width " + std::to_string(in.width_mm) +
              " mm < 1.5 × sheet_thickness (" + std::to_string(1.5 * t) +
              " mm) — too narrow to punch reliably");
    }
    if (in.depth_mm > 0.0 && t > 0.0 && in.depth_mm < t) {
        r.add("DFM-N-MIN-D", "error",
              "notch depth " + std::to_string(in.depth_mm) +
              " mm < sheet_thickness (" + std::to_string(t) +
              " mm) — sub-thickness depth not manufacturable as a notch");
    }

    // Depth must not exceed 50% of the perpendicular sheet dimension.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    double perpDim = 0.0;
    switch (in.edge_selector) {
        case EdgeSide::XMin:
        case EdgeSide::XMax:
            perpDim = xMax - xMin; break;
        case EdgeSide::YMin:
        case EdgeSide::YMax:
            perpDim = yMax - yMin; break;
    }
    if (in.depth_mm > 0.5 * perpDim) {
        r.add("DFM-N-MAX-D", "error",
              "notch depth " + std::to_string(in.depth_mm) +
              " mm > 50% of perpendicular sheet dimension (" +
              std::to_string(perpDim) + " mm)");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "notch DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double t = sheetThicknessOf(wp);
    (void)t;

    // Determine mouth midpoint XY, tangent along the edge, and inward normal.
    gp_Pnt mouthMid;
    gp_Vec tangent(0.0, 0.0, 0.0);
    gp_Vec inward (0.0, 0.0, 0.0);
    switch (in.edge_selector) {
        case EdgeSide::XMin:
            mouthMid = gp_Pnt(xMin, yMin + in.position_mm, zMin);
            tangent  = gp_Vec(0.0, 1.0, 0.0);
            inward   = gp_Vec(1.0, 0.0, 0.0);
            break;
        case EdgeSide::XMax:
            mouthMid = gp_Pnt(xMax, yMin + in.position_mm, zMin);
            tangent  = gp_Vec(0.0, 1.0, 0.0);
            inward   = gp_Vec(-1.0, 0.0, 0.0);
            break;
        case EdgeSide::YMin:
            mouthMid = gp_Pnt(xMin + in.position_mm, yMin, zMin);
            tangent  = gp_Vec(1.0, 0.0, 0.0);
            inward   = gp_Vec(0.0, 1.0, 0.0);
            break;
        case EdgeSide::YMax:
            mouthMid = gp_Pnt(xMin + in.position_mm, yMax, zMin);
            tangent  = gp_Vec(1.0, 0.0, 0.0);
            inward   = gp_Vec(0.0, -1.0, 0.0);
            break;
    }

    TopoDS_Shape cutter;
    if (in.shape == Shape::Rectangle) {
        // Box centred at mouthMid, extending `depth_mm` inward and `width_mm`
        // along tangent; extruded `(zMax - zMin) + 0.2` along Z.  We slightly
        // overhang along the inward and outward sides so the boolean cleanly
        // bites through the sheet's outer edge.
        const double halfW = in.width_mm * 0.5;
        // Box origin (corner at -tangent halfW, slightly outside the sheet edge).
        const double kOverhang = 0.1;
        const gp_Pnt boxOrigin(
            mouthMid.X() - halfW * tangent.X() - kOverhang * inward.X(),
            mouthMid.Y() - halfW * tangent.Y() - kOverhang * inward.Y(),
            zMin - 0.05);
        // OCCT box frame: mainDir = +Z, xDir = inward.  Then dx = depth (inward),
        // dy = width along tangent, dz = thickness.
        const gp_Dir mainZ(0.0, 0.0, 1.0);
        const gp_Dir xDir(inward.X(), inward.Y(), inward.Z());
        const gp_Ax2 ax(boxOrigin, mainZ, xDir);
        cutter = pr::box(ax, in.depth_mm + 2.0 * kOverhang, in.width_mm,
                         (zMax - zMin) + 0.1);
    } else {
        // Triangle.
        cutter = buildTriangleCutter(mouthMid, tangent, inward,
                                      in.width_mm, in.depth_mm,
                                      zMin, zMax);
    }

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // Removed volume estimate.
    double removedVol = 0.0;
    if (in.shape == Shape::Rectangle) {
        removedVol = in.width_mm * in.depth_mm * (zMax - zMin);
    } else {
        removedVol = 0.5 * in.width_mm * in.depth_mm * (zMax - zMin);
    }

    // Signature
    json params = {
        { "edge_selector", edgeSideToString(in.edge_selector) },
        { "shape",         shapeToString(in.shape) },
        { "position_mm",   in.position_mm },
        { "width_mm",      in.width_mm },
        { "depth_mm",      in.depth_mm },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "shape",                  shapeToString(in.shape) },
        { "edge_selector",          edgeSideToString(in.edge_selector) },
        { "width_mm",               in.width_mm },
        { "depth_mm",               in.depth_mm },
        { "removed_volume_mm3",     removedVol },
        { "sheet_thickness_mm",     sheetThicknessOf(wp) },
        { "is_through_cut",         true },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "punch_press";
    tooling.tool_dia_mm       = in.width_mm;
    tooling.tool_length_mm    = sheetThicknessOf(wp) * 2.0;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = 0.5;
    tooling.extra["machining_constraint"] =
        "Notch — punch-press with shaped die or laser/waterjet trim; "
        "min width ≥ 1.5 × thickness; depth ≥ thickness";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::notch applied: shape={} edge={} w={} d={} faces {}→{}",
                  shapeToString(in.shape),
                  edgeSideToString(in.edge_selector),
                  in.width_mm, in.depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// A notch produces vertical-walled planar faces lying INSIDE the sheet's
// bbox by at most `depth_mm` from an edge.  We scan the workpiece's planar
// faces with horizontal normals, and flag those whose centroid sits inside
// the bbox by less than (sheet's smaller perpendicular dim × 0.5).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const double t = sheetThicknessOf(wp);
    if (t <= 0.0 || t > 5.0) return out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // Look for horizontal-normal planar faces whose centre is INSET from
    // the bbox (i.e. they form the inside wall of a notch).  Group by the
    // closest bbox edge.
    struct Wall { int fIdx; gp_Pnt center; gp_Dir normal; double area; };
    std::vector<Wall> walls;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (std::abs(n.Z()) > 0.1) continue;  // not vertical
        walls.push_back({ i, wp.faceCenter(i), n, wp.faceArea(i) });
    }
    if (walls.empty()) return out;

    // For each wall face, check if it's NOT on the outer bbox surface (i.e.
    // inset by some amount).  If so, treat as a notch wall.
    for (const auto& w : walls) {
        const gp_Pnt c = w.center;
        const double dxMin = c.X() - xMin;
        const double dxMax = xMax - c.X();
        const double dyMin = c.Y() - yMin;
        const double dyMax = yMax - c.Y();
        const double m = std::min({ dxMin, dxMax, dyMin, dyMax });
        // Only walls inset from an edge by 0 (outer wall) are skipped.
        // We want walls inset by > 0 but < 30% of perpendicular dim.
        if (m <= 0.01) continue;
        // The wall normal should be perpendicular to the closest edge.
        std::string edgeSel;
        double perpDim = 0.0;
        if (m == dxMin) { edgeSel = "x_min"; perpDim = xMax - xMin; }
        else if (m == dxMax) { edgeSel = "x_max"; perpDim = xMax - xMin; }
        else if (m == dyMin) { edgeSel = "y_min"; perpDim = yMax - yMin; }
        else                  { edgeSel = "y_max"; perpDim = yMax - yMin; }

        if (m > 0.5 * perpDim) continue;   // too far in to be a notch

        // Approximate width = face area / sheet thickness.
        const double widthEst = w.area / (zMax - zMin);
        const double depthEst = m;

        // Determine shape from wall normal direction relative to edge.
        // For a Rectangle notch the walls are AXIS-ALIGNED (normal along
        // ±X or ±Y).  For a Triangle the side walls are skewed (normal
        // has both X and Y components).
        std::string shapeStr = "rectangle";
        if (std::abs(w.normal.X()) > 0.01 && std::abs(w.normal.Y()) > 0.01)
            shapeStr = "triangle";

        json recovered = {
            { "edge_selector", edgeSel },
            { "shape",         shapeStr },
            { "position_mm",   (edgeSel == "x_min" || edgeSel == "x_max")
                                 ? (c.Y() - yMin)
                                 : (c.X() - xMin) },
            { "width_mm",      widthEst },
            { "depth_mm",      depthEst },
        };
        json matched = {
            { "wall_face_id",        w.fIdx },
            { "wall_center",         { c.X(), c.Y(), c.Z() } },
            { "sheet_thickness_mm",  t },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.70, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::notch
