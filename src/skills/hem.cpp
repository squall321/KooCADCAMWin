// @lat: [[engine/skills#hem]]

#include "hem.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pln.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace koocadcam::skill::hem {

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

    if (in.hem_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "hem_width_mm must be > 0");
    }
    const double t = sheetThicknessOf(wp);
    if (t > 5.0) {
        r.add("DFM-H-SHEET", "error",
              "hem: stock thickness " + std::to_string(t) +
              " > 5 mm — not sheet metal");
    }
    if (t > 0.0 && in.hem_width_mm < 2.0 * t) {
        r.add("DFM-H-MIN-W", "error",
              "hem_width " + std::to_string(in.hem_width_mm) +
              " mm < 2 × sheet_thickness (" + std::to_string(2.0 * t) +
              " mm) — insufficient material to wrap");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// We approximate the hem as a thin extra slab of material added on top of
// the sheet along the chosen edge, of dimensions
// (hem_width × edge_length × sheet_thickness).  Its bottom face sits at
// the sheet's top plane (flat_hem=true) or above by sheet_thickness
// (flat_hem=false, open hem).

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "hem DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double t = sheetThicknessOf(wp);

    // Compute the slab corners based on edge_selector.
    double sxMin = xMin, sxMax = xMax, syMin = yMin, syMax = yMax;
    double edgeLength = 0.0;
    switch (in.edge_selector) {
        case EdgeSide::XMin:
            sxMax = xMin + in.hem_width_mm;
            edgeLength = yMax - yMin;
            break;
        case EdgeSide::XMax:
            sxMin = xMax - in.hem_width_mm;
            edgeLength = yMax - yMin;
            break;
        case EdgeSide::YMin:
            syMax = yMin + in.hem_width_mm;
            edgeLength = xMax - xMin;
            break;
        case EdgeSide::YMax:
            syMin = yMax - in.hem_width_mm;
            edgeLength = xMax - xMin;
            break;
    }

    // Z extents: top of sheet is zMax.
    //   flat_hem  → hem bottom = zMax, top = zMax + t (no gap)
    //   open hem  → hem bottom = zMax + t, top = zMax + 2t (gap = t)
    const double gap = in.flat_hem ? 0.0 : t;
    const double hemBottom = zMax + gap;

    const TopoDS_Shape hemSlab = pr::box(
        gp_Ax2(gp_Pnt(sxMin, syMin, hemBottom), gp::DZ()),
        sxMax - sxMin, syMax - syMin, t);

    const TopoDS_Shape newShape = pr::fuse(wp.shape(), hemSlab);

    // Signature
    json params = {
        { "edge_selector", edgeSideToString(in.edge_selector) },
        { "hem_width_mm",  in.hem_width_mm },
        { "flat_hem",      in.flat_hem },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "parallel_planar_face_count", 2 },
        { "separation_mm",              t + gap },   // distance between sheet top and hem top
        { "hem_width_mm",               in.hem_width_mm },
        { "edge_length_mm",             edgeLength },
        { "flat_hem",                   in.flat_hem },
        { "sheet_thickness_mm",         t },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "press_brake";
    tooling.tool_dia_mm       = 0.5 * t;   // typical hem die nose
    tooling.tool_length_mm    = edgeLength;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 3.0 + edgeLength * 0.05;  // 2 hits: bend + flatten
    tooling.extra["machining_constraint"] =
        "Hem (flat/open) — formed in two press-brake hits: 30° pre-bend, then "
        "flatten; flat hem requires zero gap, open hem leaves stock-thickness gap";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::hem applied: side={} width={} flat={} faces {}→{}",
                  edgeSideToString(in.edge_selector),
                  in.hem_width_mm, in.flat_hem,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Look for a thin parallel-planar pair near the sheet edge: i.e. two
// planar faces whose normals are ~ +Z, separated by a small Z distance
// (= sheet_thickness for flat hem, or 2 × sheet_thickness for open),
// neither of which covers the full sheet (the hem layer is local to
// one edge).
//
// Algorithm: find the LARGEST planar face with normal ~ +Z (the main
// sheet top), then find a SECOND planar face with normal ~ +Z whose
// center is at Z > sheet_top (above) and whose area is significantly
// smaller — that's the hem top.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const double t = sheetThicknessOf(wp);
    if (t <= 0.0 || t > 5.0) return out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // Find the largest +Z planar face on the original sheet plane (~ zMax - t).
    int mainTop = -1;
    double mainTopArea = -1.0;
    double mainTopZ = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.9) continue;
        const double area = wp.faceArea(i);
        if (area > mainTopArea) {
            mainTopArea = area;
            mainTop     = i;
            mainTopZ    = wp.faceCenter(i).Z();
        }
    }
    if (mainTop < 0) return out;

    // Find any other +Z planar face SIGNIFICANTLY above mainTopZ.
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (i == mainTop) continue;
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.9) continue;
        const double cZ   = wp.faceCenter(i).Z();
        const double sep  = cZ - mainTopZ;
        if (sep < t * 0.5) continue;       // not far enough above
        if (sep > 3.0 * t) continue;       // too far — not a hem

        const double area = wp.faceArea(i);
        if (area >= mainTopArea * 0.9) continue;   // not a small local feature

        // Determine edge_selector from face center XY.
        const gp_Pnt c = wp.faceCenter(i);
        std::string sel = "x_min";
        const double dxMin = c.X() - xMin;
        const double dxMax = xMax - c.X();
        const double dyMin = c.Y() - yMin;
        const double dyMax = yMax - c.Y();
        const double m = std::min({ dxMin, dxMax, dyMin, dyMax });
        if      (m == dxMin) sel = "x_min";
        else if (m == dxMax) sel = "x_max";
        else if (m == dyMin) sel = "y_min";
        else                  sel = "y_max";

        // Width: from the area and the appropriate edge length.
        double edgeLen = (sel == "x_min" || sel == "x_max") ? (yMax - yMin)
                                                            : (xMax - xMin);
        const double hem_width = (edgeLen > 1e-6) ? area / edgeLen : 0.0;
        const bool flatHem = (sep < 1.5 * t);

        json recovered = {
            { "edge_selector", sel },
            { "hem_width_mm",  hem_width },
            { "flat_hem",      flatHem },
        };
        json matched = {
            { "main_top_face_id", mainTop },
            { "hem_top_face_id",  i },
            { "separation_mm",    sep },
            { "hem_area_mm2",     area },
            { "sheet_thickness_mm", t },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.75, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::hem
