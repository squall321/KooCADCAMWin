// @lat: [[engine/skills#stretching]]

#include "stretching.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::stretching {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

double sheetThickness(const Workpiece& wp)
{
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    return std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.stretch_radius_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "stretching stretch_radius_mm must be > 0");
    }
    if (in.target_thickness_reduction_pct <= 0.0) {
        r.add("DFM-INPUT", "error",
              "stretching target_thickness_reduction_pct must be > 0");
    }
    if (in.target_thickness_reduction_pct > 30.0) {
        r.add("DFM-ST-REDUCTION", "error",
              "stretching reduction " +
              std::to_string(in.target_thickness_reduction_pct) +
              "% > 30% — sheet tearing risk");
    }

    const double t = sheetThickness(wp);
    if (t > 5.0) {
        r.add("DFM-ST-SHEET", "error",
              "stretching: stock thickness " + std::to_string(t) +
              " > 5 mm — not sheet metal");
    }
    if (t > 0.0 && in.stretch_radius_mm > 0.0 &&
        in.stretch_radius_mm <= 5.0 * t)
    {
        r.add("DFM-ST-RADIUS", "error",
              "stretching radius " + std::to_string(in.stretch_radius_mm) +
              " mm <= 5 × sheet_thickness (" + std::to_string(5.0 * t) +
              " mm) — too localized to stretch without tearing");
    }

    // Stretched region must fit on the sheet.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double R = in.stretch_radius_mm;
    if (in.position_x_mm - R < xMin - 1e-6 ||
        in.position_x_mm + R > xMax + 1e-6 ||
        in.position_y_mm - R < yMin - 1e-6 ||
        in.position_y_mm + R > yMax + 1e-6)
    {
        r.add("DFM-INPUT", "error",
              "stretching region falls off the sheet bbox");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Cut a shallow cylindrical depression from the top face.  The depth
// equals (sheet_thickness × reduction_pct / 100).

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "stretching DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double t = sheetThickness(wp);
    const double sheetTopZ = zMax;
    const double depressionDepth =
        t * (in.target_thickness_reduction_pct / 100.0);
    const double kOverhang = 0.01;

    // Cylinder cutter — top a bit above sheetTopZ to ensure clean entry.
    const gp_Ax2 cutAx(
        gp_Pnt(in.position_x_mm, in.position_y_mm,
               sheetTopZ - depressionDepth),
        gp::DZ());
    TopoDS_Shape cutter;
    try {
        cutter = pr::cylinder(cutAx, in.stretch_radius_mm,
                              depressionDepth + kOverhang);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("stretching: cutter build failed: ") + ex.what());
    }
    TopoDS_Shape result;
    try {
        result = pr::cut(wp.shape(), cutter);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("stretching: cut failed: ") + ex.what());
    }

    // Signature.
    json params = {
        { "position_x_mm",                  in.position_x_mm },
        { "position_y_mm",                  in.position_y_mm },
        { "stretch_radius_mm",              in.stretch_radius_mm },
        { "target_thickness_reduction_pct", in.target_thickness_reduction_pct },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "stretch_radius_mm",          in.stretch_radius_mm },
        { "depression_depth_mm",        depressionDepth },
        { "sheet_thickness_mm",         t },
        { "axis_perpendicular_z",       true },
        { "shape",                      "circular_depression" },
        { "reduction_pct",              in.target_thickness_reduction_pct },
    };
    ToolingMeta tooling;
    // Geometric pocket volume removed = π R² × depression_depth.  This is the
    // bbox-delta the cut leaves on the workpiece; the physical forming process
    // displaces (not removes) this material, but the geometric model records
    // the cut volume.
    const double pocketVolume =
        3.14159265358979323846 * in.stretch_radius_mm * in.stretch_radius_mm
        * depressionDepth;
    tooling.tool_type         = "stretch_form_die";
    tooling.tool_dia_mm       = 2.0 * in.stretch_radius_mm;
    tooling.tool_length_mm    = depressionDepth + 2.0;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = pocketVolume;  // geometric pocket volume (mm³)
    tooling.est_cycle_time_s  = 1.0 + depressionDepth * 0.2;
    tooling.extra["machining_constraint"] =
        "Stretching — sheet held by clamps then drawn over a form block; "
        "uniform biaxial tension required to avoid wrinkling";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::stretching applied: R={} depth={} reduction={}%",
                  in.stretch_radius_mm, depressionDepth,
                  in.target_thickness_reduction_pct);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Identify the largest +Z planar face (sheet top).  Find cylindrical
// faces with vertical axis whose bounding circles sit BELOW that top
// (i.e. they describe a depression into the sheet).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const double t = sheetThickness(wp);
    if (t <= 0.0 || t > 5.0) return out;

    // Find sheet top Z = largest +Z planar face's Z.
    int    mainTop     = -1;
    double mainTopArea = -1.0;
    double mainTopZ    = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.9) continue;
        const double a = wp.faceArea(i);
        if (a > mainTopArea) {
            mainTopArea = a;
            mainTop     = i;
            mainTopZ    = wp.faceCenter(i).Z();
        }
    }
    if (mainTop < 0) return out;

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;

        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius   = cyl.Radius();
        const gp_Ax1  axis    = cyl.Axis();
        const gp_Dir  adir    = axis.Direction();

        if (std::abs(adir.Z()) < 0.9) continue;  // need vertical axis

        // Collect bounding circle Zs.
        std::vector<double> zCircles;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            zCircles.push_back(crv.Circle().Location().Z());
        }
        if (zCircles.size() < 2) continue;
        const double zLow  = *std::min_element(zCircles.begin(), zCircles.end());
        const double zHigh = *std::max_element(zCircles.begin(), zCircles.end());
        const double span  = zHigh - zLow;

        // Stretching: depression INTO the sheet — top ring near mainTopZ,
        // bottom ring BELOW mainTopZ by depression_depth (≤ 30% of t).
        if (std::abs(zHigh - mainTopZ) > 0.05) continue;
        if (zLow >= mainTopZ - 1e-3) continue;
        if (span > 0.3 * t + 0.05) continue;   // > 30% reduction → not stretching

        const double depressionDepth = span;
        const double reductionPct = (t > 0.0)
                                    ? (depressionDepth / t) * 100.0 : 0.0;

        json recovered = {
            { "position_x_mm",                   axis.Location().X() },
            { "position_y_mm",                   axis.Location().Y() },
            { "stretch_radius_mm",               radius },
            { "target_thickness_reduction_pct",  reductionPct },
        };
        json matched = {
            { "cylindrical_face_id",  fIdx },
            { "z_top",                zHigh },
            { "z_bottom",             zLow },
            { "sheet_thickness_mm",   t },
            { "main_top_face_id",     mainTop },
        };
        // Confidence depends on the radius / thickness ratio (DFM rule).
        double confidence = 0.70;
        if (radius <= 5.0 * t) confidence -= 0.2;  // below DFM-ST-RADIUS
        confidence = std::clamp(confidence, 0.1, 0.95);
        out.push_back(RecognizedFeature{ kSkillId, recovered, confidence, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::stretching
