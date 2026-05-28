// @lat: [[engine/skills#emboss]]

#include "emboss.hpp"

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
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::emboss {

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

    if (in.height_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "emboss height_mm must be > 0");
    }
    if (in.shape == Shape::Circle && in.diameter_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "emboss circle requires diameter_mm > 0");
    }
    if (in.shape == Shape::Rectangle) {
        if (in.length_mm <= 0.0 || in.width_mm <= 0.0) {
            r.add("DFM-INPUT", "error",
                  "emboss rectangle requires length_mm > 0 and width_mm > 0");
        }
        if (in.corner_r_mm < 0.0) {
            r.add("DFM-INPUT", "error", "emboss corner_r_mm must be >= 0");
        }
    }

    const double t = sheetThicknessOf(wp);
    if (t > 5.0) {
        r.add("DFM-E-SHEET", "error",
              "emboss: stock thickness " + std::to_string(t) +
              " > 5 mm — not sheet metal");
    }

    if (in.height_mm > 0.0 && t > 0.0 && in.height_mm > 3.0 * t) {
        r.add("DFM-E-DEPTH", "error",
              "emboss height " + std::to_string(in.height_mm) +
              " mm > 3 × sheet_thickness (" + std::to_string(3.0 * t) +
              " mm) — deep-draw limit exceeded (tearing/thinning)");
    }
    if (in.shape == Shape::Rectangle && in.corner_r_mm > 0.0 &&
        t > 0.0 && in.corner_r_mm < t)
    {
        r.add("DFM-E-CORNER", "error",
              "emboss corner_r " + std::to_string(in.corner_r_mm) +
              " mm < sheet_thickness (" + std::to_string(t) +
              " mm) — sharp corner unformable");
    }
    // Rectangle: corner_r must be smaller than half the smaller side.
    if (in.shape == Shape::Rectangle && in.corner_r_mm > 0.0) {
        const double smaller = std::min(in.length_mm, in.width_mm);
        if (in.corner_r_mm >= smaller / 2.0) {
            r.add("DFM-INPUT", "error",
                  "emboss corner_r must be < min(length, width) / 2");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "emboss DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double t = sheetThicknessOf(wp);
    const double sheetTopZ = zMax;

    TopoDS_Shape bump;
    if (in.shape == Shape::Circle) {
        const gp_Ax2 ax(gp_Pnt(in.position_x_mm, in.position_y_mm, sheetTopZ),
                        gp::DZ());
        bump = pr::cylinder(ax, in.diameter_mm / 2.0, in.height_mm);
    } else {
        const gp_Pnt bottomCenter(in.position_x_mm, in.position_y_mm, sheetTopZ);
        bump = pr::roundedRectPocketTool(bottomCenter,
                                          in.length_mm, in.width_mm,
                                          in.height_mm, in.corner_r_mm);
    }

    const TopoDS_Shape newShape = pr::fuse(wp.shape(), bump);

    // Compute base footprint area.
    double footprintArea = 0.0;
    if (in.shape == Shape::Circle) {
        footprintArea = M_PI * (in.diameter_mm / 2.0) * (in.diameter_mm / 2.0);
    } else {
        // Approximate as rectangle area minus 4 corner cuts each of
        // (corner_r² × (1 - π/4)).  Ignore if corner_r = 0.
        const double r = in.corner_r_mm;
        footprintArea = in.length_mm * in.width_mm
                      - 4.0 * (r * r) * (1.0 - M_PI / 4.0);
    }

    // Signature
    json dimsJson;
    if (in.shape == Shape::Circle)
        dimsJson = { { "diameter_mm", in.diameter_mm } };
    else
        dimsJson = { { "length_mm", in.length_mm },
                     { "width_mm",  in.width_mm  } };
    json params = {
        { "position_x_mm", in.position_x_mm },
        { "position_y_mm", in.position_y_mm },
        { "shape",         shapeToString(in.shape) },
        { "dims",          dimsJson },
        { "height_mm",     in.height_mm },
        { "corner_r_mm",   in.corner_r_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "shape",                shapeToString(in.shape) },
        { "height_mm",            in.height_mm },
        { "footprint_area_mm2",   footprintArea },
        { "raised_face_present",  true },
        { "sheet_thickness_mm",   t },
        { "axis_perpendicular_z", true },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "stamping_die";
    tooling.tool_dia_mm       = (in.shape == Shape::Circle)
                                ? in.diameter_mm
                                : std::min(in.length_mm, in.width_mm);
    tooling.tool_length_mm    = in.height_mm;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;     // emboss displaces, does not remove
    tooling.est_cycle_time_s  = 0.5;     // single stamp
    tooling.extra["machining_constraint"] =
        "Emboss — formed by paired punch + die; height ≤ 3 × thickness "
        "to avoid tearing; sheet thins by ~ height/sheet_thickness in centre";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::emboss applied: shape={} height={} faces {}→{}",
                  shapeToString(in.shape), in.height_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Identify the largest +Z planar face (the sheet top) and find features
// that PROTRUDE above it:
//   - cylindrical face with vertical axis whose bottom edge sits at sheet
//     top Z → emboss "circle"
//   - cylindrical face (vertical axis) at a CORNER of a small rectangular
//     planar pad → emboss "rectangle" with corner_r_mm
//
// Confidence ~ 0.70 since rectangle reconstruction without the planar
// rectangle's exact bounds is heuristic.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const double t = sheetThicknessOf(wp);
    if (t <= 0.0 || t > 5.0) return out;

    // (bbox not required directly here — recognize uses face-area sorting)

    // Identify sheet top Z = the area-weighted-largest +Z planar face's Z.
    int mainTop = -1;
    double mainTopArea = -1.0;
    double mainTopZ = 0.0;
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

    // Scan cylindrical faces with vertical axis (~ ±Z) whose AXIAL SPAN
    // is small (~ height_mm) and whose BOTTOM Z lies near mainTopZ → that's
    // an emboss feature (circle).
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;

        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius   = cyl.Radius();
        const gp_Ax1 axis     = cyl.Axis();
        const gp_Dir adir     = axis.Direction();

        if (std::abs(adir.Z()) < 0.9) continue;   // need vertical axis

        // Determine axial span of the face via its bounding circular edges.
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

        // Emboss must protrude ABOVE main top — its lowest ring sits near
        // mainTopZ and its top ring is above.  Distinguish from sheet_punch
        // (which spans the full thickness, where both rings sit at the
        // sheet's top and bottom).
        if (std::abs(zLow - mainTopZ) > 0.05) continue;
        if (zHigh <= mainTopZ + 1e-3) continue;
        if (span > 3.0 * t + 0.5) continue;    // unreasonable height

        // Heuristic: detect rectangle (4 vertical corner cylinders all
        // sharing the same Z extent and Z bottom).  For slice-1 we report
        // each as a circle; the calling RE layer can group them.
        json recovered = {
            { "position_x_mm", axis.Location().X() },
            { "position_y_mm", axis.Location().Y() },
            { "shape",         "circle" },
            { "dims",          { { "diameter_mm", 2.0 * radius } } },
            { "height_mm",     span },
            { "corner_r_mm",   0.0 },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "axis_dir",            { adir.X(), adir.Y(), adir.Z() } },
            { "z_bottom",            zLow },
            { "z_top",               zHigh },
            { "sheet_thickness_mm",  t },
            { "main_top_face_id",    mainTop },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.70, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::emboss
