// @lat: [[engine/skills#piercing]]

#include "piercing.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepAdaptor_Curve.hxx>
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

namespace koocadcam::skill::piercing {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double sheetThicknessOf(const Workpiece& wp)
{
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    return std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
}

double minFeatureOf(const Input& in)
{
    if (in.shape == Shape::Circle) return in.diameter_mm;
    return std::min(in.length_mm, in.width_mm);
}

double effectiveClearance(const Input& in, double t)
{
    if (in.clearance_mm > 0.0) return in.clearance_mm;
    return 0.08 * t;     // default: 8% of thickness per side
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.shape == Shape::Circle && in.diameter_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "piercing circle requires diameter_mm > 0");
    }
    if (in.shape == Shape::Rectangle &&
        (in.length_mm <= 0.0 || in.width_mm <= 0.0))
    {
        r.add("DFM-INPUT", "error",
              "piercing rectangle requires length_mm > 0 and width_mm > 0");
    }

    const double t = sheetThicknessOf(wp);
    if (t > 5.0) {
        r.add("DFM-PRC-SHEET", "error",
              "piercing: stock thickness " + std::to_string(t) +
              " > 5 mm — not sheet metal");
    }
    if (t <= 0.0) {
        r.add("DFM-PRC-SHEET", "error", "piercing: degenerate stock thickness");
    }

    const double minF = minFeatureOf(in);
    if (minF > 0.0 && t > 0.0 && minF < t) {
        r.add("DFM-PRC-MIN-DIM", "error",
              "piercing min feature " + std::to_string(minF) +
              " mm < sheet_thickness (" + std::to_string(t) +
              " mm) — minimum hole rule violated");
    }

    if (in.clearance_mm > 0.0 && t > 0.0) {
        const double pct = in.clearance_mm / t;
        if (pct < 0.03 || pct > 0.15) {
            r.add("DFM-PRC-CLEARANCE", "error",
                  "piercing clearance " + std::to_string(in.clearance_mm) +
                  " mm = " + std::to_string(pct * 100.0) +
                  "% × thickness — outside [3%, 15%] range");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "piercing DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double t          = sheetThicknessOf(wp);
    const double clearance  = effectiveClearance(in, t);
    const double zSpan      = (zMax - zMin) + 0.2;
    const double zStart     = zMin - 0.1;

    // Build the punch.
    TopoDS_Shape punch;
    if (in.shape == Shape::Circle) {
        const gp_Ax2 ax(
            gp_Pnt(in.position_x_mm, in.position_y_mm, zStart), gp::DZ());
        punch = pr::cylinder(ax, in.diameter_mm / 2.0, zSpan);
    } else {
        const gp_Pnt corner(
            in.position_x_mm - in.length_mm / 2.0,
            in.position_y_mm - in.width_mm  / 2.0,
            zStart);
        punch = pr::box(gp_Ax2(corner, gp::DZ()),
                        in.length_mm, in.width_mm, zSpan);
    }

    const TopoDS_Shape result = pr::cut(wp.shape(), punch);

    // Removed volume.
    double removedVol = 0.0;
    if (in.shape == Shape::Circle) {
        const double R = in.diameter_mm / 2.0;
        removedVol = M_PI * R * R * t;
    } else {
        removedVol = in.length_mm * in.width_mm * t;
    }

    // Signature
    json dimsJson;
    if (in.shape == Shape::Circle)
        dimsJson = { { "diameter_mm", in.diameter_mm } };
    else
        dimsJson = { { "length_mm", in.length_mm },
                     { "width_mm",  in.width_mm  } };

    json params = {
        { "shape",         shapeToString(in.shape) },
        { "position_x_mm", in.position_x_mm },
        { "position_y_mm", in.position_y_mm },
        { "dims",          dimsJson },
        { "clearance_mm",  in.clearance_mm },
        { "axis_dir",      { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "shape",                      shapeToString(in.shape) },
        { "is_through_cut",             true },
        { "sheet_thickness_mm",         t },
        { "punch_die_clearance_mm",     clearance },
        { "clearance_pct_of_thickness", (t > 0.0) ? (clearance / t) * 100.0 : 0.0 },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "pierce_punch_die";
    tooling.tool_dia_mm       = (in.shape == Shape::Circle)
                                ? in.diameter_mm
                                : std::min(in.length_mm, in.width_mm);
    tooling.tool_length_mm    = t * 2.0;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = 0.5;
    tooling.extra["punch_die_clearance_mm"] = clearance;
    tooling.extra["machining_constraint"]   =
        "Piercing — punch + die clearance per side ≈ 8% of thickness; "
        "slug ejection on die side";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::piercing applied: shape={} t={} clearance={}",
                  shapeToString(in.shape), t, clearance);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Circular piercing leaves the same geometry as sheet_punch (1 cyl face
// spanning thickness).  We additionally try to recognize rectangular
// piercing via 4 vertical planar walls spanning thickness, but for slice-1
// we focus on circle and rely on metadata replay for rectangles.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const double t = sheetThicknessOf(wp);
    if (t <= 0.0 || t > 5.0) {
        // Still replay metadata signatures in case the geometry no longer
        // reads as a sheet (e.g., after subsequent ops).
        for (const auto& f : wp.features()) {
            if (f.skill_id != kSkillId) continue;
            out.push_back(RecognizedFeature{
                kSkillId, f.params, 1.0,
                json{{ "source", "metadata" }}});
        }
        return out;
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius   = cyl.Radius();
        const gp_Ax1 axis     = cyl.Axis();
        const gp_Dir adir     = axis.Direction();

        if (std::abs(adir.Z()) < 0.9) continue;

        // Collect bounding circle Z values.
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
        if (std::abs((zHigh - zLow) - t) > std::max(0.05, t * 0.05)) continue;

        json recovered = {
            { "shape",        "circle" },
            { "position_x_mm", axis.Location().X() },
            { "position_y_mm", axis.Location().Y() },
            { "dims",          { { "diameter_mm", 2.0 * radius } } },
            { "clearance_mm",  0.08 * t },
        };
        json matched = {
            { "cylindrical_face_id",  fIdx },
            { "sheet_thickness_mm",   t },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.80, matched });
    }

    // Also replay any metadata-recorded piercings (for rectangular variants).
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        const std::string sh = f.params.value("shape", std::string("circle"));
        if (sh == "rectangle") {
            out.push_back(RecognizedFeature{
                kSkillId, f.params, 1.0,
                json{{ "source", "metadata" }}});
        }
    }
    return out;
}

}  // namespace koocadcam::skill::piercing
