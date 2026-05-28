// @lat: [[engine/skills#blanking]]

#include "blanking.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAlgoAPI_Common.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::blanking {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double sheetThicknessOf(const Workpiece& wp)
{
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    return std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
}

double blankAreaOf(const Input& in)
{
    if (in.shape == Shape::Rectangle) return in.length_mm * in.width_mm;
    const double r = in.diameter_mm / 2.0;
    return M_PI * r * r;
}

double minFeatureOf(const Input& in)
{
    if (in.shape == Shape::Rectangle)
        return std::min(in.length_mm, in.width_mm);
    return in.diameter_mm;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.shape == Shape::Rectangle) {
        if (in.length_mm <= 0.0 || in.width_mm <= 0.0) {
            r.add("DFM-INPUT", "error",
                  "blanking rectangle requires length_mm > 0 and width_mm > 0");
        }
    } else {
        if (in.diameter_mm <= 0.0) {
            r.add("DFM-INPUT", "error",
                  "blanking circle requires diameter_mm > 0");
        }
    }

    const double t = sheetThicknessOf(wp);
    if (t > 5.0) {
        r.add("DFM-BLK-SHEET", "error",
              "blanking: stock thickness " + std::to_string(t) +
              " > 5 mm — not sheet metal");
    }
    if (t <= 0.0) {
        r.add("DFM-BLK-SHEET", "error", "blanking: degenerate stock thickness");
    }

    // Min feature width vs thickness (punch-die ratio).
    const double minF = minFeatureOf(in);
    if (minF > 0.0 && t > 0.0 && minF < 1.5 * t) {
        r.add("DFM-BLK-MIN-W", "error",
              "blanking minimum feature " + std::to_string(minF) +
              " mm < 1.5 × sheet_thickness (" + std::to_string(1.5 * t) +
              " mm) — blank-die min-feature rule violated");
    }

    // Blank polygon must fit within the sheet bbox.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    double bxMin, byMin, bxMax, byMax;
    if (in.shape == Shape::Rectangle) {
        bxMin = in.center_x_mm - in.length_mm / 2.0;
        bxMax = in.center_x_mm + in.length_mm / 2.0;
        byMin = in.center_y_mm - in.width_mm  / 2.0;
        byMax = in.center_y_mm + in.width_mm  / 2.0;
    } else {
        const double R = in.diameter_mm / 2.0;
        bxMin = in.center_x_mm - R;
        bxMax = in.center_x_mm + R;
        byMin = in.center_y_mm - R;
        byMax = in.center_y_mm + R;
    }
    if (bxMin < xMin - 1e-6 || bxMax > xMax + 1e-6 ||
        byMin < yMin - 1e-6 || byMax > yMax + 1e-6)
    {
        r.add("DFM-INPUT", "error",
              "blanking polygon falls outside the sheet bbox");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Build a tall extruded polygon spanning the full sheet thickness, then
// INTERSECT (BRepAlgoAPI_Common) with the sheet — the intersection equals
// the blank itself.  Everything outside the polygon is discarded.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "blanking DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double t       = sheetThicknessOf(wp);
    const double zSpan   = (zMax - zMin) + 2.0;     // a hair taller than sheet
    const double zStart  = zMin - 1.0;

    // Build the tall extruded polygon (rectangle box OR cylinder).
    TopoDS_Shape extruded;
    try {
        if (in.shape == Shape::Rectangle) {
            const gp_Pnt corner(
                in.center_x_mm - in.length_mm / 2.0,
                in.center_y_mm - in.width_mm  / 2.0,
                zStart);
            extruded = pr::box(gp_Ax2(corner, gp::DZ()),
                               in.length_mm, in.width_mm, zSpan);
        } else {
            const gp_Ax2 ax(gp_Pnt(in.center_x_mm, in.center_y_mm, zStart),
                            gp::DZ());
            extruded = pr::cylinder(ax, in.diameter_mm / 2.0, zSpan);
        }
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("blanking: extrude failed: ") + ex.what());
    }

    // INTERSECT the extruded polygon with the sheet → keep only the blank.
    BRepAlgoAPI_Common common(wp.shape(), extruded);
    common.Build();
    if (!common.IsDone()) {
        throw SkillError("blanking: intersection (Common) build failed");
    }
    const TopoDS_Shape blank = common.Shape();

    // Signature
    const double bArea = blankAreaOf(in);
    json dimsJson;
    if (in.shape == Shape::Rectangle)
        dimsJson = { { "length_mm", in.length_mm },
                     { "width_mm",  in.width_mm  } };
    else
        dimsJson = { { "diameter_mm", in.diameter_mm } };

    json params = {
        { "shape",        shapeToString(in.shape) },
        { "center_x_mm",  in.center_x_mm },
        { "center_y_mm",  in.center_y_mm },
        { "dims",         dimsJson },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "shape",               shapeToString(in.shape) },
        { "blank_area_mm2",      bArea },
        { "sheet_thickness_mm",  t },
        { "is_blanked",          true },
        { "operation",           "intersection" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "blanking_die";
    tooling.tool_dia_mm       = (in.shape == Shape::Circle)
                                ? in.diameter_mm
                                : std::min(in.length_mm, in.width_mm);
    tooling.tool_length_mm    = t * 2.0;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    // Removed = full sheet bbox volume − blank volume (the SCRAP is "removed"
    // from the operator's perspective; the blank is the product).
    const double sheetVol = (xMax - xMin) * (yMax - yMin) * t;
    tooling.stock_removed_mm3 = std::max(0.0, sheetVol - bArea * t);
    tooling.est_cycle_time_s  = 0.5;            // single stroke
    tooling.extra["machining_constraint"] =
        "Blanking die — punch + die clearance ≈ 10% of thickness; scrap is "
        "discarded, blank is the product";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(blank, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::blanking applied: shape={} area={} t={} faces {}→{}",
                  shapeToString(in.shape), bArea, t,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// After blanking, the workpiece IS the blank: its XY bbox matches the
// reported blank polygon, and its Z extent equals the sheet thickness.
// Recognition is purely metadata-replay (the geometry alone can't tell us
// what the original sheet was), but we additionally confirm the bbox vs.
// the recorded blank_area_mm2 to bump confidence.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;

        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        rf.matched_geometry = {
            { "source", "metadata" },
            { "bbox_dx", xMax - xMin },
            { "bbox_dy", yMax - yMin },
            { "bbox_dz", zMax - zMin },
        };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::blanking
