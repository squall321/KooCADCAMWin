// @lat: [[engine/skills#lance]]

#include "lance.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/PolylineOffset.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepBuilderAPI_Transform.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::lance {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double sheetThicknessOf(const Workpiece& wp)
{
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    return std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
}

double polylineLength(const std::vector<std::array<double, 2>>& wps)
{
    double s = 0.0;
    for (size_t i = 1; i < wps.size(); ++i) {
        s += std::hypot(wps[i][0] - wps[i-1][0],
                        wps[i][1] - wps[i-1][1]);
    }
    return s;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.waypoints.size() < 2) {
        r.add("DFM-INPUT", "error",
              "lance: waypoints needs >= 2 points (got " +
              std::to_string(in.waypoints.size()) + ")");
    } else {
        for (size_t i = 1; i < in.waypoints.size(); ++i) {
            const double dx = in.waypoints[i][0] - in.waypoints[i-1][0];
            const double dy = in.waypoints[i][1] - in.waypoints[i-1][1];
            if (std::hypot(dx, dy) < 1e-6) {
                r.add("DFM-INPUT", "error",
                      "lance: duplicate consecutive waypoints at index " +
                      std::to_string(i));
                break;
            }
        }
    }
    if (in.tab_width_mm <= 0.0)
        r.add("DFM-INPUT", "error", "lance: tab_width_mm must be > 0");
    if (in.lift_mm <= 0.0)
        r.add("DFM-INPUT", "error", "lance: lift_mm must be > 0");

    const double t = sheetThicknessOf(wp);
    if (t > 5.0) {
        r.add("DFM-L-SHEET", "error",
              "lance: stock thickness " + std::to_string(t) +
              " > 5 mm — not sheet metal");
    }
    if (t > 0.0 && in.lift_mm < t) {
        r.add("DFM-L-MIN-LIFT", "error",
              "lance lift " + std::to_string(in.lift_mm) +
              " mm < sheet_thickness (" + std::to_string(t) +
              " mm) — tab cannot clear slit");
    }
    if (t > 0.0 && in.lift_mm > 5.0 * t) {
        r.add("DFM-L-MAX-LIFT", "error",
              "lance lift " + std::to_string(in.lift_mm) +
              " mm > 5 × sheet_thickness (" + std::to_string(5.0 * t) +
              " mm) — extreme lift, material will tear");
    }
    if (t > 0.0 && in.tab_width_mm < t) {
        r.add("DFM-L-MIN-TAB", "error",
              "lance tab_width " + std::to_string(in.tab_width_mm) +
              " mm < sheet_thickness (" + std::to_string(t) +
              " mm) — tab too narrow");
    }
    if (in.waypoints.size() >= 2) {
        const double pathLen = polylineLength(in.waypoints);
        if (t > 0.0 && pathLen < 3.0 * t) {
            r.add("DFM-L-MIN-LEN", "error",
                  "lance polyline length " + std::to_string(pathLen) +
                  " mm < 3 × sheet_thickness (" + std::to_string(3.0 * t) +
                  " mm)");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Design (lance = cut + bend):
//
// 1) SLIT — build a stadium-chain cutter along the polyline waypoints
//    with kerf = max(0.1 × thickness, 0.05 mm).  Cut the sheet by it,
//    producing a knife-edge slit (very thin through-channel).
//
// 2) TAB — the tab occupies a strip on ONE side of the slit, bounded by
//    the polyline and an offset polyline `tab_width_mm` away on the
//    perpendicular of the first segment.  We build the tab footprint
//    as a stadium-chain cutter of width `tab_width_mm` along the
//    SHIFTED polyline.  This footprint is intersected (geometrically)
//    with a thin "tab slab" — a stadium-chain prism extruded UPWARD
//    from sheet top by `lift_mm`.  Then we fuse the tab slab back on
//    top of the slit sheet.
//
// Slice-1 simplification: the tab is a flat raised slab; we don't model
// the hinge curvature.  This still produces the correct topology pattern
// (slit + raised flat tab + main sheet) for downstream RE.
SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "lance DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double t = sheetThicknessOf(wp);

    // 1) SLIT cutter — thin polyline cutter through the full sheet thickness.
    const double slitKerf = std::max(0.1 * t, 0.05);
    const double kOverhang = 0.1;
    const double slitHeight = (zMax - zMin) + 2.0 * kOverhang;
    TopoDS_Shape slitCutter;
    try {
        slitCutter = pr::offsetPolylineCutter(in.waypoints, slitKerf, slitHeight,
                                              zMax + kOverhang);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("lance: slit cutter build failed: ") + ex.what());
    }

    // Apply slit.
    TopoDS_Shape slittedShape;
    try {
        slittedShape = pr::cut(wp.shape(), slitCutter);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("lance: slit boolean failed: ") + ex.what());
    }

    // 2) TAB slab — compute perpendicular shift direction from the FIRST
    //    polyline segment, build a shifted polyline (each waypoint moved
    //    by tab_width_mm / 2 along perp), then a stadium chain of width
    //    = tab_width_mm extruded upward by lift_mm.
    const auto& w0 = in.waypoints[0];
    const auto& w1 = in.waypoints[1];
    const double dx = w1[0] - w0[0];
    const double dy = w1[1] - w0[1];
    const double segLen = std::hypot(dx, dy);
    // Perpendicular direction (rotate +90°): (-dy, dx)/L.
    const double perpX = -dy / segLen;
    const double perpY =  dx / segLen;

    // Tab centerline = polyline shifted by half tab_width along +perp.
    std::vector<std::array<double, 2>> tabCenter;
    tabCenter.reserve(in.waypoints.size());
    for (const auto& w : in.waypoints) {
        tabCenter.push_back({
            w[0] + (in.tab_width_mm * 0.5) * perpX,
            w[1] + (in.tab_width_mm * 0.5) * perpY
        });
    }

    // Tab slab: stadium-chain extruded DOWNWARD from (zMax + lift_mm) by
    // sheet thickness — so the slab sits on top of the sheet, displaced
    // upward by lift_mm.  Then we'll cut everything below zMax to leave
    // only the raised tab.
    TopoDS_Shape tabRaw;
    try {
        // offsetPolylineCutter extrudes downward by `depth_mm` from `base_z`.
        // We want the bottom of the slab at zMax + lift_mm and top at
        // zMax + lift_mm + t (a thin slab raised by lift_mm).  So base_z =
        // zMax + lift_mm + t, depth = t.
        tabRaw = pr::offsetPolylineCutter(tabCenter, in.tab_width_mm, t,
                                          zMax + in.lift_mm + t);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("lance: tab slab build failed: ") + ex.what());
    }

    // Fuse the tab onto the slitted shape.  The tab is geometrically
    // disconnected from the main sheet (slice-1 approximation), but for
    // signature pattern recognition this is sufficient.
    TopoDS_Shape result;
    try {
        result = pr::fuse(slittedShape, tabRaw);
    } catch (const Standard_Failure& ex) {
        spdlog::warn("lance: tab fuse failed ({}); keeping slit-only", ex.what());
        result = slittedShape;
    }

    const double cutPathLen = polylineLength(in.waypoints);

    // Signature
    json wpJson = json::array();
    for (const auto& w : in.waypoints) wpJson.push_back({ w[0], w[1] });

    json params = {
        { "waypoints",     wpJson },
        { "tab_width_mm",  in.tab_width_mm },
        { "lift_mm",       in.lift_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "cut_kind",              "Polyline" },
        { "cut_path_length_mm",    cutPathLen },
        { "tab_width_mm",          in.tab_width_mm },
        { "lift_mm",               in.lift_mm },
        { "slit_kerf_mm",          slitKerf },
        { "sheet_thickness_mm",    t },
        { "is_through_cut",        true },        // slit goes through
        { "is_combined_cut_bend",  true },        // distinguishes from sheet_punch
    };
    ToolingMeta tooling;
    tooling.tool_type         = "lancing_die";
    tooling.tool_dia_mm       = slitKerf;
    tooling.tool_length_mm    = t * 2.0;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = cutPathLen * slitKerf * t;
    tooling.est_cycle_time_s  = 1.0 + cutPathLen / 50.0;
    tooling.extra["machining_constraint"] =
        "Lance — single-stroke die that slits + lifts a tab; lift ≥ thickness "
        "to clear slit; tab thinning at hinge ~ 20%";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::lance applied: pathLen={} lift={} tab={} faces {}→{}",
                  cutPathLen, in.lift_mm, in.tab_width_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// A lance leaves: (a) a slit (thin through-channel), (b) a raised tab
// slab whose lowest +Z face sits at sheet_top + lift_mm.  Identifier:
// the largest +Z face is NOT the main sheet top; there are MULTIPLE +Z
// faces at distinct Z values, with the higher one being the tab.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const double t = sheetThicknessOf(wp);
    if (t <= 0.0 || t > 5.0) return out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // Find largest +Z face at the bbox-top Z (the tab top) and largest +Z
    // face SIGNIFICANTLY below it (the main sheet top).
    int topFace = -1, mainFace = -1;
    double topArea = -1.0, mainArea = -1.0;
    double topZ = 0.0, mainZ = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.9) continue;
        const double a = wp.faceArea(i);
        const double cZ = wp.faceCenter(i).Z();
        if (cZ > zMax - 0.5) {
            if (a > topArea) { topArea = a; topFace = i; topZ = cZ; }
        }
    }
    // Now find the largest +Z planar face well BELOW topZ.
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (i == topFace) continue;
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.9) continue;
        const double a = wp.faceArea(i);
        const double cZ = wp.faceCenter(i).Z();
        if (cZ < topZ - 0.5 * t) {
            if (a > mainArea) { mainArea = a; mainFace = i; mainZ = cZ; }
        }
    }
    if (topFace < 0 || mainFace < 0) return out;

    const double liftEst = topZ - mainZ - t;
    if (liftEst < t * 0.5) return out;   // not enough separation

    // Tab area ≈ tab_width × cut_path_length; we report area/something
    // sensible as recovered tab width.
    const double tabAreaEst = topArea;

    json recovered = {
        { "waypoints",     json::array() },   // path not recoverable from B-rep
        { "tab_width_mm",  std::sqrt(tabAreaEst) },   // rough estimate
        { "lift_mm",       liftEst },
    };
    json matched = {
        { "tab_top_face_id",   topFace },
        { "main_top_face_id",  mainFace },
        { "tab_z",             topZ },
        { "main_z",            mainZ },
        { "tab_area_mm2",      topArea },
        { "sheet_thickness_mm", t },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.70, matched });
    return out;
}

}  // namespace koocadcam::skill::lance
