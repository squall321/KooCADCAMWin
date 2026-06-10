// @lat: [[engine/skills#draw_bead]]

#include "draw_bead.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/PolylineOffset.hpp"
#include "engine/primitives/Tools.hpp"

#include <Standard_Failure.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::draw_bead {

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

// Ensure closure: if first != last, return a copy with the first point
// appended.
std::vector<std::array<double, 2>> ensureClosed(
    const std::vector<std::array<double, 2>>& wps)
{
    if (wps.size() < 2) return wps;
    const double dx = wps.back()[0] - wps.front()[0];
    const double dy = wps.back()[1] - wps.front()[1];
    if (std::hypot(dx, dy) < 1e-6) return wps;
    auto closed = wps;
    closed.push_back(wps.front());
    return closed;
}

bool isClosed(const std::vector<std::array<double, 2>>& wps)
{
    if (wps.size() < 3) return false;
    const double dx = wps.back()[0] - wps.front()[0];
    const double dy = wps.back()[1] - wps.front()[1];
    return std::hypot(dx, dy) < 1e-3;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.waypoints.size() < 3) {
        r.add("DFM-INPUT", "error",
              "draw_bead: waypoints needs >= 3 points for a closed loop (got " +
              std::to_string(in.waypoints.size()) + ")");
    }
    if (in.width_mm <= 0.0)
        r.add("DFM-INPUT", "error", "draw_bead: width_mm must be > 0");
    if (in.depth_mm <= 0.0)
        r.add("DFM-INPUT", "error", "draw_bead: depth_mm must be > 0");

    if (in.waypoints.size() >= 3 && !isClosed(in.waypoints)) {
        // Closing the loop is OK programmatically — we accept either form,
        // but emit a closed-loop INFO so the caller is aware we'll auto-close.
        r.add("DFM-DB-CLOSED", "info",
              "draw_bead: waypoints do not close (first != last); "
              "auto-closing by appending first point");
    }

    const double t = sheetThicknessOf(wp);
    if (t > 5.0) {
        r.add("DFM-DB-SHEET", "error",
              "draw_bead: stock thickness " + std::to_string(t) +
              " > 5 mm — not sheet metal");
    }
    if (t > 0.0 && in.width_mm < 3.0 * t) {
        r.add("DFM-DB-MIN-W", "error",
              "draw_bead width " + std::to_string(in.width_mm) +
              " mm < 3 × sheet_thickness (" + std::to_string(3.0 * t) +
              " mm) — too narrow for drawbead restraint");
    }
    if (t > 0.0 && in.depth_mm < 0.5 * t) {
        r.add("DFM-DB-MIN-D", "error",
              "draw_bead depth " + std::to_string(in.depth_mm) +
              " mm < 0.5 × sheet_thickness (" + std::to_string(0.5 * t) +
              " mm) — insufficient restraint");
    }
    if (t > 0.0 && in.depth_mm > 1.5 * t) {
        r.add("DFM-DB-MAX-D", "error",
              "draw_bead depth " + std::to_string(in.depth_mm) +
              " mm > 1.5 × sheet_thickness (" + std::to_string(1.5 * t) +
              " mm) — material will tear");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "draw_bead DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double t = sheetThicknessOf(wp);

    // Auto-close if not already closed.
    const auto closedWps = ensureClosed(in.waypoints);

    // Build a stadium-chain cutter along the closed polyline, extruded
    // downward from above the sheet top by depth_mm + small overhang.
    TopoDS_Shape cutter;
    try {
        cutter = pr::offsetPolylineCutter(closedWps, in.width_mm,
                                           in.depth_mm + 0.05,
                                           zMax + 0.05);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("draw_bead: cutter build failed: ") + ex.what());
    }

    TopoDS_Shape newShape;
    try {
        newShape = pr::cut(wp.shape(), cutter);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("draw_bead: cut failed: ") + ex.what());
    }

    const double pathLen = polylineLength(closedWps);

    // Signature
    json wpJson = json::array();
    for (const auto& w : in.waypoints) wpJson.push_back({ w[0], w[1] });

    json params = {
        { "waypoints",  wpJson },
        { "width_mm",   in.width_mm },
        { "depth_mm",   in.depth_mm },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "is_closed",              true },
        { "path_length_mm",         pathLen },
        { "width_mm",               in.width_mm },
        { "depth_mm",               in.depth_mm },
        { "sheet_thickness_mm",     t },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drawbead_die";
    tooling.tool_dia_mm       = in.width_mm;
    tooling.tool_length_mm    = in.depth_mm;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = pathLen * in.width_mm * in.depth_mm;
    tooling.est_cycle_time_s  = 0.5 + pathLen / 100.0;
    tooling.extra["machining_constraint"] =
        "Draw bead — closed-loop restraint groove formed in the binder ring "
        "of a deep-drawing die; controls material flow during draw";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::draw_bead applied: pathLen={} w={} d={} faces {}→{}",
                  pathLen, in.width_mm, in.depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Identify a closed loop of vertical-walled faces enclosing a planar
// bottom face displaced DOWN from the main sheet top.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const double t = sheetThicknessOf(wp);
    if (t <= 0.0 || t > 5.0) return out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // Identify main sheet top Z (largest +Z face).
    int mainTopId = -1;
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
            mainTopId   = i;
            mainTopZ    = wp.faceCenter(i).Z();
        }
    }
    if (mainTopId < 0) return out;

    // Find +Z planar faces below mainTopZ by ~ depth (groove bottom).
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (i == mainTopId) continue;
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.9) continue;
        const double a = wp.faceArea(i);
        if (a >= mainTopArea * 0.9) continue;
        const double cZ = wp.faceCenter(i).Z();
        const double dz = mainTopZ - cZ;
        if (dz < 0.4 * t || dz > 1.7 * t) continue;

        const double depthEst = dz;
        // Rough width/path-length: area = width × pathLen; we report
        // both unknown but use std::sqrt(a) as a coarse width guess.
        const double widthEst = std::min(a / std::max(1.0, std::sqrt(a)),
                                          std::sqrt(a));

        json recovered = {
            { "waypoints", json::array() },
            { "width_mm",  widthEst },
            { "depth_mm",  depthEst },
        };
        json matched = {
            { "bottom_face_id",     i },
            { "main_top_face_id",   mainTopId },
            { "bottom_z",           cZ },
            { "main_z",             mainTopZ },
            { "bottom_area_mm2",    a },
            { "sheet_thickness_mm", t },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.65, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::draw_bead
