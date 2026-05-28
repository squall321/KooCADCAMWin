// @lat: [[engine/skills#beading]]

#include "beading.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/PolylineOffset.hpp"
#include "engine/primitives/Tools.hpp"

#include <Standard_Failure.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::beading {

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
              "beading: waypoints needs >= 2 points (got " +
              std::to_string(in.waypoints.size()) + ")");
    } else {
        for (size_t i = 1; i < in.waypoints.size(); ++i) {
            const double dx = in.waypoints[i][0] - in.waypoints[i-1][0];
            const double dy = in.waypoints[i][1] - in.waypoints[i-1][1];
            if (std::hypot(dx, dy) < 1e-6) {
                r.add("DFM-INPUT", "error",
                      "beading: duplicate consecutive waypoints at index " +
                      std::to_string(i));
                break;
            }
        }
    }
    if (in.width_mm <= 0.0)
        r.add("DFM-INPUT", "error", "beading: width_mm must be > 0");
    if (in.depth_mm <= 0.0)
        r.add("DFM-INPUT", "error", "beading: depth_mm must be > 0");

    const double t = sheetThicknessOf(wp);
    if (t > 5.0) {
        r.add("DFM-BD-SHEET", "error",
              "beading: stock thickness " + std::to_string(t) +
              " > 5 mm — not sheet metal");
    }
    if (t > 0.0 && in.width_mm < 2.0 * t) {
        r.add("DFM-BD-MIN-W", "error",
              "beading width " + std::to_string(in.width_mm) +
              " mm < 2 × sheet_thickness (" + std::to_string(2.0 * t) +
              " mm) — bead profile cannot form");
    }
    if (t > 0.0 && in.depth_mm < 0.5 * t) {
        r.add("DFM-BD-MIN-D", "error",
              "beading depth " + std::to_string(in.depth_mm) +
              " mm < 0.5 × sheet_thickness (" + std::to_string(0.5 * t) +
              " mm) — too shallow to stiffen");
    }
    if (t > 0.0 && in.depth_mm > 2.0 * t) {
        r.add("DFM-BD-MAX-D", "error",
              "beading depth " + std::to_string(in.depth_mm) +
              " mm > 2 × sheet_thickness (" + std::to_string(2.0 * t) +
              " mm) — material will tear");
    }
    if (in.waypoints.size() >= 2) {
        const double pathLen = polylineLength(in.waypoints);
        if (t > 0.0 && pathLen < 3.0 * t) {
            r.add("DFM-BD-MIN-LEN", "error",
                  "beading path length " + std::to_string(pathLen) +
                  " mm < 3 × sheet_thickness (" + std::to_string(3.0 * t) +
                  " mm)");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "beading DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double t = sheetThicknessOf(wp);

    TopoDS_Shape result;
    if (in.mode == Mode::Ridge) {
        // Build a stadium-chain cutter sitting ON TOP of sheet, extruded
        // DOWN by depth_mm from zMax + depth_mm.  That places its bottom
        // exactly at zMax and its top at zMax + depth_mm — a raised strip.
        TopoDS_Shape ridge;
        try {
            ridge = pr::offsetPolylineCutter(in.waypoints, in.width_mm,
                                             in.depth_mm, zMax + in.depth_mm);
        } catch (const Standard_Failure& ex) {
            throw SkillError(std::string("beading: ridge build failed: ") + ex.what());
        }
        try {
            result = pr::fuse(wp.shape(), ridge);
        } catch (const Standard_Failure& ex) {
            throw SkillError(std::string("beading: ridge fuse failed: ") + ex.what());
        }
    } else {
        // Groove: cut a channel from the top, depth_mm deep.
        TopoDS_Shape groove;
        try {
            groove = pr::offsetPolylineCutter(in.waypoints, in.width_mm,
                                              in.depth_mm + 0.05,
                                              zMax + 0.05);
        } catch (const Standard_Failure& ex) {
            throw SkillError(std::string("beading: groove build failed: ") + ex.what());
        }
        try {
            result = pr::cut(wp.shape(), groove);
        } catch (const Standard_Failure& ex) {
            throw SkillError(std::string("beading: groove cut failed: ") + ex.what());
        }
    }

    const double pathLen = polylineLength(in.waypoints);

    // Signature
    json wpJson = json::array();
    for (const auto& w : in.waypoints) wpJson.push_back({ w[0], w[1] });

    json params = {
        { "waypoints",  wpJson },
        { "width_mm",   in.width_mm },
        { "depth_mm",   in.depth_mm },
        { "mode",       modeToString(in.mode) },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "mode",                  modeToString(in.mode) },
        { "is_additive",           in.mode == Mode::Ridge },
        { "path_length_mm",        pathLen },
        { "width_mm",              in.width_mm },
        { "depth_mm",              in.depth_mm },
        { "sheet_thickness_mm",    t },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "beading_roller";
    tooling.tool_dia_mm       = in.width_mm;
    tooling.tool_length_mm    = in.depth_mm;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 =
        (in.mode == Mode::Groove) ? pathLen * in.width_mm * in.depth_mm : 0.0;
    tooling.est_cycle_time_s  = 2.0 + pathLen / 80.0;
    tooling.extra["machining_constraint"] =
        "Beading — formed by passing the sheet between a paired upper and "
        "lower roller; runs along an open or closed path; min width ≥ 2 × t";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::beading applied: mode={} pathLen={} w={} d={} faces {}→{}",
                  modeToString(in.mode), pathLen, in.width_mm, in.depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Find +Z planar faces displaced UP (ridge) or DOWN (groove) from main
// sheet top by ~ depth.  Use area / width to back out path length.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const double t = sheetThicknessOf(wp);
    if (t <= 0.0 || t > 5.0) return out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // Find the largest +Z face at the SHEET TOP plane (main sheet top).
    // Identify mainTop by area + ~ z_center near (zMax - t) for groove or
    // exactly zMax for ridge.  Easier: take the LARGEST +Z planar face.
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

    // Look for OTHER +Z planar faces offset by small Z amount → bead candidate.
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (i == mainTopId) continue;
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.9) continue;
        const double a = wp.faceArea(i);
        const double cZ = wp.faceCenter(i).Z();
        const double dz = cZ - mainTopZ;     // signed offset
        const double absDz = std::abs(dz);
        if (absDz < 0.05 * t || absDz > 2.5 * t) continue;
        if (a >= mainTopArea * 0.9) continue;  // not a small local feature

        const std::string modeStr = (dz > 0.0) ? "ridge" : "groove";
        // Approximate width: area is roughly width × pathLen.  Without
        // knowing pathLen we can only return a length-product.  Report
        // depth = |dz|; width estimate by assuming square-ish strip.
        const double widthEst = std::min(a / std::max(1.0, std::sqrt(a)),
                                          std::sqrt(a));

        json recovered = {
            { "waypoints",  json::array() },   // path not recoverable from B-rep
            { "width_mm",   widthEst },
            { "depth_mm",   absDz },
            { "mode",       modeStr },
        };
        json matched = {
            { "feature_face_id",     i },
            { "main_top_face_id",    mainTopId },
            { "feature_z",           cZ },
            { "main_z",              mainTopZ },
            { "feature_area_mm2",    a },
            { "sheet_thickness_mm",  t },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.65, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::beading
