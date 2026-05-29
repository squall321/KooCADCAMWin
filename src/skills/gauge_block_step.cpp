// @lat: [[engine/skills#gauge_block_step]]

#include "gauge_block_step.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp_Pln.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <set>

namespace koocadcam::skill::gauge_block_step {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────
//
// DFM cites JIS B 7506 (Standard gauge blocks) and ASME B89.1.9 (Stepped
// gauge for height-gauge calibration).

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.step_depths_mm.empty()) {
        r.add("DFM-INPUT", "error",
              "gauge_block_step: at least 1 step depth required");
        return r;
    }
    if (in.step_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "gauge_block_step: step_length must be > 0");
    }
    // ASME B89.1.9: each step ≥ 0.5 mm depth (resolution), step length ≥ 5 mm
    if (in.step_length_mm < 5.0) {
        r.add("DFM-STEP-LEN", "error",
              "gauge_block_step: step_length " +
              std::to_string(in.step_length_mm) +
              " mm < ASME B89.1.9 min 5 mm (probe ball clearance)");
    }
    // Monotonically increasing + each step ≥ 0.5 mm absolute, no two equal.
    double prev = 0.0;
    for (size_t i = 0; i < in.step_depths_mm.size(); ++i) {
        const double d = in.step_depths_mm[i];
        if (d < 0.5) {
            r.add("DFM-STEP-DEPTH", "error",
                  "gauge_block_step: step " + std::to_string(i) +
                  " depth " + std::to_string(d) +
                  " mm < ASME B89.1.9 min 0.5 mm");
        }
        if (i > 0 && d <= prev) {
            r.add("DFM-STEP-ORDER", "error",
                  "gauge_block_step: step depths must be strictly monotonic; "
                  "step " + std::to_string(i) + " (" + std::to_string(d) +
                  ") ≤ step " + std::to_string(i - 1) + " (" +
                  std::to_string(prev) + ")");
        }
        prev = d;
    }

    // x_start + N · step_length ≤ block length − 5 mm end clearance.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double blockLen = xMax - xMin;
    const double extent =
        in.x_start_mm + in.step_length_mm * static_cast<double>(in.step_depths_mm.size());
    if (extent > blockLen - 5.0) {
        r.add("DFM-STEP-EXTENT", "error",
              "gauge_block_step: total step extent " + std::to_string(extent) +
              " mm > block length " + std::to_string(blockLen) +
              " − 5 mm end clearance");
    }
    // Each step floor must be above the block bottom.
    const double blockHeight = zMax - zMin;
    if (!in.step_depths_mm.empty()) {
        const double deepest = in.step_depths_mm.back();
        if (deepest >= blockHeight - 1.0) {
            r.add("DFM-STEP-DEPTH", "error",
                  "gauge_block_step: deepest step " +
                  std::to_string(deepest) + " > block height " +
                  std::to_string(blockHeight) + " − 1 mm safety");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// For step i ∈ [1..N]:
//   cut a box  (x in [x_start + (i-1)*step_len, blockMaxX],
//               y in [0, blockY],
//               z in [topZ - depth_i, topZ + overhang])
// This produces N strictly increasing floor levels: the deepest cut
// "wins" over the shallower in the overlap region — which is desired
// (each successive step starts at the next x and extends to block end,
// so deeper steps overlap shallower steps).
//
// We accumulate one big fused cutter then do one Boolean cut.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "gauge_block_step DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto topId = wp.resolve(in.top_face);
    if (!topId) throw SkillError("gauge_block_step: top_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;
    const double topZ = zMax;
    const double yWidth = (in.y_full_mm > 0.0) ? in.y_full_mm : (yMax - yMin);

    TopoDS_Shape fused;
    bool first = true;
    for (size_t i = 0; i < in.step_depths_mm.size(); ++i) {
        const double depth = in.step_depths_mm[i];
        // Step floor extends from x_start + i*step_length to block end.
        const double xStart = xMin + in.x_start_mm +
                              in.step_length_mm * static_cast<double>(i);
        const double xEnd   = xMax + kOverhang;
        const double dx     = xEnd - xStart;

        // The cut box: origin at (xStart, 0, topZ - depth), extents
        // (dx, yWidth, depth + overhang).  We need it to go DOWN by depth +
        // small overhang above topZ so the Boolean isn't degenerate.
        const gp_Pnt origin(xStart, yMin, topZ - depth);
        const gp_Ax2 ax(origin, gp::DZ());
        const TopoDS_Shape stepCutter =
            pr::box(ax, dx, yWidth, depth + kOverhang);

        if (first) { fused = stepCutter; first = false; }
        else       { fused = pr::fuse(fused, stepCutter); }
    }

    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    // ── Signature ────────────────────────────────────────────────────────
    json depthsJson = json::array();
    for (double d : in.step_depths_mm) depthsJson.push_back(d);

    json params = {
        { "top_face_id",      *topId },
        { "x_start_mm",       in.x_start_mm },
        { "step_length_mm",   in.step_length_mm },
        { "y_full_mm",        in.y_full_mm },
        { "step_depths_mm",   depthsJson },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_compound",       true },
        { "subfeature_count",  static_cast<int>(in.step_depths_mm.size()) },
        { "step_count",        static_cast<int>(in.step_depths_mm.size()) },
        { "x_start_mm",        in.x_start_mm },
        { "step_length_mm",    in.step_length_mm },
        { "step_depths_mm",    depthsJson },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "endmill";
    tooling.tool_dia_mm      = std::min(in.step_length_mm * 0.8, 10.0);
    tooling.tool_length_mm   = in.step_depths_mm.back() + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 4;
    tooling.cutting_speed_sfm = 600.0;
    tooling.feed_per_tooth_mm = 0.05;
    double removedVol = 0.0;
    for (size_t i = 0; i < in.step_depths_mm.size(); ++i) {
        const double xStart = xMin + in.x_start_mm +
                              in.step_length_mm * static_cast<double>(i);
        const double dx = (xMax - xStart);
        const double dDepth = (i == 0)
            ? in.step_depths_mm[0]
            : in.step_depths_mm[i] - in.step_depths_mm[i - 1];
        if (dx > 0.0 && dDepth > 0.0) removedVol += dx * yWidth * dDepth;
    }
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = std::max(5.0, removedVol / 5000.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "face_mill" },
              { "tool_dia_mm", tooling.tool_dia_mm },
              { "operation",   "step_floor_pass" },
              { "pass_count",  static_cast<int>(in.step_depths_mm.size()) } },
        } },
        { "gauge_standard", "ASME_B89.1.9 / JIS_B7506" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::gauge_block_step applied: {} steps, "
                  "max_depth={:.2f} mm",
                  in.step_depths_mm.size(), in.step_depths_mm.back());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric heuristic PRIMARY: count horizontal planar faces (normal ≈ +Z)
// at distinct Z heights — N+1 implies N steps (top + N floors).  Recover
// step depths from Z values.  Metadata replay FALLBACK.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata path first.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" },
                                { "is_compound", true } };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // Collect distinct Z heights of upward-facing planar faces.
    std::vector<double> zUp;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (std::abs(n.Z() - 1.0) > 1e-2) continue;
        const gp_Pnt c = wp.faceCenter(i);
        // Skip the bottom of the block.
        if (std::abs(c.Z() - zMin) < 1e-3) continue;
        // Dedupe within tolerance 1e-2.
        bool seen = false;
        for (double z : zUp) if (std::abs(z - c.Z()) < 1e-2) { seen = true; break; }
        if (!seen) zUp.push_back(c.Z());
    }

    if (zUp.size() < 2) return out;
    std::sort(zUp.begin(), zUp.end(), std::greater<double>());
    // Top = zUp[0].  Step depths = zUp[0] - zUp[i] for i ≥ 1.
    json depths = json::array();
    for (size_t i = 1; i < zUp.size(); ++i) {
        depths.push_back(zUp[0] - zUp[i]);
    }
    if (depths.empty()) return out;

    json rp = {
        { "x_start_mm",      5.0 },             // not recoverable from BREP
        { "step_length_mm",  10.0 },
        { "y_full_mm",       yMax - yMin },
        { "step_depths_mm",  depths },
    };
    RecognizedFeature rf;
    rf.skill_id         = kSkillId;
    rf.recovered_params = rp;
    rf.confidence       = 0.7;
    rf.matched_geometry = { { "horizontal_face_count", static_cast<int>(zUp.size()) },
                            { "source", "geometric" } };
    out.push_back(rf);
    return out;
}

}  // namespace koocadcam::skill::gauge_block_step
