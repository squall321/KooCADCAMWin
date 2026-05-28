// @lat: [[engine/skills#trochoidal_mill]]

#include "trochoidal_mill.hpp"

#include "Workpiece.hpp"
#include "mill_rect_pocket.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::trochoidal_mill {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.length_mm <= 0.0 || in.width_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "trochoidal_mill: length_mm and width_mm must be > 0");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "trochoidal_mill: depth_mm must be > 0");
    }
    if (in.tool_dia_mm < 1.0) {
        r.add("DFM-002", "error",
              "trochoidal_mill: tool_dia_mm " + std::to_string(in.tool_dia_mm) +
              " < min 1.0 mm (high-feed strategy needs cutter stiffness)");
    }

    const double effectiveStepover = (in.stepover_mm > 0.0)
        ? in.stepover_mm
        : in.tool_dia_mm * 0.1;
    if (effectiveStepover > in.tool_dia_mm * 0.3 + 1e-9) {
        r.add("DFM-TROCH-SO", "error",
              "trochoidal_mill: stepover_mm " + std::to_string(effectiveStepover) +
              " > tool_dia × 0.3 (= " +
              std::to_string(in.tool_dia_mm * 0.3) +
              " mm) — trochoidal advantage lost");
    }

    // Mirror the rect-pocket sanity checks so we fail early with the same
    // codes the caller is used to.
    if (in.corner_r_mm < 0.0) {
        r.add("DFM-INPUT", "error", "trochoidal_mill: corner_r_mm must be >= 0");
    }
    if (in.corner_r_mm > 0.0 && in.corner_r_mm < 0.2) {
        r.add("DFM-004", "error",
              "trochoidal_mill: corner_r " + std::to_string(in.corner_r_mm) +
              " mm < min R 0.2 mm");
    }
    const double minDim = std::min(in.length_mm, in.width_mm);
    if (in.corner_r_mm > 0.0 && in.corner_r_mm >= minDim / 2.0) {
        r.add("DFM-INPUT", "error",
              "trochoidal_mill: corner_r must be < min(length, width)/2");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Geometry is identical to `mill_rect_pocket`: we delegate the BREP work to
// that skill and then re-stamp the signature so the trochoidal strategy
// metadata replaces the rect-pocket metadata.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "trochoidal_mill DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Build the rect-pocket input mirroring our geometry parameters.
    mill_rect_pocket::Input rp;
    rp.entry_face  = in.entry_face;
    rp.center_x_mm = in.center_x_mm;
    rp.center_y_mm = in.center_y_mm;
    rp.axis_dir    = in.axis_dir;
    rp.length_mm   = in.length_mm;
    rp.width_mm    = in.width_mm;
    rp.depth_mm    = in.depth_mm;
    rp.corner_r_mm = in.corner_r_mm;

    SkillOutput pocketOut = mill_rect_pocket::apply(wp, rp);

    // ── Re-stamp the signature for the trochoidal strategy ─────────────
    const double effectiveStepover = (in.stepover_mm > 0.0)
        ? in.stepover_mm
        : in.tool_dia_mm * 0.1;

    FeatureSignature sig = pocketOut.signature;
    sig.skill_id = kSkillId;

    sig.params = {
        { "center_x_mm",  in.center_x_mm },
        { "center_y_mm",  in.center_y_mm },
        { "axis_dir",     { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "length_mm",    in.length_mm },
        { "width_mm",     in.width_mm },
        { "depth_mm",     in.depth_mm },
        { "corner_r_mm",  in.corner_r_mm },
        { "tool_dia_mm",  in.tool_dia_mm },
        { "stepover_mm",  effectiveStepover },
    };
    sig.pattern = {
        { "kind",                     kSkillId },
        { "planar_wall_count",        4 },
        { "corner_cylinder_count",    in.corner_r_mm > 0.0 ? 4 : 0 },
        { "bottom_planar_face",       true },
        { "length_mm",                in.length_mm },
        { "width_mm",                 in.width_mm },
        { "corner_r_mm",              in.corner_r_mm },
        { "strategy",                 "trochoidal" },
        { "stepover_mm",              effectiveStepover },
    };

    // Trochoidal strategy metadata: 1.5× SFM, 2× feed/tooth, high-feed mill.
    ToolingMeta tooling           = sig.tooling;
    tooling.tool_type             = "end_mill_high_feed";
    tooling.tool_dia_mm           = in.tool_dia_mm;
    tooling.tool_length_mm        = in.depth_mm * 1.5 + 5.0;
    tooling.tool_material         = "carbide";
    tooling.flute_count           = 4;
    tooling.cutting_speed_sfm     = 600.0 * 1.5;
    tooling.feed_per_tooth_mm     = 0.025 * 2.0;
    tooling.stock_removed_mm3     = in.length_mm * in.width_mm * in.depth_mm;
    // Trochoidal is generally faster than conventional pocketing.
    tooling.est_cycle_time_s      = std::max(1.0,
        (in.length_mm * in.width_mm * in.depth_mm) / 8000.0);
    tooling.extra["strategy"]     = "trochoidal";
    tooling.extra["stepover_mm"]  = effectiveStepover;
    sig.tooling = tooling;

    auto wpNew = pocketOut.workpiece;
    wpNew->addFeature(sig);

    spdlog::debug("skill::trochoidal_mill applied: {}x{}x{} tool={} stepover={}",
                  in.length_mm, in.width_mm, in.depth_mm,
                  in.tool_dia_mm, effectiveStepover);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Topology is identical to mill_rect_pocket → we can't differentiate by
// geometry alone.  For the in-memory case we walk the workpiece's feature
// history: if a feature with skill_id == "trochoidal_mill" was previously
// stamped, we surface it with full confidence (0.90).  Otherwise we look
// at the rect-pocket candidates and emit a 0.40-confidence marker — that's
// well below mill_rect_pocket's 0.85, so the rect-pocket interpretation
// dominates when both compete.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // ── A. Use feature history when available ────────────────────────
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 0.90;
        rf.matched_geometry = {
            { "source", "feature_history" },
        };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // ── B. Geometric marker (low confidence — defers to rect_pocket) ─
    auto rectCands = mill_rect_pocket::recognize(wp);
    for (const auto& p : rectCands) {
        json rp = p.recovered_params;
        // Annotate that we don't actually know the tool_dia / stepover —
        // they're not recoverable from a flat-bottom pocket geometry alone.
        rp["tool_dia_mm"] = 0.0;
        rp["stepover_mm"] = 0.0;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = rp;
        rf.confidence       = 0.40;   // weak marker, rect_pocket dominates
        rf.matched_geometry = {
            { "source", "geometric_marker" },
            { "rect_pocket", p.matched_geometry },
            { "note",
              "trochoidal vs rect_pocket cannot be differentiated by geometry "
              "alone; metadata (tool_dia, stepover) required" },
        };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::trochoidal_mill
