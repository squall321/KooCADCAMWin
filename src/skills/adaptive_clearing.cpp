// @lat: [[engine/skills#adaptive_clearing]]

#include "adaptive_clearing.hpp"

#include "Workpiece.hpp"
#include "mill_rect_pocket.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::adaptive_clearing {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.length_mm <= 0.0 || in.width_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "adaptive_clearing: length_mm and width_mm must be > 0");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "adaptive_clearing: depth_mm must be > 0");
    }
    if (in.tool_dia_mm < 1.0) {
        r.add("DFM-002", "error",
              "adaptive_clearing: tool_dia_mm " + std::to_string(in.tool_dia_mm) +
              " < min 1.0 mm");
    }

    const double effectiveStepover = (in.stepover_mm > 0.0)
        ? in.stepover_mm
        : in.tool_dia_mm * 0.1;
    if (effectiveStepover > in.tool_dia_mm * 0.3 + 1e-9) {
        r.add("DFM-TROCH-SO", "error",
              "adaptive_clearing: stepover_mm " + std::to_string(effectiveStepover) +
              " > tool_dia × 0.3");
    }

    if (in.optimal_engagement_pct < 10.0 || in.optimal_engagement_pct > 60.0) {
        r.add("DFM-ADAPT-ENG", "error",
              "adaptive_clearing: optimal_engagement_pct " +
              std::to_string(in.optimal_engagement_pct) +
              " outside valid range [10, 60]");
    }

    if (in.corner_r_mm < 0.0) {
        r.add("DFM-INPUT", "error", "adaptive_clearing: corner_r_mm must be >= 0");
    }
    if (in.corner_r_mm > 0.0 && in.corner_r_mm < 0.2) {
        r.add("DFM-004", "error",
              "adaptive_clearing: corner_r " + std::to_string(in.corner_r_mm) +
              " mm < min R 0.2 mm");
    }
    const double minDim = std::min(in.length_mm, in.width_mm);
    if (in.corner_r_mm > 0.0 && in.corner_r_mm >= minDim / 2.0) {
        r.add("DFM-INPUT", "error",
              "adaptive_clearing: corner_r must be < min(length, width)/2");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Same delegate-to-rect-pocket pattern as `trochoidal_mill`: identical
// geometric result, different signature/tooling metadata.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "adaptive_clearing DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

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

    const double effectiveStepover = (in.stepover_mm > 0.0)
        ? in.stepover_mm
        : in.tool_dia_mm * 0.1;

    FeatureSignature sig = pocketOut.signature;
    sig.skill_id = kSkillId;

    sig.params = {
        { "center_x_mm",             in.center_x_mm },
        { "center_y_mm",             in.center_y_mm },
        { "axis_dir",                { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "length_mm",               in.length_mm },
        { "width_mm",                in.width_mm },
        { "depth_mm",                in.depth_mm },
        { "corner_r_mm",             in.corner_r_mm },
        { "tool_dia_mm",             in.tool_dia_mm },
        { "stepover_mm",             effectiveStepover },
        { "optimal_engagement_pct",  in.optimal_engagement_pct },
    };
    sig.pattern = {
        { "kind",                     kSkillId },
        { "planar_wall_count",        4 },
        { "corner_cylinder_count",    in.corner_r_mm > 0.0 ? 4 : 0 },
        { "bottom_planar_face",       true },
        { "length_mm",                in.length_mm },
        { "width_mm",                 in.width_mm },
        { "corner_r_mm",              in.corner_r_mm },
        { "strategy",                 "adaptive" },
        { "stepover_mm",              effectiveStepover },
        { "optimal_engagement_pct",   in.optimal_engagement_pct },
    };

    // Adaptive clearing: most aggressive parameters in the strategy family.
    ToolingMeta tooling                       = sig.tooling;
    tooling.tool_type                         = "end_mill_adaptive";
    tooling.tool_dia_mm                       = in.tool_dia_mm;
    tooling.tool_length_mm                    = in.depth_mm * 1.5 + 5.0;
    tooling.tool_material                     = "carbide";
    tooling.flute_count                       = 4;
    tooling.cutting_speed_sfm                 = 600.0 * 2.0;
    tooling.feed_per_tooth_mm                 = 0.025 * 3.0;
    tooling.stock_removed_mm3                 = in.length_mm * in.width_mm * in.depth_mm;
    tooling.est_cycle_time_s                  = std::max(1.0,
        (in.length_mm * in.width_mm * in.depth_mm) / 12000.0);
    tooling.extra["strategy"]                 = "adaptive";
    tooling.extra["stepover_mm"]              = effectiveStepover;
    tooling.extra["optimal_engagement_pct"]   = in.optimal_engagement_pct;
    sig.tooling = tooling;

    auto wpNew = pocketOut.workpiece;
    wpNew->addFeature(sig);

    spdlog::debug("skill::adaptive_clearing applied: {}x{}x{} tool={} eng%={}",
                  in.length_mm, in.width_mm, in.depth_mm,
                  in.tool_dia_mm, in.optimal_engagement_pct);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Same scheme as trochoidal_mill: feature history first (conf 0.90), then
// 0.40-confidence marker that defers to mill_rect_pocket.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

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

    auto rectCands = mill_rect_pocket::recognize(wp);
    for (const auto& p : rectCands) {
        json rp = p.recovered_params;
        rp["tool_dia_mm"]            = 0.0;
        rp["stepover_mm"]            = 0.0;
        rp["optimal_engagement_pct"] = 0.0;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = rp;
        rf.confidence       = 0.40;
        rf.matched_geometry = {
            { "source", "geometric_marker" },
            { "rect_pocket", p.matched_geometry },
            { "note",
              "adaptive_clearing vs rect_pocket cannot be differentiated "
              "by geometry alone; CAM metadata required" },
        };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::adaptive_clearing
