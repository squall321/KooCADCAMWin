// @lat: [[engine/skills#plunge_roughing]]

#include "plunge_roughing.hpp"

#include "Workpiece.hpp"
#include "mill_rect_pocket.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::plunge_roughing {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.length_mm <= 0.0 || in.width_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "plunge_roughing: length_mm and width_mm must be > 0");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "plunge_roughing: depth_mm must be > 0");
    }
    if (in.tool_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "plunge_roughing: tool_dia_mm " +
              std::to_string(in.tool_dia_mm) +
              " < min 0.8 mm");
    }

    const double effectiveStep = (in.plunge_stepover_mm > 0.0)
        ? in.plunge_stepover_mm
        : in.tool_dia_mm * 0.6;
    if (effectiveStep > in.tool_dia_mm + 1e-9) {
        r.add("DFM-PLUNGE-STEP", "error",
              "plunge_roughing: plunge_stepover_mm " +
              std::to_string(effectiveStep) +
              " > tool_dia_mm (= " + std::to_string(in.tool_dia_mm) +
              ") — plunges leave un-removed material");
    }

    if (in.corner_r_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "plunge_roughing: corner_r_mm must be >= 0");
    }
    if (in.corner_r_mm > 0.0 && in.corner_r_mm < 0.2) {
        r.add("DFM-004", "error",
              "plunge_roughing: corner_r " + std::to_string(in.corner_r_mm) +
              " mm < min R 0.2 mm");
    }
    const double minDim = std::min(in.length_mm, in.width_mm);
    if (in.corner_r_mm > 0.0 && in.corner_r_mm >= minDim / 2.0) {
        r.add("DFM-INPUT", "error",
              "plunge_roughing: corner_r must be < min(length, width)/2");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// At slice-1 fidelity: defer the geometric work to mill_rect_pocket (the
// cleanup pass leaves a clean flat-bottom pocket), then re-stamp the
// signature with plunge-roughing strategy metadata plus the grid layout.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "plunge_roughing DFM failed:";
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

    const double effectiveStep = (in.plunge_stepover_mm > 0.0)
        ? in.plunge_stepover_mm
        : in.tool_dia_mm * 0.6;

    // Build the plunge grid for the CAM toolpath generator.
    // We pack plunges along the longer axis first, indexed CC from the
    // -length/-width corner of the pocket footprint.  The number of
    // plunges along each axis is ceil(extent / stepover) + 1.
    const double r        = in.tool_dia_mm / 2.0;
    const double usableL  = std::max(0.0, in.length_mm - 2.0 * r);
    const double usableW  = std::max(0.0, in.width_mm  - 2.0 * r);
    const int    nx       = static_cast<int>(std::ceil(usableL / effectiveStep)) + 1;
    const int    ny       = static_cast<int>(std::ceil(usableW / effectiveStep)) + 1;

    json plungeGrid = json::array();
    if (nx > 0 && ny > 0) {
        const double startX = in.center_x_mm - in.length_mm / 2.0 + r;
        const double startY = in.center_y_mm - in.width_mm  / 2.0 + r;
        const double stepX  = (nx > 1) ? (usableL / (nx - 1)) : 0.0;
        const double stepY  = (ny > 1) ? (usableW / (ny - 1)) : 0.0;
        for (int iy = 0; iy < ny; ++iy) {
            for (int ix = 0; ix < nx; ++ix) {
                plungeGrid.push_back({
                    { "x", startX + ix * stepX },
                    { "y", startY + iy * stepY },
                });
            }
        }
    }

    FeatureSignature sig = pocketOut.signature;
    sig.skill_id = kSkillId;

    sig.params = {
        { "center_x_mm",         in.center_x_mm },
        { "center_y_mm",         in.center_y_mm },
        { "axis_dir",            { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "length_mm",           in.length_mm },
        { "width_mm",            in.width_mm },
        { "depth_mm",            in.depth_mm },
        { "corner_r_mm",         in.corner_r_mm },
        { "tool_dia_mm",         in.tool_dia_mm },
        { "plunge_stepover_mm",  effectiveStep },
    };
    sig.pattern = {
        { "kind",                     kSkillId },
        { "planar_wall_count",        4 },
        { "corner_cylinder_count",    in.corner_r_mm > 0.0 ? 4 : 0 },
        { "bottom_planar_face",       true },
        { "length_mm",                in.length_mm },
        { "width_mm",                 in.width_mm },
        { "corner_r_mm",              in.corner_r_mm },
        { "strategy",                 "plunge_roughing" },
        { "plunge_stepover_mm",       effectiveStep },
        { "plunge_count",             plungeGrid.size() },
        { "plunge_grid",              plungeGrid },
    };

    ToolingMeta tooling                       = sig.tooling;
    tooling.tool_type                         = "end_mill_plunge_capable";
    tooling.tool_dia_mm                       = in.tool_dia_mm;
    tooling.tool_length_mm                    = in.depth_mm * 1.5 + 5.0;
    tooling.tool_material                     = "carbide";
    tooling.flute_count                       = 2;          // plunge-friendly
    tooling.cutting_speed_sfm                 = 350.0;      // slow plunge
    tooling.feed_per_tooth_mm                 = 0.015;      // slow plunge
    tooling.stock_removed_mm3                 = in.length_mm * in.width_mm * in.depth_mm;
    // Plunge roughing is typically slower than conventional pocketing.
    tooling.est_cycle_time_s                  = std::max(1.0,
        (in.length_mm * in.width_mm * in.depth_mm) / 3500.0 +
        plungeGrid.size() * 0.3);
    tooling.extra["strategy"]                 = "plunge_roughing";
    tooling.extra["plunge_stepover_mm"]       = effectiveStep;
    tooling.extra["plunge_count"]             = plungeGrid.size();
    sig.tooling = tooling;

    auto wpNew = pocketOut.workpiece;
    wpNew->addFeature(sig);

    spdlog::debug("skill::plunge_roughing applied: {}x{}x{} tool={} plunges={}",
                  in.length_mm, in.width_mm, in.depth_mm,
                  in.tool_dia_mm, plungeGrid.size());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

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
        rp["tool_dia_mm"]        = 0.0;
        rp["plunge_stepover_mm"] = 0.0;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = rp;
        rf.confidence       = 0.40;
        rf.matched_geometry = {
            { "source", "geometric_marker" },
            { "rect_pocket", p.matched_geometry },
            { "note",
              "plunge_roughing produces the same flat-bottom pocket as "
              "conventional milling; CAM strategy metadata required" },
        };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::plunge_roughing
