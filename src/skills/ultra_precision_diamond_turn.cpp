// @lat: [[engine/skills#ultra_precision_diamond_turn]]

#include "ultra_precision_diamond_turn.hpp"

#include "Workpiece.hpp"
#include "turn_external.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::ultra_precision_diamond_turn {

using nlohmann::json;

namespace {

// Ferrous materials cause graphitization of diamond cutting edge.
bool isFerrousMaterial(const std::string& m)
{
    // Substring matching: any material id containing "steel" | "iron" |
    // explicit "ferrous_*" prefix is treated as ferrous for diamond-turn
    // purposes.  Hardened steels are doubly forbidden.
    if (m.find("steel") != std::string::npos) return true;
    if (m.find("iron") != std::string::npos)  return true;
    if (m.rfind("ferrous", 0) == 0)           return true;
    return false;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.final_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "ultra_precision_diamond_turn: final_dia_mm must be > 0");
    }
    if (in.end_z_mm <= in.start_z_mm) {
        r.add("DFM-INPUT", "error",
              "ultra_precision_diamond_turn: end_z_mm must be > start_z_mm");
    }
    if (in.roughing_passes < 1) {
        r.add("DFM-INPUT", "error",
              "ultra_precision_diamond_turn: roughing_passes must be ≥ 1");
    }

    // CRITICAL — ferrous materials destroy the diamond edge by graphitization.
    if (isFerrousMaterial(in.workpiece_material)) {
        r.add("DFM-SPDT-FERROUS", "error",
              "ultra_precision_diamond_turn: workpiece_material '" +
              in.workpiece_material +
              "' is ferrous — diamond tool graphitizes in seconds; "
              "use hard_turning (CBN) instead");
    }

    // Target Ra envelope (in nm).
    if (in.target_ra_nm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "ultra_precision_diamond_turn: target_ra_nm must be > 0");
    }
    if (in.target_ra_nm > 0.0 && in.target_ra_nm < 1.0) {
        r.add("DFM-SPDT-RA", "warning",
              "ultra_precision_diamond_turn: target_ra_nm " +
              std::to_string(in.target_ra_nm) +
              " < 1 nm — below practical SPDT limit; consider polishing");
    }
    if (in.target_ra_nm > 100.0) {
        r.add("DFM-SPDT-RA", "warning",
              "ultra_precision_diamond_turn: target_ra_nm " +
              std::to_string(in.target_ra_nm) +
              " > 100 nm — outside ultra-precision regime; "
              "use turn_external instead");
    }

    // Diamond nose radius envelope.
    if (in.tool_nose_r_mm < 0.1 || in.tool_nose_r_mm > 5.0) {
        r.add("DFM-SPDT-NOSE-R", "warning",
              "ultra_precision_diamond_turn: tool_nose_r_mm " +
              std::to_string(in.tool_nose_r_mm) +
              " outside typical [0.1, 5.0] mm — verify tool availability");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "ultra_precision_diamond_turn DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Geometry: delegate to turn_external.
    turn_external::Input te;
    te.start_z_mm      = in.start_z_mm;
    te.end_z_mm        = in.end_z_mm;
    te.final_dia_mm    = in.final_dia_mm;
    te.roughing_passes = in.roughing_passes;

    SkillOutput baseOut = turn_external::apply(wp, te);

    FeatureSignature sig = baseOut.signature;
    sig.skill_id = kSkillId;

    sig.params = {
        { "start_z_mm",         in.start_z_mm },
        { "end_z_mm",           in.end_z_mm },
        { "final_dia_mm",       in.final_dia_mm },
        { "roughing_passes",    in.roughing_passes },
        { "tool_nose_r_mm",     in.tool_nose_r_mm },
        { "target_ra_nm",       in.target_ra_nm },
        { "workpiece_material", in.workpiece_material },
    };
    sig.pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "annular_planar_face_count",  2 },
        { "axis",                       { 0.0, 0.0, 1.0 } },
        { "final_dia_mm",               in.final_dia_mm },
        { "start_z_mm",                 in.start_z_mm },
        { "end_z_mm",                   in.end_z_mm },
        { "strategy",                   "spdt" },
        { "target_ra_nm",               in.target_ra_nm },
        { "tool_nose_r_mm",             in.tool_nose_r_mm },
    };

    ToolingMeta tooling          = sig.tooling;
    tooling.tool_type            = "spdt_diamond_insert";
    tooling.tool_dia_mm          = 0.0;
    tooling.tool_length_mm       = (in.end_z_mm - in.start_z_mm) + 5.0;
    tooling.tool_material        = "single_crystal_diamond";
    tooling.flute_count          = 1;
    tooling.cutting_speed_sfm    = 1500.0;        // very high SFM
    tooling.feed_per_tooth_mm    = 0.005;         // sub-µm chip
    // SPDT cycle time is dominated by very low feed per rev.
    tooling.est_cycle_time_s     = std::max(2.0,
        in.roughing_passes * (in.end_z_mm - in.start_z_mm) / 5.0);
    tooling.extra["strategy"]     = "spdt";
    tooling.extra["target_ra_nm"] = in.target_ra_nm;
    tooling.extra["tool_nose_r_mm"] = in.tool_nose_r_mm;
    tooling.extra["form_error_um"] = 0.1;  // optical-grade form error budget
    sig.tooling = tooling;

    auto wpNew = baseOut.workpiece;
    wpNew->addFeature(sig);

    spdlog::debug("skill::ultra_precision_diamond_turn applied: Ra={}nm material={}",
                  in.target_ra_nm, in.workpiece_material);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Topology is identical to turn_external — recognition is metadata-only.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::ultra_precision_diamond_turn
