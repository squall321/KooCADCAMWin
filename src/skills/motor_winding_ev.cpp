// @lat: [[engine/skills#motor_winding_ev]]

#include "motor_winding_ev.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::motor_winding_ev {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.stator_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": stator_id must be non-empty");
    }

    if (in.slot_count <= 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": slot_count must be > 0 (got " +
              std::to_string(in.slot_count) + ")");
    } else if (in.slot_count < 12 || in.slot_count > 96) {
        r.add("DFM-WIND-SLOT", "info",
              std::string(kSkillId) + ": slot_count " +
              std::to_string(in.slot_count) +
              " outside typical EV range [12, 96]");
    }

    if (in.copper_fill_pct <= 0.0) {
        r.add("DFM-WIND-FILL", "error",
              std::string(kSkillId) + ": copper_fill_pct must be > 0 (got " +
              std::to_string(in.copper_fill_pct) + ")");
    } else if (in.copper_fill_pct > 75.0) {
        r.add("DFM-WIND-FILL", "error",
              std::string(kSkillId) + ": copper_fill_pct " +
              std::to_string(in.copper_fill_pct) +
              " > 75% — exceeds physical slot fill ceiling");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "motor_winding_ev DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "stator_id",       in.stator_id },
        { "slot_count",      in.slot_count },
        { "copper_fill_pct", in.copper_fill_pct },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "stator_id",        in.stator_id },
        { "slot_count",       in.slot_count },
        { "copper_fill_pct",  in.copper_fill_pct },
        { "winding_topology", "distributed_3phase" },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "hairpin_insertion_station";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "copper";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~ 1.2 s per slot for hairpin insertion + 30 s twist/weld setup.
    tooling.est_cycle_time_s  = 30.0 + 1.2 * static_cast<double>(in.slot_count);
    tooling.extra = {
        { "process",         "ev_stator_winding" },
        { "stator_id",       in.stator_id },
        { "slot_count",      in.slot_count },
        { "copper_fill_pct", in.copper_fill_pct },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::motor_winding_ev applied: stator={} slots={} fill={}%",
                  in.stator_id, in.slot_count, in.copper_fill_pct);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Copper winding inside slots is sub-OCCT — recognition is metadata-only.

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

}  // namespace koocadcam::skill::motor_winding_ev
