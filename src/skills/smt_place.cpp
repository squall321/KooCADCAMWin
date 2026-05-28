// @lat: [[engine/skills#smt_place]]

#include "smt_place.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::smt_place {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.component_count <= 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": component_count must be > 0 (got " +
              std::to_string(in.component_count) + ")");
    }
    if (in.placement_rate_cph <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": placement_rate_cph must be > 0 (got " +
              std::to_string(in.placement_rate_cph) + ")");
    }
    if (in.accuracy_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": accuracy_um must be > 0 (got " +
              std::to_string(in.accuracy_um) + ")");
    } else if (in.accuracy_um > 100.0) {
        r.add("DFM-SMT-ACCURACY", "error",
              std::string(kSkillId) + ": accuracy_um " +
              std::to_string(in.accuracy_um) +
              " > 100 μm — too coarse for 01005/0201 placement");
    }
    if (in.placement_rate_cph > 200000.0) {
        r.add("DFM-SMT-RATE", "info",
              std::string(kSkillId) + ": placement_rate_cph " +
              std::to_string(in.placement_rate_cph) +
              " above any industrial machine spec (~150k cph ceiling)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "component_count",    in.component_count },
        { "placement_rate_cph", in.placement_rate_cph },
        { "accuracy_um",        in.accuracy_um },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "component_count",     in.component_count },
        { "placement_rate_cph",  in.placement_rate_cph },
        { "accuracy_um",         in.accuracy_um },
        { "geometry_changed",    false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "pick_and_place";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "vacuum_nozzle";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Cycle time = component_count / placement_rate_cph, in seconds.
    tooling.est_cycle_time_s  =
        (in.placement_rate_cph > 0.0)
            ? (static_cast<double>(in.component_count) /
               in.placement_rate_cph) * 3600.0
            : 0.0;
    tooling.extra = {
        { "process",            "smt_pick_and_place" },
        { "process_category",   "electronics_assembly" },
        { "component_count",    in.component_count },
        { "placement_rate_cph", in.placement_rate_cph },
        { "accuracy_um",        in.accuracy_um },
        { "geometric_no_op",    true },
        { "dfm_findings",       findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::smt_place applied: count={} rate={} cph acc={} μm",
                  in.component_count, in.placement_rate_cph, in.accuracy_um);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// SMT placement leaves no macroscopic OCCT trace (every component is a
// pre-existing part).  Recognize ONLY via metadata replay.

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

}  // namespace koocadcam::skill::smt_place
