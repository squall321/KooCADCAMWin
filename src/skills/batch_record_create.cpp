// @lat: [[engine/skills#batch_record_create]]

#include "batch_record_create.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::batch_record_create {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.lot_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": lot_id must not be empty");
    }
    if (in.operator_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": operator_id must not be empty");
    }
    if (in.timestamp.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": timestamp must not be empty");
    }
    if (in.parameters_logged_count < 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": parameters_logged_count must be >= 0 (got " +
              std::to_string(in.parameters_logged_count) + ")");
    }
    if (in.parameters_logged_count == 0 && r.passed) {
        r.add("DFM-EBR-EMPTY", "info",
              std::string(kSkillId) +
              ": parameters_logged_count == 0 — eBatch record has no "
              "logged parameters, verify intent");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "batch_record_create DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "lot_id",                  in.lot_id },
        { "operator_id",             in.operator_id },
        { "timestamp",               in.timestamp },
        { "parameters_logged_count", in.parameters_logged_count },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "lot_id",                  in.lot_id },
        { "operator_id",             in.operator_id },
        { "timestamp",               in.timestamp },
        { "parameters_logged_count", in.parameters_logged_count },
        { "geometry_changed",        false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "ebatch_recorder";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;
    tooling.extra = {
        { "process",                 "ebatch_record_create" },
        { "lot_id",                  in.lot_id },
        { "operator_id",             in.operator_id },
        { "timestamp",               in.timestamp },
        { "parameters_logged_count", in.parameters_logged_count },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    // No geometric change — clone shape + material verbatim.
    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::batch_record_create applied: lot={} op={} ts={} params={}",
                  in.lot_id, in.operator_id, in.timestamp,
                  in.parameters_logged_count);

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
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::batch_record_create
