// @lat: [[engine/skills#replace_part]]

#include "replace_part.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <unordered_set>

namespace koocadcam::skill::replace_part {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.old_part_ids.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": old_part_ids must be non-empty");
    }
    if (in.new_part_ids.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": new_part_ids must be non-empty");
    }
    if (in.work_time_min <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": work_time_min must be > 0 (got " +
              std::to_string(in.work_time_min) + ")");
    }
    if (!in.old_part_ids.empty() && !in.new_part_ids.empty() &&
        in.old_part_ids.size() != in.new_part_ids.size()) {
        r.add("DFM-REPL-COUNT", "error",
              std::string(kSkillId) + ": old_part_ids.size()=" +
              std::to_string(in.old_part_ids.size()) +
              " != new_part_ids.size()=" +
              std::to_string(in.new_part_ids.size()) +
              " — replacements must be 1:1");
    }

    // Duplicate-id detection within old_part_ids (info-only).
    std::unordered_set<std::string> seen;
    for (const auto& id : in.old_part_ids) {
        if (!seen.insert(id).second) {
            r.add("DFM-REPL-DUP", "info",
                  std::string(kSkillId) + ": duplicated old_part_id '" + id +
                  "' — verify operator log");
            break;
        }
    }

    if (in.work_time_min > 240.0) {
        r.add("DFM-REPL-LONG", "info",
              std::string(kSkillId) + ": work_time_min " +
              std::to_string(in.work_time_min) +
              " > 240 (>4 h replacement is unusual)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "replace_part DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json oldIds = json::array();
    for (const auto& id : in.old_part_ids) oldIds.push_back(id);
    json newIds = json::array();
    for (const auto& id : in.new_part_ids) newIds.push_back(id);

    json params = {
        { "old_part_ids",  oldIds },
        { "new_part_ids",  newIds },
        { "work_time_min", in.work_time_min },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "old_part_ids",     oldIds },
        { "new_part_ids",     newIds },
        { "part_count",       static_cast<int>(in.old_part_ids.size()) },
        { "work_time_min",    in.work_time_min },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "hand_tools";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = in.work_time_min * 60.0;
    tooling.extra = {
        { "process",          "replace_part" },
        { "process_category", "maintenance" },
        { "geometric_no_op",  true },
        { "part_count",       static_cast<int>(in.old_part_ids.size()) },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::replace_part applied: {} parts swapped in {} min",
        in.old_part_ids.size(), in.work_time_min);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Form-fit-function-identical replacements leave no B-rep trace.  Metadata
// replay only.

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

}  // namespace koocadcam::skill::replace_part
