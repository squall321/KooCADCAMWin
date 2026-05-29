// @lat: [[engine/skills#ore_crush]]

#include "ore_crush.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::ore_crush {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownCrushers()
{
    static const std::unordered_set<std::string> s = {
        "jaw", "cone", "gyratory", "impact",
    };
    return s;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.feed_size_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "ore_crush feed_size_mm must be > 0 (got " +
              std::to_string(in.feed_size_mm) + ")");
    }

    if (in.target_p80_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "ore_crush target_p80_mm must be > 0 (got " +
              std::to_string(in.target_p80_mm) + ")");
    }

    if (in.crusher_type.empty() ||
        knownCrushers().count(in.crusher_type) == 0) {
        r.add("DFM-CRUSHER-TYPE", "error",
              "ore_crush unknown crusher_type '" + in.crusher_type +
              "' (expected jaw | cone | gyratory | impact)");
    }

    if (in.feed_size_mm > 0.0 && in.target_p80_mm > 0.0) {
        if (in.target_p80_mm >= in.feed_size_mm) {
            r.add("DFM-CRUSH-RATIO", "error",
                  "ore_crush target_p80_mm (" + std::to_string(in.target_p80_mm) +
                  ") must be < feed_size_mm (" + std::to_string(in.feed_size_mm) +
                  ") — no size reduction");
        } else {
            const double ratio = in.feed_size_mm / in.target_p80_mm;
            if (ratio > 12.0) {
                r.add("DFM-CRUSH-RATIO-HIGH", "info",
                      "ore_crush reduction ratio " + std::to_string(ratio) +
                      " > 12 — single stage unlikely; cascade crushers");
            }
        }
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "ore_crush DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double ratio = in.feed_size_mm / in.target_p80_mm;

    json params = {
        { "feed_size_mm",  in.feed_size_mm },
        { "target_p80_mm", in.target_p80_mm },
        { "crusher_type",  in.crusher_type },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "feed_size_mm",     in.feed_size_mm },
        { "target_p80_mm",    in.target_p80_mm },
        { "crusher_type",     in.crusher_type },
        { "reduction_ratio",  ratio },
        { "process_family",   "comminution" },
        { "geometry_changed", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    if      (in.crusher_type == "jaw")      tooling.tool_type = "jaw_crusher";
    else if (in.crusher_type == "cone")     tooling.tool_type = "cone_crusher";
    else if (in.crusher_type == "gyratory") tooling.tool_type = "gyratory_crusher";
    else if (in.crusher_type == "impact")   tooling.tool_type = "impact_crusher";
    else                                     tooling.tool_type = "ore_crusher";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "manganese_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 60.0;  // residence time scale for control
    tooling.extra = {
        { "process",          "comminution_crush" },
        { "feed_size_mm",     in.feed_size_mm },
        { "target_p80_mm",    in.target_p80_mm },
        { "crusher_type",     in.crusher_type },
        { "reduction_ratio",  ratio },
        { "process_category", "mineral_processing" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::ore_crush applied: feed={}mm P80={}mm type={} ratio={}",
                  in.feed_size_mm, in.target_p80_mm, in.crusher_type, ratio);

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
        rf.matched_geometry = { { "source", "feature_history" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::ore_crush
