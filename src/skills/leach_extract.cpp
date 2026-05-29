// @lat: [[engine/skills#leach_extract]]

#include "leach_extract.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::leach_extract {

using nlohmann::json;

namespace {

// Categorise the lixiviant by pH band — used purely for metadata + tooling
// selection (downstream materials of construction differ).
std::string lixiviantClass(double ph)
{
    if (ph <  4.0) return "acid";
    if (ph <  8.5) return "neutral";
    return            "alkaline";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.ph < 0.0 || in.ph > 14.0) {
        r.add("DFM-INPUT", "error",
              "leach_extract ph must be in [0, 14] (got " +
              std::to_string(in.ph) + ")");
    }

    if (in.leach_time_h <= 0.0) {
        r.add("DFM-INPUT", "error",
              "leach_extract leach_time_h must be > 0 (got " +
              std::to_string(in.leach_time_h) + ")");
    }

    if (in.recovery_pct <= 0.0 || in.recovery_pct > 100.0) {
        r.add("DFM-INPUT", "error",
              "leach_extract recovery_pct must be in (0, 100] (got " +
              std::to_string(in.recovery_pct) + ")");
    } else if (in.recovery_pct < 50.0) {
        r.add("DFM-RECOVERY-LOW", "info",
              "leach_extract recovery_pct " + std::to_string(in.recovery_pct) +
              " < 50% — sub-economic; consider grind / reagent dose review");
    }

    if (in.ph >= 0.0 && in.ph <= 14.0) {
        if (in.ph < 1.0) {
            r.add("DFM-PH-EXTREME", "info",
                  "leach_extract ph " + std::to_string(in.ph) +
                  " < 1 — severe acid corrosion; use HDPE/FRP linings");
        } else if (in.ph > 13.0) {
            r.add("DFM-PH-EXTREME", "info",
                  "leach_extract ph " + std::to_string(in.ph) +
                  " > 13 — strong caustic; verify gasket compatibility");
        }
    }

    if (in.leach_time_h > 240.0) {
        r.add("DFM-LEACH-LONG", "info",
              "leach_extract leach_time_h " + std::to_string(in.leach_time_h) +
              " > 240 h — heap/dump regime; reactor sizing not applicable");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "leach_extract DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string lix_class = lixiviantClass(in.ph);

    json params = {
        { "ph",           in.ph },
        { "leach_time_h", in.leach_time_h },
        { "recovery_pct", in.recovery_pct },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "ph",               in.ph },
        { "leach_time_h",     in.leach_time_h },
        { "recovery_pct",     in.recovery_pct },
        { "lixiviant_class",  lix_class },
        { "process_family",   "hydrometallurgy_leach" },
        { "geometry_changed", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "leach_tank";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    if      (lix_class == "acid")     tooling.tool_material = "rubber_lined_steel";
    else if (lix_class == "alkaline") tooling.tool_material = "carbon_steel";
    else                               tooling.tool_material = "stainless_steel_316";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = in.leach_time_h * 3600.0;
    tooling.extra = {
        { "process",          "hydrometallurgy_leach" },
        { "ph",               in.ph },
        { "leach_time_h",     in.leach_time_h },
        { "recovery_pct",     in.recovery_pct },
        { "lixiviant_class",  lix_class },
        { "process_category", "mineral_processing" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::leach_extract applied: ph={} t={}h recovery={}% class={}",
                  in.ph, in.leach_time_h, in.recovery_pct, lix_class);

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

}  // namespace koocadcam::skill::leach_extract
