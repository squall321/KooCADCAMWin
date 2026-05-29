// @lat: [[engine/skills#food_packaging]]

#include "food_packaging.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::food_packaging {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownPackageTypes()
{
    static const std::unordered_set<std::string> s = {
        "vacuum", "map_tray", "map_pouch", "flow_wrap", "thermoform",
    };
    return s;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.package_type.empty() ||
        knownPackageTypes().count(in.package_type) == 0) {
        r.add("DFM-INPUT", "error",
              "food_packaging unknown package_type '" + in.package_type +
              "' (expected vacuum | map_tray | map_pouch | flow_wrap | "
              "thermoform)");
    }

    if (in.gas_mix.empty()) {
        r.add("DFM-PKG-GAS", "error",
              "food_packaging gas_mix must not be empty");
    }

    if (in.oxygen_pct < 0.0 || in.oxygen_pct > 21.0) {
        r.add("DFM-PKG-OXYGEN", "error",
              "food_packaging oxygen_pct " +
              std::to_string(in.oxygen_pct) +
              " outside [0, 21] %% range");
    } else if (in.package_type == "vacuum" && in.oxygen_pct > 1.0) {
        r.add("DFM-PKG-VACUUM", "info",
              "food_packaging vacuum pack with oxygen_pct " +
              std::to_string(in.oxygen_pct) +
              " %% > 1 %% — verify vacuum-pump performance");
    } else if (in.oxygen_pct > 10.0) {
        r.add("DFM-PKG-HIGH-O2", "info",
              "food_packaging oxygen_pct " +
              std::to_string(in.oxygen_pct) +
              " %% > 10 %% — shelf-life risk for oxygen-sensitive product");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "food_packaging DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "package_type", in.package_type },
        { "gas_mix",      in.gas_mix },
        { "oxygen_pct",   in.oxygen_pct },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "package_type",     in.package_type },
        { "gas_mix",          in.gas_mix },
        { "oxygen_pct",       in.oxygen_pct },
        { "process_family",   "food_processing" },
        { "geometric_change", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "food_packaging_" + in.package_type;
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_316l_food_grade";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;
    tooling.extra = {
        { "process",          "food_packaging" },
        { "package_type",     in.package_type },
        { "gas_mix",          in.gas_mix },
        { "oxygen_pct",       in.oxygen_pct },
        { "process_category", "food_processing" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::food_packaging applied: pkg={} gas={} O2={}%%",
                  in.package_type, in.gas_mix, in.oxygen_pct);

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

}  // namespace koocadcam::skill::food_packaging
