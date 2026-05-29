// @lat: [[engine/skills#plastic_pyrolysis]]

#include "plastic_pyrolysis.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::plastic_pyrolysis {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.feedstock_kg <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": feedstock_kg must be > 0 (got " +
              std::to_string(in.feedstock_kg) + ")");
    }

    if (in.temp_c < 300.0) {
        r.add("DFM-PYRO-TEMP", "info",
              std::string(kSkillId) + ": temp_c " +
              std::to_string(in.temp_c) +
              " below 300°C — insufficient for thermal depolymerization");
    } else if (in.temp_c > 800.0) {
        r.add("DFM-PYRO-TEMP", "info",
              std::string(kSkillId) + ": temp_c " +
              std::to_string(in.temp_c) +
              " above 800°C — risk of gas-phase favored over oil yield");
    }

    if (in.oil_yield_pct <= 0.0 || in.oil_yield_pct > 100.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": oil_yield_pct must be in (0, 100] (got " +
              std::to_string(in.oil_yield_pct) + ")");
    } else if (in.oil_yield_pct > 85.0) {
        r.add("DFM-PYRO-YIELD", "info",
              std::string(kSkillId) + ": oil_yield_pct " +
              std::to_string(in.oil_yield_pct) +
              " exceeds 85% — atypical for mixed feedstock, verify assay");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "plastic_pyrolysis DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double recoveredOilKg = in.feedstock_kg * in.oil_yield_pct / 100.0;

    json params = {
        { "feedstock_kg",   in.feedstock_kg },
        { "temp_c",         in.temp_c },
        { "oil_yield_pct",  in.oil_yield_pct },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "feedstock_kg",       in.feedstock_kg },
        { "temp_c",             in.temp_c },
        { "oil_yield_pct",      in.oil_yield_pct },
        { "recovered_oil_kg",   recoveredOilKg },
        { "geometry_changed",   false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "pyrolysis_reactor";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Batch reactor: ~60 s/kg + 600 s setup/cooldown.
    tooling.est_cycle_time_s  = 600.0 + 60.0 * in.feedstock_kg;
    tooling.extra = {
        { "process",          "plastic_pyrolysis" },
        { "feedstock_kg",     in.feedstock_kg },
        { "temp_c",           in.temp_c },
        { "recovered_oil_kg", recoveredOilKg },
        { "process_category", "circular_economy" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::plastic_pyrolysis applied: feed={}kg T={}°C yield={}% oil={}kg",
        in.feedstock_kg, in.temp_c, in.oil_yield_pct, recoveredOilKg);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != std::string(kSkillId)) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::plastic_pyrolysis
