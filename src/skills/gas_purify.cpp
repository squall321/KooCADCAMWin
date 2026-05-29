// @lat: [[engine/skills#gas_purify]]

#include "gas_purify.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::gas_purify {

using nlohmann::json;

namespace {

bool isKnownMethod(const std::string& m)
{
    return m == "getter" || m == "cold_trap" ||
           m == "Pd_membrane" || m == "mol_sieve";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.gas_species.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + " gas_species must be non-empty");
    }

    if (!isKnownMethod(in.purification_method)) {
        r.add("DFM-GAS-METHOD", "info",
              std::string(kSkillId) + " purification_method '" +
              in.purification_method +
              "' not in {getter, cold_trap, Pd_membrane, mol_sieve}");
    }

    if (in.final_purity_ppb <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              " final_purity_ppb must be > 0 (got " +
              std::to_string(in.final_purity_ppb) + ")");
    } else if (in.final_purity_ppb > 1000.0) {
        r.add("DFM-GAS-PURITY", "warning",
              std::string(kSkillId) +
              " final_purity_ppb " + std::to_string(in.final_purity_ppb) +
              " > 1000 ppb — not UHV-grade, expect outgassing penalty");
    }

    // Method/gas compatibility heuristic.
    if (in.purification_method == "Pd_membrane" && in.gas_species != "hydrogen") {
        r.add("DFM-GAS-METHOD", "warning",
              std::string(kSkillId) +
              " Pd_membrane only purifies hydrogen — got '" +
              in.gas_species + "'");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "gas_purify DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string grade =
        in.final_purity_ppb <= 1.0    ? "UHP-9N" :
        in.final_purity_ppb <= 10.0   ? "UHP-8N" :
        in.final_purity_ppb <= 100.0  ? "UHP-7N" :
        in.final_purity_ppb <= 1000.0 ? "UHP-6N" :
                                        "industrial";

    json params = {
        { "gas_species",         in.gas_species },
        { "purification_method", in.purification_method },
        { "final_purity_ppb",    in.final_purity_ppb },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "gas_species",         in.gas_species },
        { "purification_method", in.purification_method },
        { "final_purity_ppb",    in.final_purity_ppb },
        { "purity_grade",        grade },
        { "geometry_changed",    false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "gas_purifier";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = (in.purification_method == "Pd_membrane")
                                    ? "palladium" : "stainless_steel_316L";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 1800.0;   // ~30 min steady-state flush
    tooling.extra = {
        { "process",             "gas_purify" },
        { "gas_species",         in.gas_species },
        { "purification_method", in.purification_method },
        { "final_purity_ppb",    in.final_purity_ppb },
        { "purity_grade",        grade },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::gas_purify applied: gas={} method={} purity={}ppb grade={}",
                  in.gas_species, in.purification_method,
                  in.final_purity_ppb, grade);

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

}  // namespace koocadcam::skill::gas_purify
