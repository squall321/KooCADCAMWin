// @lat: [[engine/skills#fertilize_apply]]

#include "fertilize_apply.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cstddef>
#include <memory>
#include <string>

namespace koocadcam::skill::fertilize_apply {

using nlohmann::json;

namespace {

// Parse "N-P-K" into three doubles.  Returns true on full success.
bool parseNpk(const std::string& s, double& n, double& p, double& k)
{
    std::size_t d1 = s.find('-');
    if (d1 == std::string::npos) return false;
    std::size_t d2 = s.find('-', d1 + 1);
    if (d2 == std::string::npos) return false;
    try {
        n = std::stod(s.substr(0, d1));
        p = std::stod(s.substr(d1 + 1, d2 - d1 - 1));
        k = std::stod(s.substr(d2 + 1));
    } catch (...) {
        return false;
    }
    return true;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.fertilizer_type.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": fertilizer_type must be non-empty");
    }

    if (in.npk_ratio.empty()) {
        r.add("DFM-FERT-NPK", "info",
              std::string(kSkillId) + ": npk_ratio empty — none recorded");
    } else {
        double n = 0.0, p = 0.0, k = 0.0;
        if (!parseNpk(in.npk_ratio, n, p, k)) {
            r.add("DFM-FERT-NPK", "info",
                  std::string(kSkillId) + ": npk_ratio '" + in.npk_ratio +
                  "' not parseable as 'N-P-K'");
        } else if (n + p + k > 100.0) {
            r.add("DFM-FERT-NPK", "warning",
                  std::string(kSkillId) + ": npk_ratio sum " +
                  std::to_string(n + p + k) +
                  " > 100 — inconsistent ratio");
        }
    }

    if (in.dose_kg_ha <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": dose_kg_ha must be > 0 (got " +
              std::to_string(in.dose_kg_ha) + ")");
    } else if (in.dose_kg_ha > 2000.0) {
        r.add("DFM-FERT-DOSE", "error",
              std::string(kSkillId) + ": dose_kg_ha " +
              std::to_string(in.dose_kg_ha) +
              " > 2000 — exceeds typical agronomic cap");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "fertilize_apply DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double n = 0.0, pp = 0.0, k = 0.0;
    const bool npkOk = parseNpk(in.npk_ratio, n, pp, k);

    json params = {
        { "fertilizer_type", in.fertilizer_type },
        { "npk_ratio",       in.npk_ratio },
        { "dose_kg_ha",      in.dose_kg_ha },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "fertilizer_type",  in.fertilizer_type },
        { "npk_ratio",        in.npk_ratio },
        { "npk_parsed",       npkOk },
        { "dose_kg_ha",       in.dose_kg_ha },
        { "geometry_changed", false },
    };
    if (npkOk) {
        pattern["n_pct"] = n;
        pattern["p_pct"] = pp;
        pattern["k_pct"] = k;
    }

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "fertilizer_broadcaster";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 1200.0;  // ~20 min per typical pass
    tooling.extra = {
        { "process",          "fertilize_apply" },
        { "fertilizer_type",  in.fertilizer_type },
        { "npk_ratio",        in.npk_ratio },
        { "dose_kg_ha",       in.dose_kg_ha },
        { "process_category", "agriculture" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::fertilize_apply applied: type={} npk={} dose={}kg/ha",
        in.fertilizer_type, in.npk_ratio, in.dose_kg_ha);

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

}  // namespace koocadcam::skill::fertilize_apply
