// @lat: [[engine/skills#hydrogen_purify]]

#include "hydrogen_purify.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::hydrogen_purify {

using nlohmann::json;

namespace {

bool isKnownMethod(const std::string& m)
{
    return m == "PSA" || m == "cryogenic" || m == "electrolysis";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (!isKnownMethod(in.method)) {
        r.add("DFM-H2-METHOD", "info",
              std::string(kSkillId) + ": method '" + in.method +
              "' not in {PSA, cryogenic, electrolysis}");
    }

    if (in.purity_pct <= 0.0 || in.purity_pct >= 100.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": purity_pct must be in (0, 100) — got " +
              std::to_string(in.purity_pct));
    } else if (in.purity_pct < 99.0) {
        r.add("DFM-H2-PURITY", "info",
              std::string(kSkillId) + ": purity_pct " +
              std::to_string(in.purity_pct) +
              " < 99 % — typically too low for industrial / fuel-cell H2");
    }

    if (in.flow_nm3_h <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": flow_nm3_h must be > 0 (got " +
              std::to_string(in.flow_nm3_h) + ")");
    } else if (in.method == "PSA" &&
               (in.flow_nm3_h < 100.0 || in.flow_nm3_h > 50000.0)) {
        r.add("DFM-H2-FLOW", "info",
              std::string(kSkillId) + ": PSA flow_nm3_h " +
              std::to_string(in.flow_nm3_h) +
              " outside typical PSA envelope [100, 50000]");
    } else if (in.method == "cryogenic" && in.flow_nm3_h < 5000.0) {
        r.add("DFM-H2-FLOW", "info",
              std::string(kSkillId) + ": cryogenic flow_nm3_h " +
              std::to_string(in.flow_nm3_h) +
              " < 5000 — cryo plants are economic only at large scale");
    } else if (in.method == "electrolysis" && in.flow_nm3_h > 1000.0) {
        r.add("DFM-H2-FLOW", "info",
              std::string(kSkillId) + ": electrolysis flow_nm3_h " +
              std::to_string(in.flow_nm3_h) +
              " > 1000 — large for electrochemical purification stage");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "hydrogen_purify DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "method",     in.method },
        { "purity_pct", in.purity_pct },
        { "flow_nm3_h", in.flow_nm3_h },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "method",           in.method },
        { "purity_pct",       in.purity_pct },
        { "flow_nm3_h",       in.flow_nm3_h },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    if (in.method == "cryogenic")
        tooling.tool_type = "cryo_distillation_unit";
    else if (in.method == "electrolysis")
        tooling.tool_type = "pem_pd_membrane";
    else
        tooling.tool_type = "psa_adsorber";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Throughput is steady-state; report 1 h batch for reporting purposes.
    tooling.est_cycle_time_s  = 3600.0;
    tooling.extra = {
        { "process",          "hydrogen_purify" },
        { "method",           in.method },
        { "purity_pct",       in.purity_pct },
        { "flow_nm3_h",       in.flow_nm3_h },
        { "process_category", "gas_separation" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::hydrogen_purify applied: method={} purity={}% flow={}Nm3/h",
        in.method, in.purity_pct, in.flow_nm3_h);

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

}  // namespace koocadcam::skill::hydrogen_purify
