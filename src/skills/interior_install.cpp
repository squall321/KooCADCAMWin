// @lat: [[engine/skills#interior_install]]

#include "interior_install.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::interior_install {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.component_count <= 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": component_count must be > 0 (got " +
              std::to_string(in.component_count) + ")");
    }
    if (in.harness_length_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": harness_length_m must be > 0 (got " +
              std::to_string(in.harness_length_m) + ")");
    }
    if (in.component_count > 100) {
        r.add("DFM-INTERIOR-COMPONENT", "info",
              std::string(kSkillId) + ": component_count " +
              std::to_string(in.component_count) +
              " > 100 — consider splitting across multiple stations");
    }
    if (in.harness_length_m > 200.0) {
        r.add("DFM-INTERIOR-HARNESS", "info",
              std::string(kSkillId) + ": harness_length_m " +
              std::to_string(in.harness_length_m) +
              " > 200 — exceeds typical passenger-car routed harness");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "interior_install DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "component_count",  in.component_count },
        { "harness_length_m", in.harness_length_m },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "component_count",  in.component_count },
        { "harness_length_m", in.harness_length_m },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "interior_install_station";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // 30 s / component + 8 s / m harness + 30 s station handshake.
    tooling.est_cycle_time_s  = 30.0
                              + 30.0 * static_cast<double>(in.component_count)
                              + 8.0  * in.harness_length_m;
    tooling.extra = {
        { "process",          "interior_install" },
        { "component_count",  in.component_count },
        { "harness_length_m", in.harness_length_m },
        { "process_category", "automotive_assembly" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::interior_install applied: components={} harness={}m",
        in.component_count, in.harness_length_m);

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

}  // namespace koocadcam::skill::interior_install
