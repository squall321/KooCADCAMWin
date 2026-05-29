// @lat: [[engine/skills#fabric_finishing]]

#include "fabric_finishing.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::fabric_finishing {

using nlohmann::json;

namespace {

bool isKnownTechnique(const std::string& s)
{
    return s == "sanforize" || s == "calendering" || s == "mercerize" ||
           s == "heatset" || s == "raising";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.finishing_temp_c <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": finishing_temp_c must be > 0 (got " +
              std::to_string(in.finishing_temp_c) + ")");
    }

    if (!isKnownTechnique(in.technique)) {
        r.add("DFM-TEX-FINISH", "info",
              std::string(kSkillId) + ": technique '" + in.technique +
              "' unrecognised — expected sanforize | calendering | mercerize | heatset | raising");
    }

    if (in.finishing_temp_c > 230.0) {
        r.add("DFM-TEX-FINISH-TEMP", "info",
              std::string(kSkillId) + ": finishing_temp_c " +
              std::to_string(in.finishing_temp_c) +
              " > 230 °C — fibre yellowing / strength loss risk");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "fabric_finishing DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "technique",        in.technique },
        { "finishing_temp_c", in.finishing_temp_c },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "technique",         in.technique },
        { "finishing_temp_c",  in.finishing_temp_c },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "finishing_range";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "chromed_steel_roll";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Continuous finishing line: ~ 30 m/min → seconds per metre of fabric.
    tooling.est_cycle_time_s  = 2.0;
    tooling.extra = {
        { "process",          "fabric_finishing" },
        { "technique",        in.technique },
        { "finishing_temp_c", in.finishing_temp_c },
        { "process_category", "textile" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::fabric_finishing applied: technique={} T={}°C",
        in.technique, in.finishing_temp_c);

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

}  // namespace koocadcam::skill::fabric_finishing
