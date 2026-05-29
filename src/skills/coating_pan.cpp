// @lat: [[engine/skills#coating_pan]]

#include "coating_pan.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::coating_pan {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.coat_pickup_mg <= 0.0) {
        r.add("DFM-INPUT", "error",
              "coating_pan coat_pickup_mg must be > 0 (got " +
              std::to_string(in.coat_pickup_mg) + ")");
    } else if (in.coat_pickup_mg > 50.0) {
        r.add("DFM-CP-PICKUP-HIGH", "info",
              "coating_pan coat_pickup_mg " +
              std::to_string(in.coat_pickup_mg) +
              " > 50 mg — excessive film build, drying load high");
    }

    if (in.drying_temp_c <= 10.0 || in.drying_temp_c > 100.0) {
        r.add("DFM-INPUT", "error",
              "coating_pan drying_temp_c " +
              std::to_string(in.drying_temp_c) +
              " outside (10, 100] °C range");
    }

    if (in.coat_type != "film" &&
        in.coat_type != "enteric" &&
        in.coat_type != "sugar") {
        r.add("DFM-CP-TYPE", "info",
              "coating_pan coat_type '" + in.coat_type +
              "' unrecognised — expected 'film' | 'enteric' | 'sugar'");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "coating_pan DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "coat_type",      in.coat_type },
        { "coat_pickup_mg", in.coat_pickup_mg },
        { "drying_temp_c",  in.drying_temp_c },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "coat_type",        in.coat_type },
        { "coat_pickup_mg",   in.coat_pickup_mg },
        { "drying_temp_c",    in.drying_temp_c },
        { "process_family",   "pharma_coating" },
        { "geometric_change", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "coating_pan";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_316l_perforated";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;
    tooling.extra = {
        { "process",          "tablet_coating" },
        { "coat_type",        in.coat_type },
        { "coat_pickup_mg",   in.coat_pickup_mg },
        { "drying_temp_c",    in.drying_temp_c },
        { "process_category", "pharma_solid_dose" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::coating_pan applied: type={} pickup={}mg T={}C",
                  in.coat_type, in.coat_pickup_mg, in.drying_temp_c);

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

}  // namespace koocadcam::skill::coating_pan
