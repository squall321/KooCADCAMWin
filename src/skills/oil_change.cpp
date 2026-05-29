// @lat: [[engine/skills#oil_change]]

#include "oil_change.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::oil_change {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.oil_grade.empty()) {
        r.add("DFM-INPUT", "error",
              "oil_change oil_grade must be non-empty");
    }

    if (in.volume_l <= 0.0) {
        r.add("DFM-INPUT", "error",
              "oil_change volume_l must be > 0 (got " +
              std::to_string(in.volume_l) + ")");
    }

    if (in.drain_temp_c < 30.0 || in.drain_temp_c > 90.0) {
        r.add("DFM-OIL-TEMP", "info",
              "oil_change drain_temp_c " + std::to_string(in.drain_temp_c) +
              " outside typical 30–90 °C drain window (cold = slow drain, "
              "hot = burn risk)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "oil_change DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "oil_grade",    in.oil_grade },
        { "volume_l",     in.volume_l },
        { "drain_temp_c", in.drain_temp_c },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "oil_grade",        in.oil_grade },
        { "volume_l",         in.volume_l },
        { "drain_temp_c",     in.drain_temp_c },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "drain_pan_and_fill_pump";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 1200.0;    // ~20 min drain + refill
    tooling.extra = {
        { "process",      "lubricant_swap" },
        { "oil_grade",    in.oil_grade },
        { "volume_l",     in.volume_l },
        { "drain_temp_c", in.drain_temp_c },
        { "dfm_findings", findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::oil_change applied: grade={} vol={}L drain_T={}°C",
                  in.oil_grade, in.volume_l, in.drain_temp_c);

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

}  // namespace koocadcam::skill::oil_change
