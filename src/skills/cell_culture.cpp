// @lat: [[engine/skills#cell_culture]]

#include "cell_culture.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::cell_culture {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.cell_line.empty()) {
        r.add("DFM-INPUT", "error",
              "cell_culture cell_line must not be empty");
    }
    if (in.media_type.empty()) {
        r.add("DFM-INPUT", "error",
              "cell_culture media_type must not be empty");
    }

    if (in.days_in_culture <= 0.0) {
        r.add("DFM-CULT-TIME", "error",
              "cell_culture days_in_culture must be > 0 (got " +
              std::to_string(in.days_in_culture) + ")");
    } else if (in.days_in_culture > 60.0) {
        r.add("DFM-CULT-TIME", "info",
              "cell_culture days_in_culture " +
              std::to_string(in.days_in_culture) +
              " exceeds 60 d — primary lines typically senesce");
    }

    if (in.target_confluence_pct <= 0.0 ||
        in.target_confluence_pct > 100.0)
    {
        r.add("DFM-CULT-CONFL", "error",
              "cell_culture target_confluence_pct " +
              std::to_string(in.target_confluence_pct) +
              " must be in (0, 100]");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cell_culture DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "cell_line",              in.cell_line },
        { "media_type",             in.media_type },
        { "days_in_culture",        in.days_in_culture },
        { "target_confluence_pct",  in.target_confluence_pct },
    };

    json pattern = {
        { "kind",                  kSkillId },
        { "cell_line",             in.cell_line },
        { "media_type",            in.media_type },
        { "days_in_culture",       in.days_in_culture },
        { "target_confluence_pct", in.target_confluence_pct },
        { "process_family",        "in_vitro_culture" },
        { "geometry_changed",      false },
        { "is_process_only",       true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    // Long-duration runs prefer a controlled bioreactor; short runs stay in
    // a humidified CO2 incubator at the bench.
    tooling.tool_type         = (in.days_in_culture > 14.0)
                                   ? "bioreactor"
                                   : "co2_incubator";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "polystyrene_treated";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = in.days_in_culture * 86400.0;
    tooling.extra = {
        { "process",               "cell_culture" },
        { "cell_line",             in.cell_line },
        { "media_type",            in.media_type },
        { "days_in_culture",       in.days_in_culture },
        { "target_confluence_pct", in.target_confluence_pct },
        { "process_category",      "laboratory" },
        { "dfm_findings",          findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::cell_culture applied: line={} media={} d={} conf={}%",
                  in.cell_line, in.media_type, in.days_in_culture,
                  in.target_confluence_pct);

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

}  // namespace koocadcam::skill::cell_culture
