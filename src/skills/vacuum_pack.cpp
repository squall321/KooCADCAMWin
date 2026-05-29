// @lat: [[engine/skills#vacuum_pack]]

#include "vacuum_pack.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::vacuum_pack {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.vacuum_mbar <= 0.0) {
        r.add("DFM-INPUT", "error",
              "vacuum_pack vacuum_mbar must be > 0 (got " +
              std::to_string(in.vacuum_mbar) + ")");
    }

    if (in.seal_time_s <= 0.0) {
        r.add("DFM-INPUT", "error",
              "vacuum_pack seal_time_s must be > 0 (got " +
              std::to_string(in.seal_time_s) + ")");
    }

    if (in.vacuum_mbar > 100.0) {
        r.add("DFM-VAC-WEAK", "info",
              "vacuum_pack vacuum_mbar " + std::to_string(in.vacuum_mbar) +
              " > 100 mbar — weak vacuum, oxygen permeation accelerated");
    }

    if (in.seal_time_s > 10.0) {
        r.add("DFM-VAC-TIME-LONG", "info",
              "vacuum_pack seal_time_s " + std::to_string(in.seal_time_s) +
              " > 10 s — long cycle, energy cost elevated");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "vacuum_pack DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "vacuum_mbar", in.vacuum_mbar },
        { "seal_time_s", in.seal_time_s },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "vacuum_mbar",      in.vacuum_mbar },
        { "seal_time_s",      in.seal_time_s },
        { "process_family",   "vacuum_packaging" },
        { "geometry_changed", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "vacuum_chamber_sealer";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_chamber";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Cycle: evacuation (~ proportional to log of vacuum) + seal time.
    tooling.est_cycle_time_s  = 5.0 + in.seal_time_s;
    tooling.extra = {
        { "process",          "vacuum_pack" },
        { "vacuum_mbar",      in.vacuum_mbar },
        { "seal_time_s",      in.seal_time_s },
        { "process_category", "packaging" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::vacuum_pack applied: vac={}mbar seal_t={}s",
                  in.vacuum_mbar, in.seal_time_s);

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

}  // namespace koocadcam::skill::vacuum_pack
