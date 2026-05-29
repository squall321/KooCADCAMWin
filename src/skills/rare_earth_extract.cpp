// @lat: [[engine/skills#rare_earth_extract]]

#include "rare_earth_extract.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::rare_earth_extract {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.element_target.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": element_target must be non-empty");
    } else if (in.element_target != "Nd" &&
               in.element_target != "Dy" &&
               in.element_target != "Pr") {
        r.add("DFM-REE-TARGET", "info",
              std::string(kSkillId) + ": element_target '" +
              in.element_target +
              "' unrecognised — expected 'Nd', 'Dy', or 'Pr'");
    }

    if (in.feed_kg <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": feed_kg must be > 0 (got " +
              std::to_string(in.feed_kg) + ")");
    }

    if (in.recovery_pct <= 0.0 || in.recovery_pct > 100.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": recovery_pct must be in (0, 100] (got " +
              std::to_string(in.recovery_pct) + ")");
    } else if (in.recovery_pct > 90.0) {
        r.add("DFM-REE-RECOVERY", "info",
              std::string(kSkillId) + ": recovery_pct " +
              std::to_string(in.recovery_pct) +
              " exceeds 90% — verify against pilot-scale solvent-extraction yield");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "rare_earth_extract DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double recoveredKg = in.feed_kg * in.recovery_pct / 100.0;

    json params = {
        { "feed_kg",        in.feed_kg },
        { "element_target", in.element_target },
        { "recovery_pct",   in.recovery_pct },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "feed_kg",            in.feed_kg },
        { "element_target",     in.element_target },
        { "recovery_pct",       in.recovery_pct },
        { "recovered_kg",       recoveredKg },
        { "geometry_changed",   false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "solvent_extraction_train";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Mixer-settler bank: ~300 s/kg + 1800 s setup.
    tooling.est_cycle_time_s  = 1800.0 + 300.0 * in.feed_kg;
    tooling.extra = {
        { "process",          "rare_earth_extract" },
        { "feed_kg",          in.feed_kg },
        { "element_target",   in.element_target },
        { "recovered_kg",     recoveredKg },
        { "process_category", "circular_economy" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::rare_earth_extract applied: feed={}kg target={} recovery={}% recovered={}kg",
        in.feed_kg, in.element_target, in.recovery_pct, recoveredKg);

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

}  // namespace koocadcam::skill::rare_earth_extract
