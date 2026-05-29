// @lat: [[engine/skills#belt_replace]]

#include "belt_replace.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::belt_replace {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.belt_id.empty()) {
        r.add("DFM-INPUT", "error",
              "belt_replace belt_id must be non-empty");
    }

    if (in.belt_type != "V" && in.belt_type != "timing" && in.belt_type != "flat") {
        r.add("DFM-BELT-TYPE", "info",
              "belt_replace belt_type '" + in.belt_type +
              "' not in {V, timing, flat} — treated as generic");
    }

    if (in.tension_n <= 0.0) {
        r.add("DFM-INPUT", "error",
              "belt_replace tension_n must be > 0 (got " +
              std::to_string(in.tension_n) + ")");
    } else if (in.tension_n < 50.0 || in.tension_n > 5000.0) {
        r.add("DFM-BELT-TENSION", "info",
              "belt_replace tension_n " + std::to_string(in.tension_n) +
              " outside typical 50–5000 N range");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "belt_replace DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string canonicalType =
        (in.belt_type == "V" || in.belt_type == "timing" || in.belt_type == "flat")
            ? in.belt_type : "generic";

    json params = {
        { "belt_type", in.belt_type },
        { "belt_id",   in.belt_id },
        { "tension_n", in.tension_n },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "belt_type",        canonicalType },
        { "belt_id",          in.belt_id },
        { "tension_n",        in.tension_n },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = (canonicalType == "timing")
                                  ? "timing_belt_tension_gauge"
                                  : "belt_tension_gauge";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "rubber_polyester";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 1200.0;    // ~20 min belt swap + retension
    tooling.extra = {
        { "process",      "belt_swap" },
        { "belt_type",    canonicalType },
        { "belt_id",      in.belt_id },
        { "tension_n",    in.tension_n },
        { "dfm_findings", findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::belt_replace applied: type={} id={} tension={} N",
                  canonicalType, in.belt_id, in.tension_n);

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

}  // namespace koocadcam::skill::belt_replace
