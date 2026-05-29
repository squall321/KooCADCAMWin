// @lat: [[engine/skills#propulsion_install]]

#include "propulsion_install.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::propulsion_install {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.propulsion_type.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": propulsion_type must be non-empty");
    }

    if (in.power_kw <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": power_kw must be > 0 (got " +
              std::to_string(in.power_kw) + ")");
    }

    if (in.shaft_dia_mm < 50.0 || in.shaft_dia_mm > 1500.0) {
        r.add("DFM-PROP-SHAFT", "error",
              std::string(kSkillId) + ": shaft_dia_mm " +
              std::to_string(in.shaft_dia_mm) +
              " outside [50, 1500] mm — outside class-society shaft range");
    }

    if (in.power_kw > 50000.0) {
        r.add("DFM-PROP-POWER", "info",
              std::string(kSkillId) + ": power_kw " +
              std::to_string(in.power_kw) +
              " > 50000 — class-society design review + torsional analysis advisory");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "propulsion_install DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "propulsion_type", in.propulsion_type },
        { "power_kw",        in.power_kw },
        { "shaft_dia_mm",    in.shaft_dia_mm },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "propulsion_type",   in.propulsion_type },
        { "power_kw",          in.power_kw },
        { "shaft_dia_mm",      in.shaft_dia_mm },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "engine_room_overhead_crane";
    tooling.tool_dia_mm       = in.shaft_dia_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "carbon_steel_shaft";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Allow ~ 12 h base + 1 h per 5 MW of installed power for shaft alignment.
    tooling.est_cycle_time_s  =
        12.0 * 3600.0 + (in.power_kw / 5000.0) * 3600.0;
    tooling.extra = {
        { "process",          "propulsion_machinery_install" },
        { "propulsion_type",  in.propulsion_type },
        { "power_kw",         in.power_kw },
        { "shaft_dia_mm",     in.shaft_dia_mm },
        { "code_reference",   "ABS_part4_propulsion" },
        { "process_category", "shipbuilding_machinery" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::propulsion_install applied: type={} power={}kW shaft={}mm",
        in.propulsion_type, in.power_kw, in.shaft_dia_mm);

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

}  // namespace koocadcam::skill::propulsion_install
