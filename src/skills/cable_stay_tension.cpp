// @lat: [[engine/skills#cable_stay_tension]]

#include "cable_stay_tension.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <string>

namespace koocadcam::skill::cable_stay_tension {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.cable_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": cable_id must be non-empty (e.g. 'S12N')");
    }

    if (in.design_tension_kn <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": design_tension_kn must be > 0 (got " +
              std::to_string(in.design_tension_kn) + ")");
    }

    if (in.jack_force_kn <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": jack_force_kn must be > 0 (got " +
              std::to_string(in.jack_force_kn) + ")");
    }

    if (in.design_tension_kn > 0.0 && in.jack_force_kn > 0.0) {
        const double dev_pct = (in.jack_force_kn - in.design_tension_kn)
                              / in.design_tension_kn * 100.0;
        const double abs_dev = std::fabs(dev_pct);
        if (abs_dev > 5.0) {
            r.add("DFM-CABLE-TENSION", "error",
                  std::string(kSkillId) + ": tension deviation " +
                  std::to_string(dev_pct) +
                  "% exceeds ±5% tolerance — re-stress and re-measure");
        } else if (abs_dev > 2.0) {
            r.add("DFM-CABLE-TENSION", "info",
                  std::string(kSkillId) + ": tension deviation " +
                  std::to_string(dev_pct) +
                  "% > 2% — note for load-balance review");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cable_stay_tension DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double dev_pct = (in.jack_force_kn - in.design_tension_kn)
                          / in.design_tension_kn * 100.0;

    json params = {
        { "cable_id",          in.cable_id },
        { "design_tension_kn", in.design_tension_kn },
        { "jack_force_kn",     in.jack_force_kn },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "cable_id",                in.cable_id },
        { "design_tension_kn",       in.design_tension_kn },
        { "jack_force_kn",           in.jack_force_kn },
        { "tension_deviation_pct",   dev_pct },
        { "geometry_changed",        false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "hydraulic_stressing_jack";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Single-cable stressing + lock-off ~ 20 min per cable.
    tooling.est_cycle_time_s  = 1200.0;
    tooling.extra = {
        { "process",                 "cable_stay_stress" },
        { "cable_id",                in.cable_id },
        { "design_tension_kn",       in.design_tension_kn },
        { "jack_force_kn",           in.jack_force_kn },
        { "tension_deviation_pct",   dev_pct },
        { "code_reference",          "PTI_DC45_1" },
        { "process_category",        "civil_bridge_posttension" },
        { "geometric_no_op",         true },
        { "dfm_findings",            findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::cable_stay_tension applied: cable={} design={}kN jack={}kN dev={}%",
        in.cable_id, in.design_tension_kn, in.jack_force_kn, dev_pct);

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

}  // namespace koocadcam::skill::cable_stay_tension
