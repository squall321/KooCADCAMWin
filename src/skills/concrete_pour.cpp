// @lat: [[engine/skills#concrete_pour]]

#include "concrete_pour.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::concrete_pour {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.volume_m3 <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": volume_m3 must be > 0 (got " +
              std::to_string(in.volume_m3) + ")");
    }

    if (in.mix_design.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": mix_design must be non-empty (e.g. 'C25/30')");
    }

    if (in.slump_mm < 0.0 || in.slump_mm > 250.0) {
        r.add("DFM-CONC-SLUMP", "error",
              std::string(kSkillId) + ": slump_mm " +
              std::to_string(in.slump_mm) +
              " outside [0, 250] mm — outside ASTM C143 measurable range");
    }

    if (in.volume_m3 > 100.0) {
        r.add("DFM-CONC-VOL", "info",
              std::string(kSkillId) + ": volume_m3 " +
              std::to_string(in.volume_m3) +
              " > 100 — stage pour with cold joints planned");
    }

    if (in.slump_mm > 220.0) {
        r.add("DFM-CONC-SLUMP", "info",
              std::string(kSkillId) + ": slump_mm " +
              std::to_string(in.slump_mm) +
              " > 220 — self-consolidating concrete; verify mix is SCC, not over-watered");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "concrete_pour DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "volume_m3",  in.volume_m3 },
        { "mix_design", in.mix_design },
        { "slump_mm",   in.slump_mm },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "volume_m3",         in.volume_m3 },
        { "mix_design",        in.mix_design },
        { "slump_mm",          in.slump_mm },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "concrete_pump";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    // Concrete is ADDED, not removed.  Record as negative mm3 like spot_weld.
    tooling.stock_removed_mm3 = -in.volume_m3 * 1.0e9;     // m3 → mm3
    // Boom-pump placement rate ~ 30 m3/h; add 15 min finishing per pour.
    tooling.est_cycle_time_s  = (in.volume_m3 / 30.0) * 3600.0 + 900.0;
    tooling.extra = {
        { "process",          "concrete_place_finish" },
        { "volume_m3",        in.volume_m3 },
        { "mix_design",       in.mix_design },
        { "slump_mm",         in.slump_mm },
        { "code_reference",   "EN_206" },
        { "process_category", "construction_concrete" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::concrete_pour applied: volume={}m3 mix={} slump={}mm",
        in.volume_m3, in.mix_design, in.slump_mm);

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

}  // namespace koocadcam::skill::concrete_pour
