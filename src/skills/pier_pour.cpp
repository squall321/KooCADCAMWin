// @lat: [[engine/skills#pier_pour]]

#include "pier_pour.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::pier_pour {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.pier_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": pier_id must be non-empty (e.g. 'P3')");
    }

    if (in.volume_m3 <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": volume_m3 must be > 0 (got " +
              std::to_string(in.volume_m3) + ")");
    }

    if (in.mix_grade.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": mix_grade must be non-empty (e.g. 'C40/50')");
    }

    if (in.volume_m3 > 80.0) {
        r.add("DFM-PIER-VOL", "info",
              std::string(kSkillId) + ": volume_m3 " +
              std::to_string(in.volume_m3) +
              " > 80 — plan staged pour with temp control to avoid mass-concrete cracking");
    }

    if (in.volume_m3 > 0.0 && in.volume_m3 < 1.0) {
        r.add("DFM-PIER-VOL", "info",
              std::string(kSkillId) + ": volume_m3 " +
              std::to_string(in.volume_m3) +
              " < 1.0 — unusually small for a structural pier; verify geometry");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pier_pour DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "pier_id",   in.pier_id },
        { "volume_m3", in.volume_m3 },
        { "mix_grade", in.mix_grade },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "pier_id",           in.pier_id },
        { "volume_m3",         in.volume_m3 },
        { "mix_grade",         in.mix_grade },
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
    tooling.stock_removed_mm3 = 0.0;
    // Pier boom-pump placement ~ 25 m3/h; add 30 min for finishing/curing setup.
    tooling.est_cycle_time_s  = (in.volume_m3 / 25.0) * 3600.0 + 1800.0;
    tooling.extra = {
        { "process",          "bridge_pier_pour" },
        { "pier_id",          in.pier_id },
        { "volume_m3",        in.volume_m3 },
        { "mix_grade",        in.mix_grade },
        { "code_reference",   "AASHTO_LRFD" },
        { "process_category", "civil_bridge_concrete" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::pier_pour applied: pier={} vol={}m3 mix={}",
        in.pier_id, in.volume_m3, in.mix_grade);

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

}  // namespace koocadcam::skill::pier_pour
