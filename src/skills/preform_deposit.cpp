// @lat: [[engine/skills#preform_deposit]]

#include "preform_deposit.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace koocadcam::skill::preform_deposit {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.core_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": core_dia_mm must be > 0 (got " +
              std::to_string(in.core_dia_mm) + ")");
    }
    if (in.clad_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": clad_dia_mm must be > 0 (got " +
              std::to_string(in.clad_dia_mm) + ")");
    }

    if (in.core_dia_mm > 0.0 && in.clad_dia_mm > 0.0 &&
        in.core_dia_mm >= in.clad_dia_mm) {
        r.add("DFM-PREFORM-GEOMETRY", "error",
              std::string(kSkillId) + ": core_dia_mm (" +
              std::to_string(in.core_dia_mm) +
              ") must be strictly less than clad_dia_mm (" +
              std::to_string(in.clad_dia_mm) + ")");
    }

    if (in.dopant_concentration_pct < 0.1 ||
        in.dopant_concentration_pct > 30.0) {
        r.add("DFM-PREFORM-DOPANT", "error",
              std::string(kSkillId) + ": dopant_concentration_pct " +
              std::to_string(in.dopant_concentration_pct) +
              " not in [0.1, 30] mol %");
    }

    if (in.method != "MCVD" && in.method != "OVD" && in.method != "VAD") {
        r.add("DFM-PREFORM-METHOD", "info",
              std::string(kSkillId) + ": method '" + in.method +
              "' unrecognised — expected MCVD | OVD | VAD");
    }

    if (in.core_dia_mm > 0.0 && in.clad_dia_mm > 0.0) {
        const double ratio = in.clad_dia_mm / in.core_dia_mm;
        if (ratio < 5.0) {
            r.add("DFM-PREFORM-NA", "info",
                  std::string(kSkillId) + ": clad/core ratio " +
                  std::to_string(ratio) +
                  " < 5 — multi-mode profile; verify intended NA");
        }
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "preform_deposit DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double ratio = (in.core_dia_mm > 0.0)
                       ? in.clad_dia_mm / in.core_dia_mm
                       : 0.0;

    json params = {
        { "method",                    in.method },
        { "core_dia_mm",               in.core_dia_mm },
        { "clad_dia_mm",               in.clad_dia_mm },
        { "dopant_concentration_pct",  in.dopant_concentration_pct },
        { "dopant_species",            in.dopant_species },
    };
    json pattern = {
        { "kind",                      kSkillId },
        { "method",                    in.method },
        { "core_dia_mm",               in.core_dia_mm },
        { "clad_dia_mm",               in.clad_dia_mm },
        { "dopant_concentration_pct",  in.dopant_concentration_pct },
        { "dopant_species",            in.dopant_species },
        { "clad_to_core_ratio",        ratio },
        { "geometry_changed",          false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "mcvd_lathe";
    tooling.tool_dia_mm       = in.clad_dia_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "fused_silica_substrate";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;     // additive (deposition)
    // Typical MCVD pass: 30–60 min per deposition layer; assume 10 layers.
    tooling.est_cycle_time_s  = 10.0 * 45.0 * 60.0;
    tooling.extra = {
        { "process",                   "preform_deposition" },
        { "method",                    in.method },
        { "dopant_species",            in.dopant_species },
        { "dopant_concentration_pct",  in.dopant_concentration_pct },
        { "dfm_findings",              findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::preform_deposit applied: method={} core={}mm clad={}mm dopant={}%",
                  in.method, in.core_dia_mm, in.clad_dia_mm,
                  in.dopant_concentration_pct);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// preform_deposit leaves no geometric trace at the workpiece level — the
// preform is conceptually a downstream artifact.  Recognize ONLY by replay.

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

}  // namespace koocadcam::skill::preform_deposit
