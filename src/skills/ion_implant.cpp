// @lat: [[engine/skills#ion_implant]]

#include "ion_implant.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::ion_implant {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.dopant.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": dopant must be non-empty");
    }

    if (in.energy_kev <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": energy_kev must be > 0 (got " +
              std::to_string(in.energy_kev) + ")");
    } else if (in.energy_kev < 0.5) {
        r.add("DFM-IMPLANT-ENERGY-LOW", "info",
              std::string(kSkillId) + ": energy_kev " +
              std::to_string(in.energy_kev) +
              " < 0.5 keV — ultra-low-energy, verify implanter capability");
    } else if (in.energy_kev > 500.0) {
        r.add("DFM-IMPLANT-ENERGY-HIGH", "info",
              std::string(kSkillId) + ": energy_kev " +
              std::to_string(in.energy_kev) +
              " > 500 keV — typical production limit");
    }

    if (in.dose_atoms_cm2 <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": dose_atoms_cm2 must be > 0 (got " +
              std::to_string(in.dose_atoms_cm2) + ")");
    }

    if (in.tilt_deg < 0.0 || in.tilt_deg > 60.0) {
        r.add("DFM-IMPLANT-TILT", "info",
              std::string(kSkillId) + ": tilt_deg " +
              std::to_string(in.tilt_deg) +
              " outside [0, 60] — extreme tilts shadow features");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "ion_implant DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "dopant",         in.dopant },
        { "energy_kev",     in.energy_kev },
        { "dose_atoms_cm2", in.dose_atoms_cm2 },
        { "tilt_deg",       in.tilt_deg },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "dopant",           in.dopant },
        { "energy_kev",       in.energy_kev },
        { "dose_atoms_cm2",   in.dose_atoms_cm2 },
        { "tilt_deg",         in.tilt_deg },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "ion_implanter";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "beamline_optics";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Cycle scales with dose: ~ 1 s per 1e13 atoms/cm² + 60 s setup.
    tooling.est_cycle_time_s  = 60.0 + (in.dose_atoms_cm2 / 1.0e13) * 1.0;
    tooling.extra = {
        { "process",          "ion_implantation" },
        { "dopant",           in.dopant },
        { "energy_kev",       in.energy_kev },
        { "dose_atoms_cm2",   in.dose_atoms_cm2 },
        { "tilt_deg",         in.tilt_deg },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::ion_implant applied: dopant={} E={}keV dose={} tilt={}°",
                  in.dopant, in.energy_kev, in.dose_atoms_cm2, in.tilt_deg);

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

}  // namespace koocadcam::skill::ion_implant
