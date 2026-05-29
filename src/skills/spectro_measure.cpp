// @lat: [[engine/skills#spectro_measure]]

#include "spectro_measure.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::spectro_measure {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownInstruments()
{
    static const std::unordered_set<std::string> s = {
        "UV-Vis", "IR", "Raman", "MS",
    };
    return s;
}

struct WlBand { double lo_nm; double hi_nm; };

WlBand bandFor(const std::string& inst)
{
    if (inst == "UV-Vis") return {  190.0,    800.0 };
    if (inst == "IR")     return { 2500.0,  25000.0 };
    if (inst == "Raman")  return {  200.0,   1064.0 };  // excitation line
    return { 0.0, 0.0 };  // MS / unknown — no wavelength band
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.instrument_type.empty() ||
        knownInstruments().count(in.instrument_type) == 0)
    {
        r.add("DFM-INPUT", "error",
              "spectro_measure unknown instrument_type '" +
              in.instrument_type + "' (expected UV-Vis | IR | Raman | MS)");
    }

    if (in.scan_count < 1) {
        r.add("DFM-SPECTRO-SCAN", "error",
              "spectro_measure scan_count must be >= 1 (got " +
              std::to_string(in.scan_count) + ")");
    }

    // Wavelength is only meaningful for optical modalities.
    const bool optical =
        in.instrument_type == "UV-Vis" ||
        in.instrument_type == "IR"     ||
        in.instrument_type == "Raman";

    if (optical) {
        if (in.wavelength_nm <= 0.0) {
            r.add("DFM-SPECTRO-WL", "error",
                  "spectro_measure wavelength_nm must be > 0 for " +
                  in.instrument_type + " (got " +
                  std::to_string(in.wavelength_nm) + ")");
        } else {
            const auto band = bandFor(in.instrument_type);
            if (in.wavelength_nm < band.lo_nm ||
                in.wavelength_nm > band.hi_nm)
            {
                r.add("DFM-SPECTRO-WL", "info",
                      "spectro_measure wavelength_nm " +
                      std::to_string(in.wavelength_nm) +
                      " nm outside typical band [" +
                      std::to_string(band.lo_nm) + ", " +
                      std::to_string(band.hi_nm) + "] for " +
                      in.instrument_type);
            }
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
        std::string msg = "spectro_measure DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "instrument_type", in.instrument_type },
        { "wavelength_nm",   in.wavelength_nm },
        { "scan_count",      in.scan_count },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "instrument_type",  in.instrument_type },
        { "wavelength_nm",    in.wavelength_nm },
        { "scan_count",       in.scan_count },
        { "process_family",   "analytical_spectroscopy" },
        { "geometry_changed", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    if      (in.instrument_type == "UV-Vis") tooling.tool_type = "uvvis_spectrophotometer";
    else if (in.instrument_type == "IR")     tooling.tool_type = "ftir_spectrometer";
    else if (in.instrument_type == "Raman")  tooling.tool_type = "raman_spectrometer";
    else if (in.instrument_type == "MS")     tooling.tool_type = "mass_spectrometer";
    else                                      tooling.tool_type = "spectrometer";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = (in.instrument_type == "IR") ? "kbr_window"
                                                              : "quartz_cuvette";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ≈ 1 s per scan + 5 s instrument overhead.
    tooling.est_cycle_time_s  = 5.0 + static_cast<double>(in.scan_count);
    tooling.extra = {
        { "process",          "spectroscopic_measurement" },
        { "instrument_type",  in.instrument_type },
        { "wavelength_nm",    in.wavelength_nm },
        { "scan_count",       in.scan_count },
        { "process_category", "laboratory" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::spectro_measure applied: inst={} λ={}nm scans={}",
                  in.instrument_type, in.wavelength_nm, in.scan_count);

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

}  // namespace koocadcam::skill::spectro_measure
