// @lat: [[engine/skills#dichroic_coat]]

#include "dichroic_coat.hpp"

#include "Workpiece.hpp"
#include "_coating_common.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>

namespace koocadcam::skill::dichroic_coat {

namespace cc = koocadcam::skill::coating_common;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.cutoff_wavelength_nm < 200.0 || in.cutoff_wavelength_nm > 14000.0) {
        r.add("DFM-DICHROIC-CUTOFF", "error",
              "dichroic_coat cutoff_wavelength_nm " +
              std::to_string(in.cutoff_wavelength_nm) +
              " out of supported window [200, 14000] nm");
    }

    if (in.transmission_min_nm <= 0.0 ||
        in.transmission_max_nm <= 0.0 ||
        in.transmission_min_nm >= in.transmission_max_nm)
    {
        r.add("DFM-DICHROIC-TBAND", "error",
              "dichroic_coat transmission band invalid: [" +
              std::to_string(in.transmission_min_nm) + ", " +
              std::to_string(in.transmission_max_nm) + "] nm");
    }

    if (in.reflection_min_nm <= 0.0 ||
        in.reflection_max_nm <= 0.0 ||
        in.reflection_min_nm >= in.reflection_max_nm)
    {
        r.add("DFM-DICHROIC-RBAND", "error",
              "dichroic_coat reflection band invalid: [" +
              std::to_string(in.reflection_min_nm) + ", " +
              std::to_string(in.reflection_max_nm) + "] nm");
    }

    // Heuristic: cutoff should sit near one band boundary.
    if (r.passed) {
        const double t_lo = in.transmission_min_nm;
        const double t_hi = in.transmission_max_nm;
        const double r_lo = in.reflection_min_nm;
        const double r_hi = in.reflection_max_nm;
        const double c    = in.cutoff_wavelength_nm;

        const bool nearTbound = std::fabs(c - t_lo) < 30.0
                             || std::fabs(c - t_hi) < 30.0;
        const bool nearRbound = std::fabs(c - r_lo) < 30.0
                             || std::fabs(c - r_hi) < 30.0;
        if (!nearTbound && !nearRbound) {
            r.add("DFM-DICHROIC-CUTOFF-LOC", "info",
                  "dichroic_coat cutoff_wavelength_nm " + std::to_string(c) +
                  " is not within 30 nm of any transmission/reflection band edge");
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
        std::string msg = "dichroic_coat DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("dichroic_coat: entry_face datum unresolved");

    const std::string filter_kind =
        in.transmission_min_nm > in.reflection_max_nm ? "longpass"
      : in.transmission_max_nm < in.reflection_min_nm ? "shortpass"
                                                      : "bandpass";

    json params = {
        { "entry_face_id",         *entryId },
        { "cutoff_wavelength_nm",  in.cutoff_wavelength_nm },
        { "transmission_min_nm",   in.transmission_min_nm },
        { "transmission_max_nm",   in.transmission_max_nm },
        { "reflection_min_nm",     in.reflection_min_nm },
        { "reflection_max_nm",     in.reflection_max_nm },
    };
    json pattern = {
        { "kind",                  "dichroic_coat" },
        { "target_face_id",        *entryId },
        { "cutoff_wavelength_nm",  in.cutoff_wavelength_nm },
        { "transmission_band_nm",  { in.transmission_min_nm,
                                     in.transmission_max_nm } },
        { "reflection_band_nm",    { in.reflection_min_nm,
                                     in.reflection_max_nm } },
        { "filter_kind",           filter_kind },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "ion_beam_sputter";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "Ta2O5_SiO2_dichroic_stack";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Long deposition: dichroics often need 40-100 layers; estimate ~ 4 hours.
    tooling.est_cycle_time_s  = 4.0 * 3600.0;
    tooling.extra = {
        { "process",              "dichroic_coat" },
        { "cutoff_wavelength_nm", in.cutoff_wavelength_nm },
        { "filter_kind",          filter_kind },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto stamped = cc::stampNoOpCoating(wp, *entryId, sig);

    spdlog::debug("skill::dichroic_coat applied: face={} cutoff={}nm kind={}",
                  *entryId, in.cutoff_wavelength_nm, filter_kind);

    return SkillOutput{ stamped.workpiece, stamped.signature };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    return cc::recognizeFromMetadata(wp, kSkillId);
}

}  // namespace koocadcam::skill::dichroic_coat
