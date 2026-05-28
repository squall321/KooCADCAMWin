// @lat: [[engine/skills#mirror_finish]]

#include "mirror_finish.hpp"

#include "Workpiece.hpp"
#include "_coating_common.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>

namespace koocadcam::skill::mirror_finish {

namespace cc = koocadcam::skill::coating_common;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.reflectivity_pct <= 0.0 || in.reflectivity_pct > 100.0) {
        r.add("DFM-MIRROR-REFL", "error",
              "mirror_finish reflectivity_pct " +
              std::to_string(in.reflectivity_pct) +
              " out of range (0, 100]");
    } else {
        if (in.reflectivity_pct < 70.0) {
            r.add("DFM-MIRROR-REFL", "info",
                  "mirror_finish reflectivity_pct " +
                  std::to_string(in.reflectivity_pct) +
                  " < 70 — this is not really a mirror; "
                  "consider lens_polish or AR coat instead");
        } else if (in.reflectivity_pct >= 99.5) {
            r.add("DFM-MIRROR-REFL", "info",
                  "mirror_finish reflectivity_pct " +
                  std::to_string(in.reflectivity_pct) +
                  " ≥ 99.5 — requires enhanced/protected metal or "
                  "dielectric high-reflector stack");
        }
    }

    if (in.wavelength_min_nm <= 0.0 || in.wavelength_max_nm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "mirror_finish wavelength bounds must be > 0");
    } else if (in.wavelength_min_nm >= in.wavelength_max_nm) {
        r.add("DFM-INPUT", "error",
              "mirror_finish wavelength_min_nm " +
              std::to_string(in.wavelength_min_nm) +
              " must be < wavelength_max_nm " +
              std::to_string(in.wavelength_max_nm));
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "mirror_finish DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("mirror_finish: entry_face datum unresolved");

    // Coating-class hint based on reflectivity target.
    const std::string coating_class =
        in.reflectivity_pct >= 99.5 ? "dielectric_high_reflector"
      : in.reflectivity_pct >= 95.0 ? "protected_metal"
                                    : "bare_metal";

    json params = {
        { "entry_face_id",       *entryId },
        { "reflectivity_pct",    in.reflectivity_pct },
        { "wavelength_min_nm",   in.wavelength_min_nm },
        { "wavelength_max_nm",   in.wavelength_max_nm },
    };
    json pattern = {
        { "kind",                "mirror_finish" },
        { "target_face_id",      *entryId },
        { "reflectivity_pct",    in.reflectivity_pct },
        { "wavelength_band_nm",  { in.wavelength_min_nm, in.wavelength_max_nm } },
        { "coating_class",       coating_class },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "thermal_evaporator";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = coating_class == "dielectric_high_reflector"
                              ? "SiO2_TiO2_HR_stack"
                              : "aluminum_silver_alloy";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = coating_class == "dielectric_high_reflector"
                              ? 3.0 * 3600.0     // long multilayer
                              : 45.0 * 60.0;     // simple metal evap
    tooling.extra = {
        { "process",            "mirror_finish" },
        { "reflectivity_pct",   in.reflectivity_pct },
        { "wavelength_min_nm",  in.wavelength_min_nm },
        { "wavelength_max_nm",  in.wavelength_max_nm },
        { "coating_class",      coating_class },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto stamped = cc::stampNoOpCoating(wp, *entryId, sig);

    spdlog::debug("skill::mirror_finish applied: face={} R={}% band=[{},{}]nm class={}",
                  *entryId, in.reflectivity_pct,
                  in.wavelength_min_nm, in.wavelength_max_nm, coating_class);

    return SkillOutput{ stamped.workpiece, stamped.signature };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    return cc::recognizeFromMetadata(wp, kSkillId);
}

}  // namespace koocadcam::skill::mirror_finish
