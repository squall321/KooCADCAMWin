// @lat: [[engine/skills#antireflective_coat]]

#include "antireflective_coat.hpp"

#include "Workpiece.hpp"
#include "_coating_common.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>

namespace koocadcam::skill::antireflective_coat {

namespace cc = koocadcam::skill::coating_common;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.wavelength_min_nm <= 0.0 || in.wavelength_max_nm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "antireflective_coat wavelength bounds must be > 0");
    } else if (in.wavelength_min_nm >= in.wavelength_max_nm) {
        r.add("DFM-INPUT", "error",
              "antireflective_coat wavelength_min_nm " +
              std::to_string(in.wavelength_min_nm) +
              " must be < wavelength_max_nm " +
              std::to_string(in.wavelength_max_nm));
    } else {
        if (in.wavelength_min_nm < 200.0 || in.wavelength_max_nm > 14000.0) {
            r.add("DFM-AR-BAND", "info",
                  "antireflective_coat wavelength band [" +
                  std::to_string(in.wavelength_min_nm) + ", " +
                  std::to_string(in.wavelength_max_nm) +
                  "] nm outside common UV-MWIR window [200, 14000] nm");
        }
    }

    if (in.reflectance_pct <= 0.0 || in.reflectance_pct > 10.0) {
        r.add("DFM-AR-REFL", "error",
              "antireflective_coat reflectance_pct " +
              std::to_string(in.reflectance_pct) +
              " out of practical range (0, 10] %");
    }

    if (in.layer_count < 1) {
        r.add("DFM-AR-LAYERS", "error",
              "antireflective_coat layer_count must be ≥ 1 (got " +
              std::to_string(in.layer_count) + ")");
    } else if (in.layer_count > 30) {
        r.add("DFM-AR-LAYERS", "info",
              "antireflective_coat layer_count " +
              std::to_string(in.layer_count) +
              " > 30 — exotic broadband stack, verify deposition budget");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "antireflective_coat DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId)
        throw SkillError("antireflective_coat: entry_face datum unresolved");

    const std::string ar_class =
        in.layer_count <= 2  ? "single_v_coat"
      : in.layer_count <= 7  ? "broadband_ar"
                             : "super_broadband_ar";

    json params = {
        { "entry_face_id",    *entryId },
        { "wavelength_min_nm", in.wavelength_min_nm },
        { "wavelength_max_nm", in.wavelength_max_nm },
        { "reflectance_pct",   in.reflectance_pct },
        { "layer_count",       in.layer_count },
    };
    json pattern = {
        { "kind",              "antireflective_coat" },
        { "target_face_id",    *entryId },
        { "wavelength_band_nm", { in.wavelength_min_nm, in.wavelength_max_nm } },
        { "reflectance_pct",   in.reflectance_pct },
        { "layer_count",       in.layer_count },
        { "ar_class",          ar_class },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "vacuum_evaporator";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "MgF2_SiO2_TiO2_stack";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~ 5 min per deposited layer, plus 30 min pumpdown.
    tooling.est_cycle_time_s  = (30.0 + 5.0 * in.layer_count) * 60.0;
    tooling.extra = {
        { "process",           "antireflective_coat" },
        { "wavelength_min_nm", in.wavelength_min_nm },
        { "wavelength_max_nm", in.wavelength_max_nm },
        { "reflectance_pct",   in.reflectance_pct },
        { "layer_count",       in.layer_count },
        { "ar_class",          ar_class },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto stamped = cc::stampNoOpCoating(wp, *entryId, sig);

    spdlog::debug("skill::antireflective_coat applied: face={} band=[{},{}]nm R={}% layers={}",
                  *entryId, in.wavelength_min_nm, in.wavelength_max_nm,
                  in.reflectance_pct, in.layer_count);

    return SkillOutput{ stamped.workpiece, stamped.signature };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    return cc::recognizeFromMetadata(wp, kSkillId);
}

}  // namespace koocadcam::skill::antireflective_coat
