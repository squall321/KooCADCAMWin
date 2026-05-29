// @lat: [[engine/skills#vacuum_leak_seal]]

#include "vacuum_leak_seal.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::vacuum_leak_seal {

using nlohmann::json;

namespace {

bool isKnownMethod(const std::string& m)
{
    return m == "torr_seal_epoxy" || m == "re_torque" ||
           m == "braze_patch"     || m == "remake";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.leak_rate_pa_m3_s <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              " leak_rate_pa_m3_s must be > 0 (got " +
              std::to_string(in.leak_rate_pa_m3_s) + ")");
    } else if (in.leak_rate_pa_m3_s > 1.0e-7) {
        r.add("DFM-LEAK-GROSS", "error",
              std::string(kSkillId) +
              " leak_rate_pa_m3_s " + std::to_string(in.leak_rate_pa_m3_s) +
              " > 1e-7 Pa m^3/s — gross leak; cannot seal in place, joint must be remade");
    }

    if (!isKnownMethod(in.seal_method)) {
        r.add("DFM-LEAK-METHOD", "info",
              std::string(kSkillId) + " seal_method '" + in.seal_method +
              "' not in {torr_seal_epoxy, re_torque, braze_patch, remake}");
    }

    if (in.ultimate_pressure_torr <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              " ultimate_pressure_torr must be > 0");
    } else if (in.ultimate_pressure_torr > 1.0e-6) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              " ultimate_pressure_torr " +
              std::to_string(in.ultimate_pressure_torr) +
              " > 1e-6 torr — not HV/UHV class, seal ineffective");
    } else if (in.ultimate_pressure_torr > 1.0e-9) {
        r.add("DFM-LEAK-UHV", "info",
              std::string(kSkillId) +
              " ultimate_pressure_torr " +
              std::to_string(in.ultimate_pressure_torr) +
              " > 1e-9 torr — high vacuum only, not UHV");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "vacuum_leak_seal DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string regime =
        in.ultimate_pressure_torr <= 1.0e-11 ? "XHV" :
        in.ultimate_pressure_torr <= 1.0e-9  ? "UHV" :
        in.ultimate_pressure_torr <= 1.0e-6  ? "HV"  :
                                               "MV";

    json params = {
        { "leak_rate_pa_m3_s",      in.leak_rate_pa_m3_s },
        { "seal_method",            in.seal_method },
        { "ultimate_pressure_torr", in.ultimate_pressure_torr },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "leak_rate_pa_m3_s",      in.leak_rate_pa_m3_s },
        { "seal_method",            in.seal_method },
        { "ultimate_pressure_torr", in.ultimate_pressure_torr },
        { "vacuum_regime",          regime },
        { "geometry_changed",       false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "helium_leak_detector";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = (in.seal_method == "torr_seal_epoxy")
                                    ? "epoxy_torr_seal"
                                : (in.seal_method == "braze_patch")
                                    ? "BAg-8_filler"
                                    : "stainless_steel_316L";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Seal + cure + re-pump + leak re-test ≈ 1 h typical.
    tooling.est_cycle_time_s  = 3600.0;
    tooling.extra = {
        { "process",                "vacuum_leak_seal" },
        { "seal_method",            in.seal_method },
        { "leak_rate_pa_m3_s",      in.leak_rate_pa_m3_s },
        { "ultimate_pressure_torr", in.ultimate_pressure_torr },
        { "vacuum_regime",          regime },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::vacuum_leak_seal applied: leak={} method={} ult={}torr regime={}",
                  in.leak_rate_pa_m3_s, in.seal_method,
                  in.ultimate_pressure_torr, regime);

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

}  // namespace koocadcam::skill::vacuum_leak_seal
