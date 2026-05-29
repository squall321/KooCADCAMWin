// @lat: [[engine/skills#cell_seal]]

#include "cell_seal.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::cell_seal {

using nlohmann::json;

namespace {

bool isKnownSealMethod(const std::string& m)
{
    return m == "weld" || m == "crimp" || m == "heat";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!isKnownSealMethod(in.seal_method)) {
        r.add("DFM-SEAL-METHOD", "error",
              "cell_seal: seal_method '" + in.seal_method +
              "' must be 'weld', 'crimp', or 'heat'");
    }

    if (in.leak_pa_m3_s < 0.0) {
        r.add("DFM-INPUT", "error",
              "cell_seal: leak_pa_m3_s must be >= 0 (got " +
              std::to_string(in.leak_pa_m3_s) + ")");
    } else {
        if (in.leak_pa_m3_s > 1.0e-8) {
            r.add("DFM-SEAL-LEAK", "error",
                  "cell_seal: leak_pa_m3_s " +
                  std::to_string(in.leak_pa_m3_s) +
                  " > 1e-8 — fails hermetic seal acceptance");
        } else if (in.leak_pa_m3_s > 1.0e-9) {
            r.add("DFM-SEAL-LEAK-WARN", "info",
                  "cell_seal: leak_pa_m3_s " +
                  std::to_string(in.leak_pa_m3_s) +
                  " > 1e-9 — marginal hermeticity, monitor field returns");
        }
    }

    if (in.seal_method == "weld") {
        if (in.weld_energy_j < 1.0) {
            r.add("DFM-SEAL-WELD-ENERGY", "info",
                  "cell_seal: weld_energy_j " +
                  std::to_string(in.weld_energy_j) +
                  " < 1 J — likely under-fused weld");
        }
    } else if (in.seal_method == "heat") {
        if (in.seal_temp_c < 120.0 || in.seal_temp_c > 200.0) {
            r.add("DFM-SEAL-HEAT-TEMP", "info",
                  "cell_seal: seal_temp_c " +
                  std::to_string(in.seal_temp_c) +
                  " outside typical [120, 200] °C heat-seal window");
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
        std::string msg = "cell_seal DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "seal_method",   in.seal_method },
        { "leak_pa_m3_s",  in.leak_pa_m3_s },
        { "weld_energy_j", in.weld_energy_j },
        { "seal_temp_c",   in.seal_temp_c },
    };
    json pattern = {
        { "kind",          kSkillId },
        { "seal_method",   in.seal_method },
        { "leak_pa_m3_s",  in.leak_pa_m3_s },
        { "weld_energy_j", in.seal_method == "weld" ? in.weld_energy_j : 0.0 },
        { "seal_temp_c",   in.seal_method == "heat" ? in.seal_temp_c   : 0.0 },
        { "hermetic_pass", in.leak_pa_m3_s <= 1.0e-8 },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    if (in.seal_method == "weld") {
        tooling.tool_type      = "laser_seam_welder";
        tooling.tool_dia_mm    = 0.4;       // typical spot diameter
        tooling.tool_material  = "fibre_laser_head";
        tooling.est_cycle_time_s = 10.0;
    } else if (in.seal_method == "crimp") {
        tooling.tool_type      = "crimp_die";
        tooling.tool_dia_mm    = 18.0;      // typical 18650 die
        tooling.tool_material  = "hardened_tool_steel";
        tooling.est_cycle_time_s = 4.0;
    } else {
        tooling.tool_type      = "heat_seal_jaw";
        tooling.tool_dia_mm    = 0.0;
        tooling.tool_material  = "PTFE_coated_aluminum";
        tooling.est_cycle_time_s = 6.0;
    }
    tooling.tool_length_mm = 0.0;
    tooling.extra = {
        { "process",          "cell_closure" },
        { "seal_method",      in.seal_method },
        { "leak_pa_m3_s",     in.leak_pa_m3_s },
        { "process_category", "battery_manufacturing" },
        { "geometric_no_op",  true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::cell_seal applied: method={} leak={}Pa·m3/s",
        in.seal_method, in.leak_pa_m3_s);

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

}  // namespace koocadcam::skill::cell_seal
