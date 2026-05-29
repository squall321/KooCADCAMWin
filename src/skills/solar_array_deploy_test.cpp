// @lat: [[engine/skills#solar_array_deploy_test]]

#include "solar_array_deploy_test.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::solar_array_deploy_test {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.array_id.empty()) {
        r.add("DFM-INPUT", "error",
              "solar_array_deploy_test array_id must not be empty");
    }

    if (in.cycles_count < 1) {
        r.add("DFM-SPACE-DEPLOY", "error",
              "solar_array_deploy_test cycles_count " +
              std::to_string(in.cycles_count) +
              " < 1 — at least one functional deployment required");
    } else if (in.cycles_count > 100) {
        r.add("DFM-SPACE-DEPLOY", "error",
              "solar_array_deploy_test cycles_count " +
              std::to_string(in.cycles_count) +
              " > 100 — exceeds standard qualification envelope");
    }

    if (in.latch_torque_nm <= 0.0) {
        r.add("DFM-SPACE-LATCH", "error",
              "solar_array_deploy_test latch_torque_nm " +
              std::to_string(in.latch_torque_nm) +
              " must be > 0");
    } else if (in.latch_torque_nm > 500.0) {
        r.add("DFM-SPACE-LATCH", "error",
              "solar_array_deploy_test latch_torque_nm " +
              std::to_string(in.latch_torque_nm) +
              " > 500 N·m — far above typical hinge release torque");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "solar_array_deploy_test DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "array_id",         in.array_id },
        { "cycles_count",     in.cycles_count },
        { "latch_torque_nm",  in.latch_torque_nm },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "array_id",         in.array_id },
        { "cycles_count",     in.cycles_count },
        { "latch_torque_nm",  in.latch_torque_nm },
        { "offload_method",   "gravity_offload_balloons" },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "gravity_offload_rig";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "aluminum_extrusion";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~ 30 min deploy + restow per cycle.
    tooling.est_cycle_time_s  =
        in.cycles_count * 1800.0;
    tooling.extra = {
        { "process",         "solar_array_deploy_qual" },
        { "array_id",        in.array_id },
        { "cycles_count",    in.cycles_count },
        { "latch_torque_nm", in.latch_torque_nm },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::solar_array_deploy_test applied: array={} cycles={} torque={}Nm",
                  in.array_id, in.cycles_count, in.latch_torque_nm);

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

}  // namespace koocadcam::skill::solar_array_deploy_test
