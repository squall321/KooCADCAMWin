// @lat: [[engine/skills#electrical_wire]]

#include "electrical_wire.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::electrical_wire {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.conductor_dia_mm <= 0.0 || in.conductor_dia_mm > 50.0) {
        r.add("DFM-WIRE-DIA", "error",
              std::string(kSkillId) + ": conductor_dia_mm " +
              std::to_string(in.conductor_dia_mm) +
              " outside (0, 50] mm — non-standard conductor size");
    }

    if (in.length_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": length_m must be > 0");
    }

    if (in.voltage_v <= 0.0 || in.voltage_v > 35000.0) {
        r.add("DFM-WIRE-VOLT", "error",
              std::string(kSkillId) + ": voltage_v " +
              std::to_string(in.voltage_v) +
              " outside (0, 35000] V — out of LV/MV range");
    }

    if (in.circuit_count < 1) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": circuit_count must be >= 1");
    }

    if (in.voltage_v > 1000.0 && in.conductor_dia_mm > 0.0 &&
        in.conductor_dia_mm < 5.0) {
        r.add("DFM-WIRE-INSUL", "info",
              std::string(kSkillId) + ": HV (" +
              std::to_string(in.voltage_v) + " V) on thin conductor " +
              std::to_string(in.conductor_dia_mm) +
              " mm — check insulation rating");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "conductor_dia_mm", in.conductor_dia_mm },
        { "length_m",         in.length_m },
        { "voltage_v",        in.voltage_v },
        { "circuit_count",    in.circuit_count },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "conductor_dia_mm", in.conductor_dia_mm },
        { "length_m",         in.length_m },
        { "voltage_v",        in.voltage_v },
        { "circuit_count",    in.circuit_count },
        { "voltage_class",
          (in.voltage_v <= 1000.0) ? std::string("LV") : std::string("MV") },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "electrician_crew";
    tooling.tool_dia_mm       = in.conductor_dia_mm;
    tooling.tool_length_mm    = in.length_m * 1000.0;
    tooling.tool_material     = "copper";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~ 8 min / m of pulling + termination, per circuit.
    tooling.est_cycle_time_s  = in.length_m * 480.0 * in.circuit_count;
    tooling.extra = {
        { "process",          "electrical_wire" },
        { "conductor_dia_mm", in.conductor_dia_mm },
        { "length_m",         in.length_m },
        { "voltage_v",        in.voltage_v },
        { "circuit_count",    in.circuit_count },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::electrical_wire applied: dia={} len={} V={} circuits={}",
                  in.conductor_dia_mm, in.length_m, in.voltage_v, in.circuit_count);

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

}  // namespace koocadcam::skill::electrical_wire
