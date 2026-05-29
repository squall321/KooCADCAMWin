// @lat: [[engine/skills#tool_measurement]]

#include "tool_measurement.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::tool_measurement {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.tool_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": tool_id must not be empty (tool traceability)");
    }
    if (in.meas_machine_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": meas_machine_id must not be empty");
    }
    if (in.dia_meas_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": dia_meas_mm must be > 0 (got " +
              std::to_string(in.dia_meas_mm) + ")");
    }
    if (in.length_meas_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": length_meas_mm must be > 0 (got " +
              std::to_string(in.length_meas_mm) + ")");
    }

    if (in.dia_meas_mm > 0.0 &&
        (in.dia_meas_mm < 0.1 || in.dia_meas_mm > 200.0)) {
        r.add("DFM-TM-DIA", "info",
              std::string(kSkillId) + ": dia_meas_mm " +
              std::to_string(in.dia_meas_mm) +
              " outside typical [0.1, 200] mm range");
    }
    if (in.length_meas_mm > 0.0 &&
        (in.length_meas_mm < 5.0 || in.length_meas_mm > 500.0)) {
        r.add("DFM-TM-LEN", "info",
              std::string(kSkillId) + ": length_meas_mm " +
              std::to_string(in.length_meas_mm) +
              " outside typical [5, 500] mm range");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "tool_measurement DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "tool_id",         in.tool_id },
        { "meas_machine_id", in.meas_machine_id },
        { "dia_meas_mm",     in.dia_meas_mm },
        { "length_meas_mm",  in.length_meas_mm },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "tool_id",           in.tool_id },
        { "meas_machine_id",   in.meas_machine_id },
        { "dia_meas_mm",       in.dia_meas_mm },
        { "length_meas_mm",    in.length_meas_mm },
        { "process",           "tool_presetting" },
        { "is_inspection_only",true },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "tool_presetter";
    tooling.tool_dia_mm       = in.dia_meas_mm;
    tooling.tool_length_mm    = in.length_meas_mm;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Optical pre-setter cycle ~ 60-180 s per tool.
    tooling.est_cycle_time_s  = 120.0;
    tooling.extra["process"]          = "tool_measurement";
    tooling.extra["tool_id"]          = in.tool_id;
    tooling.extra["meas_machine_id"]  = in.meas_machine_id;
    tooling.extra["dia_meas_mm"]      = in.dia_meas_mm;
    tooling.extra["length_meas_mm"]   = in.length_meas_mm;
    tooling.extra["dfm_findings"]     = findings;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::tool_measurement applied: tool={} machine={} D={} L={}",
                  in.tool_id, in.meas_machine_id,
                  in.dia_meas_mm, in.length_meas_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != std::string(kSkillId)) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::tool_measurement
