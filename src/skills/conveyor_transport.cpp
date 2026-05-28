// @lat: [[engine/skills#conveyor_transport]]

#include "conveyor_transport.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::conveyor_transport {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.start_station.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": start_station must not be empty");
    }
    if (in.end_station.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": end_station must not be empty");
    }
    if (!in.start_station.empty() && !in.end_station.empty() &&
        in.start_station == in.end_station)
    {
        r.add("DFM-CONVEYOR-DEGENERATE", "warning",
              std::string(kSkillId) + ": start_station == end_station ('" +
              in.start_station + "') — degenerate transport");
    }

    if (in.speed_m_min <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": speed_m_min must be > 0 (got " +
              std::to_string(in.speed_m_min) + ")");
    } else if (in.speed_m_min > 60.0) {
        r.add("DFM-CONVEYOR-SPEED", "info",
              std::string(kSkillId) + ": speed_m_min " +
              std::to_string(in.speed_m_min) +
              " exceeds typical industrial belt rate (60 m/min)");
    }

    if (in.length_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": length_m must be > 0 (got " +
              std::to_string(in.length_m) + ")");
    } else if (in.length_m > 200.0) {
        r.add("DFM-CONVEYOR-LENGTH", "info",
              std::string(kSkillId) + ": length_m " +
              std::to_string(in.length_m) +
              " exceeds typical cell-scale conveyor (200 m)");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "conveyor_transport DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Transit time = (length_m / speed_m_min) * 60.  speed > 0 guaranteed.
    const double transit_s = (in.length_m / in.speed_m_min) * 60.0;

    json params = {
        { "start_station", in.start_station },
        { "end_station",   in.end_station },
        { "speed_m_min",   in.speed_m_min },
        { "length_m",      in.length_m },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "start_station",    in.start_station },
        { "end_station",      in.end_station },
        { "speed_m_min",      in.speed_m_min },
        { "length_m",         in.length_m },
        { "transit_time_s",   transit_s },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "conveyor_belt";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = in.length_m * 1000.0;
    tooling.tool_material     = "polymer_belt";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = transit_s;
    tooling.extra = {
        { "process",          "conveyor_transport" },
        { "start_station",    in.start_station },
        { "end_station",      in.end_station },
        { "speed_m_min",      in.speed_m_min },
        { "length_m",         in.length_m },
        { "process_category", "material_handling" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::conveyor_transport applied: {} → {} @ {}m/min over {}m ({}s)",
        in.start_station, in.end_station,
        in.speed_m_min, in.length_m, transit_s);

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

}  // namespace koocadcam::skill::conveyor_transport
