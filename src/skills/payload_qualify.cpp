// @lat: [[engine/skills#payload_qualify]]

#include "payload_qualify.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::payload_qualify {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.tested_mass_kg <= 0.0) {
        r.add("DFM-INPUT", "error",
              "payload_qualify tested_mass_kg must be > 0 (got " +
              std::to_string(in.tested_mass_kg) + ")");
    }

    if (in.max_speed_pct < 0.0 || in.max_speed_pct > 100.0) {
        r.add("DFM-PAYLOAD-SPEED", "error",
              "payload_qualify max_speed_pct " +
              std::to_string(in.max_speed_pct) +
              " outside [0, 100]");
    } else if (in.max_speed_pct < 25.0) {
        r.add("DFM-PAYLOAD-SPEED", "info",
              "payload_qualify max_speed_pct " +
              std::to_string(in.max_speed_pct) +
              " < 25 — heavily derated, expect long cycle times");
    }

    if (!in.brake_test_passed) {
        r.add("DFM-PAYLOAD-BRAKE", "error",
              "payload_qualify brake_test_passed=false — safety brakes did "
              "not hold rated payload, robot cannot be released to production");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "payload_qualify DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "tested_mass_kg",    in.tested_mass_kg },
        { "max_speed_pct",     in.max_speed_pct },
        { "brake_test_passed", in.brake_test_passed },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "tested_mass_kg",     in.tested_mass_kg },
        { "max_speed_pct",      in.max_speed_pct },
        { "brake_test_passed",  in.brake_test_passed },
        { "geometry_changed",   false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "payload_test_mass";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "steel_test_mass";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 300.0;   // ~5 min standard payload run
    tooling.extra = {
        { "process",            "robot_payload_qualification" },
        { "tested_mass_kg",     in.tested_mass_kg },
        { "max_speed_pct",      in.max_speed_pct },
        { "brake_test_passed",  in.brake_test_passed },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::payload_qualify applied: mass={} kg speed={}% brake={}",
                  in.tested_mass_kg, in.max_speed_pct, in.brake_test_passed);

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

}  // namespace koocadcam::skill::payload_qualify
