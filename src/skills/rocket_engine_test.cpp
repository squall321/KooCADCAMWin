// @lat: [[engine/skills#rocket_engine_test]]

#include "rocket_engine_test.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::rocket_engine_test {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.engine_id.empty()) {
        r.add("DFM-INPUT", "error",
              "rocket_engine_test engine_id must not be empty");
    }

    if (in.thrust_kn <= 0.0) {
        r.add("DFM-SPACE-THRUST", "error",
              "rocket_engine_test thrust_kn " + std::to_string(in.thrust_kn) +
              " must be > 0");
    } else if (in.thrust_kn > 50000.0) {
        r.add("DFM-SPACE-THRUST", "error",
              "rocket_engine_test thrust_kn " + std::to_string(in.thrust_kn) +
              " > 50000 kN — exceeds heaviest known liquid stage envelope");
    }

    if (in.burn_time_s <= 0.0) {
        r.add("DFM-SPACE-BURN", "error",
              "rocket_engine_test burn_time_s " + std::to_string(in.burn_time_s) +
              " must be > 0");
    } else if (in.burn_time_s > 1800.0) {
        r.add("DFM-SPACE-BURN", "error",
              "rocket_engine_test burn_time_s " + std::to_string(in.burn_time_s) +
              " > 1800 s — exceeds single-firing test stand budget");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "rocket_engine_test DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "engine_id",   in.engine_id },
        { "thrust_kn",   in.thrust_kn },
        { "burn_time_s", in.burn_time_s },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "engine_id",        in.engine_id },
        { "thrust_kn",        in.thrust_kn },
        { "burn_time_s",      in.burn_time_s },
        { "total_impulse_kns", in.thrust_kn * in.burn_time_s },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "test_stand";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "structural_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = in.burn_time_s + 600.0;   // burn + chill/safe
    tooling.extra = {
        { "process",     "rocket_engine_hot_fire" },
        { "engine_id",   in.engine_id },
        { "thrust_kn",   in.thrust_kn },
        { "burn_time_s", in.burn_time_s },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::rocket_engine_test applied: engine={} thrust={}kN burn={}s",
                  in.engine_id, in.thrust_kn, in.burn_time_s);

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

}  // namespace koocadcam::skill::rocket_engine_test
