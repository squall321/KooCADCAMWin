// @lat: [[engine/skills#launching]]

#include "launching.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::launching {

using nlohmann::json;

namespace {

bool isKnownMethod(const std::string& m)
{
    return m == "side" || m == "end" || m == "floating";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!isKnownMethod(in.launch_method)) {
        r.add("DFM-LAUNCH-METHOD", "error",
              std::string("launching: launch_method '") + in.launch_method +
              "' not in {side, end, floating}");
    }

    if (in.vessel_mass_t <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string("launching: vessel_mass_t must be > 0 (got ") +
              std::to_string(in.vessel_mass_t) + ")");
    } else if (in.launch_method == "side" && in.vessel_mass_t > 5000.0) {
        r.add("DFM-LAUNCH-SIDE", "info",
              std::string("launching: vessel_mass_t ") +
              std::to_string(in.vessel_mass_t) +
              " t > 5000 t with side launch — impractical, "
              "prefer floating launch in a graving dock");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "launching DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "launch_method", in.launch_method },
        { "vessel_mass_t", in.vessel_mass_t },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "launch_method",    in.launch_method },
        { "vessel_mass_t",    in.vessel_mass_t },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = (in.launch_method == "floating")
                                    ? "graving_dock"
                                    : "slipway";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = (in.launch_method == "floating")
                                    ? "concrete_dock"
                                    : "greased_timber_ways";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Floating launch ~ 6 h dock flood; slipway side/end ~ 30 min run.
    tooling.est_cycle_time_s  = (in.launch_method == "floating") ? 21600.0 : 1800.0;
    tooling.extra = {
        { "process",       "vessel_launch" },
        { "launch_method", in.launch_method },
        { "vessel_mass_t", in.vessel_mass_t },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::launching applied: method={} mass={}t",
                  in.launch_method, in.vessel_mass_t);

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

}  // namespace koocadcam::skill::launching
