// @lat: [[engine/skills#erection_lift]]

#include "erection_lift.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::erection_lift {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.lift_id.empty()) {
        r.add("DFM-INPUT", "warning",
              "erection_lift: lift_id empty — defaulting to 'LFT-UNK'");
    }

    if (in.mass_t <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string("erection_lift: mass_t must be > 0 (got ") +
              std::to_string(in.mass_t) + ")");
    }
    if (in.lift_height_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string("erection_lift: lift_height_m must be > 0 (got ") +
              std::to_string(in.lift_height_m) + ")");
    }
    if (in.crane_capacity_t <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string("erection_lift: crane_capacity_t must be > 0 (got ") +
              std::to_string(in.crane_capacity_t) + ")");
    }

    if (in.mass_t > 0.0 && in.crane_capacity_t > 0.0) {
        if (in.mass_t > in.crane_capacity_t) {
            r.add("DFM-LIFT-CAPACITY", "error",
                  std::string("erection_lift: mass_t ") +
                  std::to_string(in.mass_t) +
                  " exceeds crane_capacity_t " +
                  std::to_string(in.crane_capacity_t) + " — overloaded");
        } else {
            const double util = in.mass_t / in.crane_capacity_t;
            if (util > 0.85) {
                r.add("DFM-LIFT-UTIL", "info",
                      std::string("erection_lift: utilization ") +
                      std::to_string(util) +
                      " > 0.85 — within capacity but at safety margin");
            }
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
        std::string msg = "erection_lift DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string lift_id = in.lift_id.empty() ? "LFT-UNK" : in.lift_id;
    const double util         = in.crane_capacity_t > 0.0
                                    ? in.mass_t / in.crane_capacity_t : 0.0;

    json params = {
        { "lift_id",          lift_id },
        { "mass_t",           in.mass_t },
        { "lift_height_m",    in.lift_height_m },
        { "crane_capacity_t", in.crane_capacity_t },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "lift_id",           lift_id },
        { "mass_t",            in.mass_t },
        { "lift_height_m",     in.lift_height_m },
        { "crane_capacity_t",  in.crane_capacity_t },
        { "utilization",       util },
        { "geometry_changed",  false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "goliath_crane";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "structural_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Typical hoist speed ~ 6 m/min loaded; round-trip + rigging adds ~ 30 min.
    tooling.est_cycle_time_s  = 1800.0 + (in.lift_height_m / 6.0) * 60.0;
    tooling.extra = {
        { "process",         "block_erection_hoist" },
        { "lift_id",         lift_id },
        { "mass_t",          in.mass_t },
        { "lift_height_m",   in.lift_height_m },
        { "crane_capacity_t",in.crane_capacity_t },
        { "utilization",     util },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::erection_lift applied: id={} mass={}t height={}m cap={}t util={}",
                  lift_id, in.mass_t, in.lift_height_m, in.crane_capacity_t, util);

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

}  // namespace koocadcam::skill::erection_lift
