// @lat: [[engine/skills#fire_protection_install]]

#include "fire_protection_install.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::fire_protection_install {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.sprinkler_count < 1) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": sprinkler_count must be >= 1");
    }

    if (in.supply_pipe_dia_mm <= 0.0 || in.supply_pipe_dia_mm > 300.0) {
        r.add("DFM-FIRE-DIA", "error",
              std::string(kSkillId) + ": supply_pipe_dia_mm " +
              std::to_string(in.supply_pipe_dia_mm) +
              " outside (0, 300] mm — non-standard sprinkler supply");
    }

    if (in.coverage_m2 <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": coverage_m2 must be > 0");
    }

    if (in.sprinkler_count > 0 && in.coverage_m2 > 0.0) {
        const double per = in.coverage_m2 / in.sprinkler_count;
        if (per > 20.0) {
            r.add("DFM-FIRE-COVERAGE", "info",
                  std::string(kSkillId) + ": " + std::to_string(per) +
                  " m^2/head > 20 m^2 — exceeds NFPA-13 light-hazard limit");
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
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double coveragePerHead =
        (in.sprinkler_count > 0)
            ? (in.coverage_m2 / in.sprinkler_count) : 0.0;

    json params = {
        { "sprinkler_count",    in.sprinkler_count },
        { "supply_pipe_dia_mm", in.supply_pipe_dia_mm },
        { "coverage_m2",        in.coverage_m2 },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "sprinkler_count",       in.sprinkler_count },
        { "supply_pipe_dia_mm",    in.supply_pipe_dia_mm },
        { "coverage_m2",           in.coverage_m2 },
        { "coverage_per_head_m2",  coveragePerHead },
        { "hazard_class",
          (coveragePerHead <= 20.0)
              ? std::string("light_hazard_nfpa13_ok")
              : std::string("over_limit") },
        { "geometry_changed",      false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "fire_protection_crew";
    tooling.tool_dia_mm       = in.supply_pipe_dia_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "steel_sched40";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~ 45 min / head install + pipe routing overhead.
    tooling.est_cycle_time_s  = in.sprinkler_count * 2700.0 + 3600.0;
    tooling.extra = {
        { "process",               "fire_protection_install" },
        { "sprinkler_count",       in.sprinkler_count },
        { "supply_pipe_dia_mm",    in.supply_pipe_dia_mm },
        { "coverage_m2",           in.coverage_m2 },
        { "coverage_per_head_m2",  coveragePerHead },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::fire_protection_install applied: heads={} pipe={} area={} m2",
                  in.sprinkler_count, in.supply_pipe_dia_mm, in.coverage_m2);

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

}  // namespace koocadcam::skill::fire_protection_install
