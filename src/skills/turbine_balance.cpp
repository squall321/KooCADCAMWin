// @lat: [[engine/skills#turbine_balance]]

#include "turbine_balance.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <set>
#include <string>

namespace koocadcam::skill::turbine_balance {

using nlohmann::json;

namespace {

const std::set<std::string>& knownGrades()
{
    static const std::set<std::string> g{
        "G0.4", "G1.0", "G2.5", "G6.3", "G16"
    };
    return g;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.rotor_id.empty()) {
        r.add("DFM-INPUT", "error",
              "turbine_balance rotor_id must be non-empty");
    }
    if (in.mass_imbalance_g_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "turbine_balance mass_imbalance_g_mm must be ≥ 0 (got " +
              std::to_string(in.mass_imbalance_g_mm) + ")");
    }
    if (knownGrades().find(in.balance_grade) == knownGrades().end()) {
        r.add("DFM-BAL-GRADE", "info",
              "turbine_balance balance_grade '" + in.balance_grade +
              "' not in ISO 21940 {G0.4,G1.0,G2.5,G6.3,G16}");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "turbine_balance DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "rotor_id",            in.rotor_id },
        { "mass_imbalance_g_mm", in.mass_imbalance_g_mm },
        { "balance_grade",       in.balance_grade },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "rotor_id",            in.rotor_id },
        { "mass_imbalance_g_mm", in.mass_imbalance_g_mm },
        { "balance_grade",       in.balance_grade },
        { "geometry_changed",    false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "balance_machine";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 300.0;          // 5 min typical balance run
    tooling.extra = {
        { "process",             "dynamic_balance" },
        { "rotor_id",            in.rotor_id },
        { "balance_grade",       in.balance_grade },
        { "mass_imbalance_g_mm", in.mass_imbalance_g_mm },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::turbine_balance applied: rotor={} grade={} U={} g·mm",
                  in.rotor_id, in.balance_grade, in.mass_imbalance_g_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata-only) ──────────────────────────────────────────

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

}  // namespace koocadcam::skill::turbine_balance
