// @lat: [[engine/skills#cycle_optimize]]

#include "cycle_optimize.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::cycle_optimize {

using nlohmann::json;

namespace {
double savingsPct(double orig, double opt)
{
    if (orig <= 0.0) return 0.0;
    return (orig - opt) / orig * 100.0;
}
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.original_cycle_s <= 0.0) {
        r.add("DFM-INPUT", "error",
              "cycle_optimize original_cycle_s must be > 0 (got " +
              std::to_string(in.original_cycle_s) + ")");
    }
    if (in.optimized_cycle_s <= 0.0) {
        r.add("DFM-INPUT", "error",
              "cycle_optimize optimized_cycle_s must be > 0 (got " +
              std::to_string(in.optimized_cycle_s) + ")");
    }

    if (in.original_cycle_s > 0.0 && in.optimized_cycle_s > 0.0 &&
        in.optimized_cycle_s > in.original_cycle_s) {
        r.add("DFM-CYCLE-REGRESSION", "error",
              "cycle_optimize optimized_cycle_s " +
              std::to_string(in.optimized_cycle_s) +
              " > original_cycle_s " + std::to_string(in.original_cycle_s) +
              " — optimization regressed cycle time");
    }

    if (in.method.empty()) {
        r.add("DFM-INPUT", "warning",
              "cycle_optimize method empty — record at least the technique used");
    }

    if (in.original_cycle_s > 0.0 && in.optimized_cycle_s > 0.0 &&
        in.optimized_cycle_s <= in.original_cycle_s) {
        const double sp = savingsPct(in.original_cycle_s, in.optimized_cycle_s);
        if (sp < 5.0) {
            r.add("DFM-CYCLE-MARGINAL", "info",
                  "cycle_optimize savings_pct " + std::to_string(sp) +
                  " < 5% — marginal gain, weigh against validation overhead");
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
        std::string msg = "cycle_optimize DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string method = in.method.empty() ? "unspecified" : in.method;
    const double sp = savingsPct(in.original_cycle_s, in.optimized_cycle_s);

    json params = {
        { "original_cycle_s",  in.original_cycle_s },
        { "optimized_cycle_s", in.optimized_cycle_s },
        { "method",            method },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "original_cycle_s",  in.original_cycle_s },
        { "optimized_cycle_s", in.optimized_cycle_s },
        { "method",            method },
        { "savings_pct",       sp },
        { "geometry_changed",  false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "cycle_time_analyzer";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = in.optimized_cycle_s;
    tooling.extra = {
        { "process",           "robot_cycle_optimization" },
        { "method",            method },
        { "original_cycle_s",  in.original_cycle_s },
        { "optimized_cycle_s", in.optimized_cycle_s },
        { "savings_pct",       sp },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::cycle_optimize applied: orig={}s opt={}s method={} ({}% saved)",
                  in.original_cycle_s, in.optimized_cycle_s, method, sp);

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

}  // namespace koocadcam::skill::cycle_optimize
