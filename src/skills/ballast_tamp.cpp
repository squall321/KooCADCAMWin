// @lat: [[engine/skills#ballast_tamp]]

#include "ballast_tamp.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::ballast_tamp {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.lift_mm <= 0.0) {
        r.add("DFM-BT-LIFT", "error",
              std::string(kSkillId) + ": lift_mm must be > 0 (got " +
              std::to_string(in.lift_mm) + ")");
    } else if (in.lift_mm > 60.0) {
        r.add("DFM-BT-LIFT", "error",
              std::string(kSkillId) + ": lift_mm " +
              std::to_string(in.lift_mm) +
              " > 60 mm — split into multiple stages or use stoneblower");
    }

    if (in.line_class != "freight" &&
        in.line_class != "main" &&
        in.line_class != "high_speed") {
        r.add("DFM-BT-CLASS", "info",
              std::string(kSkillId) + ": line_class '" + in.line_class +
              "' unrecognised — expected freight/main/high_speed");
    }

    if (in.passes_count < 1) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": passes_count must be >= 1 (got " +
              std::to_string(in.passes_count) + ")");
    } else if (in.line_class == "high_speed" && in.passes_count < 2) {
        r.add("DFM-BT-PASS", "info",
              std::string(kSkillId) +
              ": high-speed line typically requires >= 2 tamping passes");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "ballast_tamp DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "lift_mm",      in.lift_mm },
        { "line_class",   in.line_class },
        { "passes_count", in.passes_count },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "lift_mm",          in.lift_mm },
        { "line_class",       in.line_class },
        { "passes_count",     in.passes_count },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "tamping_machine";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = in.lift_mm;
    tooling.tool_material     = "vibratory_tine";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Single sleeper tamp ~ 6 s per pass.
    tooling.est_cycle_time_s  = 6.0 * static_cast<double>(in.passes_count);
    tooling.extra = {
        { "process",      "ballast_tamp" },
        { "lift_mm",      in.lift_mm },
        { "line_class",   in.line_class },
        { "passes_count", in.passes_count },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::ballast_tamp applied: lift={} class={} passes={}",
                  in.lift_mm, in.line_class, in.passes_count);

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

}  // namespace koocadcam::skill::ballast_tamp
