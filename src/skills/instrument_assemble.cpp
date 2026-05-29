// @lat: [[engine/skills#instrument_assemble]]

#include "instrument_assemble.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::instrument_assemble {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.instrument_type.empty()) {
        r.add("DFM-INPUT", "error",
              "instrument_assemble instrument_type must be non-empty");
    }

    if (in.component_count < 2) {
        r.add("DFM-ASM-COUNT", "error",
              "instrument_assemble component_count " +
              std::to_string(in.component_count) +
              " < 2 — assembly requires at least two components");
    } else if (in.component_count > 500) {
        r.add("DFM-ASM-COUNT", "info",
              "instrument_assemble component_count " +
              std::to_string(in.component_count) +
              " > 500 — unusually high, verify BOM");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "instrument_assemble DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "instrument_type", in.instrument_type },
        { "component_count", in.component_count },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "instrument_type",  in.instrument_type },
        { "component_count",  in.component_count },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "assembly_bench";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "none";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Rough estimate: 20 s per component for hand assembly.
    tooling.est_cycle_time_s  = 20.0 * static_cast<double>(in.component_count);
    tooling.extra = {
        { "process",         "instrument_assembly" },
        { "instrument_type", in.instrument_type },
        { "component_count", in.component_count },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    // No geometric change — clone shape + material verbatim.
    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::instrument_assemble applied: type={} components={}",
                  in.instrument_type, in.component_count);

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

}  // namespace koocadcam::skill::instrument_assemble
