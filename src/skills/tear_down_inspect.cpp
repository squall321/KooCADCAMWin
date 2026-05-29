// @lat: [[engine/skills#tear_down_inspect]]

#include "tear_down_inspect.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::tear_down_inspect {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.steps_taken <= 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": steps_taken must be > 0 (got " +
              std::to_string(in.steps_taken) + ")");
    }
    if (in.time_min <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": time_min must be > 0 (got " +
              std::to_string(in.time_min) + ")");
    }
    if (in.parts_marked < 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": parts_marked must be >= 0 (got " +
              std::to_string(in.parts_marked) + ")");
    }

    if (in.steps_taken == 1 && in.parts_marked > 0) {
        r.add("DFM-TDI-CONSIST", "info",
              std::string(kSkillId) + ": parts_marked=" +
              std::to_string(in.parts_marked) +
              " after a single-step teardown — verify count");
    }
    if (in.time_min > 480.0) {
        r.add("DFM-TDI-LONG", "info",
              std::string(kSkillId) + ": time_min " +
              std::to_string(in.time_min) +
              " > 480 (>8 h teardown is unusual)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "tear_down_inspect DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const bool inspectionPass = (in.parts_marked == 0);

    json params = {
        { "steps_taken",  in.steps_taken },
        { "time_min",     in.time_min },
        { "parts_marked", in.parts_marked },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "steps_taken",      in.steps_taken },
        { "time_min",         in.time_min },
        { "parts_marked",     in.parts_marked },
        { "inspection_pass",  inspectionPass },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "inspection_bench";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = in.time_min * 60.0;
    tooling.extra = {
        { "process",          "tear_down_inspect" },
        { "process_category", "maintenance" },
        { "geometric_no_op",  true },
        { "steps_taken",      in.steps_taken },
        { "parts_marked",     in.parts_marked },
        { "inspection_pass",  inspectionPass },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::tear_down_inspect applied: steps={} time={}min marked={}",
        in.steps_taken, in.time_min, in.parts_marked);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Disassembly leaves no B-rep trace.  Metadata replay only.

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

}  // namespace koocadcam::skill::tear_down_inspect
