// @lat: [[engine/skills#disassemble_step]]

#include "disassemble_step.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::disassemble_step {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.step_count <= 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": step_count must be > 0 (got " +
              std::to_string(in.step_count) + ")");
    }
    if (in.fasteners_removed < 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": fasteners_removed must be >= 0 (got " +
              std::to_string(in.fasteners_removed) + ")");
    }
    if (in.time_min <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": time_min must be > 0 (got " +
              std::to_string(in.time_min) + ")");
    }

    if (in.time_min >= 120.0) {
        r.add("DFM-DISASM-TIME", "warning",
              std::string(kSkillId) + ": time_min " +
              std::to_string(in.time_min) +
              " >= 120 min — disassembly cost may exceed recovered scrap value");
    }
    if (in.step_count > 50) {
        r.add("DFM-DISASM-COMPLEX", "info",
              std::string(kSkillId) + ": step_count " +
              std::to_string(in.step_count) +
              " > 50 — consider design-for-disassembly (DfD) review");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "disassemble_step DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "step_count",          in.step_count },
        { "fasteners_removed",   in.fasteners_removed },
        { "time_min",            in.time_min },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "step_count",          in.step_count },
        { "fasteners_removed",   in.fasteners_removed },
        { "time_min",            in.time_min },
        { "process_category",    "eol_disassembly" },
        { "geometry_changed",    false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "hand_tools";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = in.time_min * 60.0;
    tooling.extra = {
        { "process",            "disassembly" },
        { "step_count",         in.step_count },
        { "fasteners_removed",  in.fasteners_removed },
        { "time_min",           in.time_min },
        { "process_category",   "eol_disassembly" },
        { "geometric_no_op",    true },
        { "dfm_findings",       findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::disassemble_step applied: steps={} fasteners={} time={}min",
                  in.step_count, in.fasteners_removed, in.time_min);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Disassembly leaves no geometric trace on the held workpiece — we can ONLY
// recognize a prior disassemble_step by replaying signatures previously
// stamped onto the workpiece.

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

}  // namespace koocadcam::skill::disassemble_step
