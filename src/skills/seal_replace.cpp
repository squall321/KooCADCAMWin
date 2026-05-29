// @lat: [[engine/skills#seal_replace]]

#include "seal_replace.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::seal_replace {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.seal_type.empty()) {
        r.add("DFM-INPUT", "error",
              "seal_replace seal_type must be non-empty");
    }
    if (in.seal_id.empty()) {
        r.add("DFM-INPUT", "error",
              "seal_replace seal_id must be non-empty");
    }

    if (!in.leak_test_passed) {
        r.add("DFM-SEAL-LEAK", "error",
              "seal_replace leak_test_passed=false — seal failed post-install "
              "leak test, do not close work order");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "seal_replace DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "seal_type",        in.seal_type },
        { "seal_id",          in.seal_id },
        { "leak_test_passed", in.leak_test_passed },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "seal_type",         in.seal_type },
        { "seal_id",           in.seal_id },
        { "leak_test_passed",  in.leak_test_passed },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "seal_pick_and_press";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "polymer";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 900.0;     // ~15 min seal swap + leak test
    tooling.extra = {
        { "process",           "seal_swap" },
        { "seal_type",         in.seal_type },
        { "seal_id",           in.seal_id },
        { "leak_test_passed",  in.leak_test_passed },
        { "dfm_findings",      findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::seal_replace applied: type={} id={} leak_pass={}",
                  in.seal_type, in.seal_id, in.leak_test_passed);

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

}  // namespace koocadcam::skill::seal_replace
