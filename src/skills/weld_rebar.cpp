// @lat: [[engine/skills#weld_rebar]]

#include "weld_rebar.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::weld_rebar {

using nlohmann::json;

namespace {

bool isKnownWeldType(const std::string& t)
{
    return t == "tack" || t == "full" || t == "lap" || t == "butt";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.joint_count < 1) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": joint_count must be ≥ 1 (got " +
              std::to_string(in.joint_count) + ")");
    }

    if (!isKnownWeldType(in.weld_type)) {
        r.add("DFM-WELD-TYPE", "error",
              std::string(kSkillId) + ": weld_type '" + in.weld_type +
              "' not in {tack, full, lap, butt}");
    }

    if (in.weld_type == "tack" && in.joint_count > 50) {
        r.add("DFM-WELD-TACK", "info",
              std::string(kSkillId) + ": tack welds for " +
              std::to_string(in.joint_count) +
              " joints — verify these are not structural splices");
    }

    if (in.joint_count > 500) {
        r.add("DFM-WELD-BATCH", "info",
              std::string(kSkillId) + ": joint_count " +
              std::to_string(in.joint_count) +
              " > 500 — stage in multiple batches for QA traceability");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "weld_rebar DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "weld_type",   in.weld_type },
        { "joint_count", in.joint_count },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "weld_type",         in.weld_type },
        { "joint_count",       in.joint_count },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    // Tack welds are typically MIG/stick lugs; full structural joints use
    // SMAW with preheat under AWS D1.4.
    tooling.tool_type         = (in.weld_type == "tack") ? "mig_torch" : "smaw_electrode";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "e7018_electrode";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Tack: ~5 s/joint.  Full/lap/butt: ~45 s/joint.
    const double per_joint_s = (in.weld_type == "tack") ? 5.0 : 45.0;
    tooling.est_cycle_time_s  = per_joint_s * static_cast<double>(in.joint_count);
    tooling.extra = {
        { "process",          "arc_weld_rebar" },
        { "weld_type",        in.weld_type },
        { "joint_count",      in.joint_count },
        { "code_reference",   "AWS_D1.4" },
        { "process_category", "construction_weld" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::weld_rebar applied: type={} joints={}",
        in.weld_type, in.joint_count);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != std::string(kSkillId)) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::weld_rebar
