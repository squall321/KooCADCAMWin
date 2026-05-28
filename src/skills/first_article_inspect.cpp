// @lat: [[engine/skills#first_article_inspect]]

#include "first_article_inspect.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::first_article_inspect {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.dimensions_total <= 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": dimensions_total must be > 0 (got " +
              std::to_string(in.dimensions_total) + ")");
    }
    if (in.dimensions_pass < 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": dimensions_pass must be >= 0 (got " +
              std::to_string(in.dimensions_pass) + ")");
    }
    if (in.dimensions_total > 0 && in.dimensions_pass > in.dimensions_total) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": dimensions_pass (" +
              std::to_string(in.dimensions_pass) +
              ") cannot exceed dimensions_total (" +
              std::to_string(in.dimensions_total) + ")");
    }
    if (in.features_checked < 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": features_checked must be >= 0 (got " +
              std::to_string(in.features_checked) + ")");
    }
    if (in.approval_signature.empty()) {
        r.add("DFM-FAI-APPROVAL", "error",
              std::string(kSkillId) +
              ": approval_signature must not be empty (FAI requires sign-off)");
    }

    // Info-level: incomplete FAI (still recordable for audit trail).
    if (in.dimensions_total > 0 && in.dimensions_pass < in.dimensions_total) {
        r.add("DFM-FAI-INCOMPLETE", "info",
              std::string(kSkillId) + ": dimensions_pass " +
              std::to_string(in.dimensions_pass) + "/" +
              std::to_string(in.dimensions_total) +
              " — first article did not meet every called dimension");
    }
    if (in.features_checked == 0 && in.dimensions_total > 0) {
        r.add("DFM-FAI-COVERAGE", "info",
              std::string(kSkillId) +
              ": features_checked == 0 with dimensions_total > 0 — "
              "verify the inspection coverage report is complete");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "first_article_inspect DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double pass_rate =
        (in.dimensions_total > 0)
            ? static_cast<double>(in.dimensions_pass) /
              static_cast<double>(in.dimensions_total)
            : 0.0;
    const bool fully_approved = (in.dimensions_pass == in.dimensions_total);

    json params = {
        { "features_checked",   in.features_checked },
        { "dimensions_pass",    in.dimensions_pass },
        { "dimensions_total",   in.dimensions_total },
        { "approval_signature", in.approval_signature },
        { "inspector_name",     in.inspector_name },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_inspection_only",  true },
        { "geometric_change",    false },
        { "features_checked",    in.features_checked },
        { "dimensions_pass",     in.dimensions_pass },
        { "dimensions_total",    in.dimensions_total },
        { "pass_rate",           pass_rate },
        { "fully_approved",      fully_approved },
        { "approval_signature",  in.approval_signature },
        { "inspector_name",      in.inspector_name },
        { "inspection_method",   "first_article" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "inspection_record";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // FAI is laborious: assume 60 s per dimension call-out.
    tooling.est_cycle_time_s  =
        60.0 * static_cast<double>(in.dimensions_total);
    tooling.extra["inspection_type"] = "first_article";
    tooling.extra["pass_rate"]       = pass_rate;
    tooling.extra["fully_approved"]  = fully_approved;
    tooling.extra["dfm_findings"]    = findings;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::first_article_inspect applied: features={} pass={}/{} signed='{}'",
        in.features_checked, in.dimensions_pass, in.dimensions_total,
        in.approval_signature);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata-only — replay any prior FAI from workpiece.features().

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

}  // namespace koocadcam::skill::first_article_inspect
