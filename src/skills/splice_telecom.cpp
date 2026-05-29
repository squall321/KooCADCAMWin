// @lat: [[engine/skills#splice_telecom]]

#include "splice_telecom.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::splice_telecom {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.splice_count <= 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": splice_count must be > 0 (got " +
              std::to_string(in.splice_count) + ")");
    } else if (in.splice_count > 50) {
        r.add("DFM-SPLICE-COUNT", "warning",
              std::string(kSkillId) + ": splice_count " +
              std::to_string(in.splice_count) +
              " exceeds 50 — accumulated link loss may breach budget");
    }

    if (in.loss_db_per_splice < 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": loss_db_per_splice must be >= 0 (got " +
              std::to_string(in.loss_db_per_splice) + ")");
    } else if (in.loss_db_per_splice > 0.5) {
        r.add("DFM-SPLICE-LOSS", "warning",
              std::string(kSkillId) + ": loss_db_per_splice " +
              std::to_string(in.loss_db_per_splice) +
              " exceeds 0.5 dB — fusion target is < 0.1 dB; verify cleave quality");
    }

    if (in.splice_method.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": splice_method must be non-empty");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "splice_telecom DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double total_loss =
        static_cast<double>(in.splice_count) * in.loss_db_per_splice;

    json params = {
        { "splice_count",       in.splice_count },
        { "loss_db_per_splice", in.loss_db_per_splice },
        { "splice_method",      in.splice_method },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "splice_count",       in.splice_count },
        { "loss_db_per_splice", in.loss_db_per_splice },
        { "total_loss_db",      total_loss },
        { "splice_method",      in.splice_method },
        { "geometry_changed",   false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = (in.splice_method == "fusion")
                                  ? "fusion_splicer"
                                  : "splice_kit";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Fusion ~ 90 s per splice; mechanical ~ 180 s.
    const double sec_per_splice =
        (in.splice_method == "fusion") ? 90.0 : 180.0;
    tooling.est_cycle_time_s  =
        sec_per_splice * static_cast<double>(in.splice_count);
    tooling.extra = {
        { "process",          "telecom_splice" },
        { "splice_count",     in.splice_count },
        { "total_loss_db",    total_loss },
        { "splice_method",    in.splice_method },
        { "process_category", "telecom_install" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::splice_telecom applied: count={} loss/splice={}dB method={}",
        in.splice_count, in.loss_db_per_splice, in.splice_method);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata-only: leaves no geometric trace on the workpiece.

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

}  // namespace koocadcam::skill::splice_telecom
