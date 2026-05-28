// @lat: [[engine/skills#in_process_inspect]]

#include "in_process_inspect.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::in_process_inspect {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.sample_size <= 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": sample_size must be > 0 (got " +
              std::to_string(in.sample_size) + ")");
    }
    if (in.accept_size < 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": accept_size must be >= 0 (got " +
              std::to_string(in.accept_size) + ")");
    }
    if (in.defect_count < 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": defect_count must be >= 0 (got " +
              std::to_string(in.defect_count) + ")");
    }
    if (in.sample_size > 0 && in.defect_count > in.sample_size) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": defect_count (" +
              std::to_string(in.defect_count) +
              ") cannot exceed sample_size (" +
              std::to_string(in.sample_size) + ")");
    }
    if (in.lot_number.empty()) {
        r.add("DFM-IPI-LOT", "error",
              std::string(kSkillId) +
              ": lot_number must not be empty (traceability requirement)");
    }

    // Info-level rolls
    if (in.defect_count > in.accept_size && in.sample_size > 0) {
        r.add("DFM-IPI-REJECT", "info",
              std::string(kSkillId) + ": defect_count " +
              std::to_string(in.defect_count) + " > accept_size " +
              std::to_string(in.accept_size) +
              " — lot fails sampling plan; recorded for hold/rework");
    }
    if (in.sample_size > 0 && in.sample_size < 5) {
        r.add("DFM-IPI-SMALL", "info",
              std::string(kSkillId) + ": sample_size " +
              std::to_string(in.sample_size) +
              " < 5 — limited statistical power, consider larger n");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "in_process_inspect DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const bool lot_accepted = (in.defect_count <= in.accept_size);
    const double defect_rate =
        (in.sample_size > 0)
            ? static_cast<double>(in.defect_count) /
              static_cast<double>(in.sample_size)
            : 0.0;

    json params = {
        { "lot_number",   in.lot_number },
        { "sample_size",  in.sample_size },
        { "accept_size",  in.accept_size },
        { "defect_count", in.defect_count },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_inspection_only",   true },
        { "geometric_change",     false },
        { "lot_number",           in.lot_number },
        { "sample_size",          in.sample_size },
        { "accept_size",          in.accept_size },
        { "defect_count",         in.defect_count },
        { "defect_rate",          defect_rate },
        { "lot_accepted",         lot_accepted },
        { "inspection_method",    "ansi_asq_z1_4_sampling" },
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
    // ~30 s per sampled part for measurement + record.
    tooling.est_cycle_time_s  = 30.0 * static_cast<double>(in.sample_size);
    tooling.extra["inspection_type"] = "in_process";
    tooling.extra["lot_accepted"]    = lot_accepted;
    tooling.extra["defect_rate"]     = defect_rate;
    tooling.extra["dfm_findings"]    = findings;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::in_process_inspect applied: lot={} n={} Ac={} defects={}",
        in.lot_number, in.sample_size, in.accept_size, in.defect_count);

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

}  // namespace koocadcam::skill::in_process_inspect
