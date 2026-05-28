// @lat: [[engine/skills#force_feedback_scan]]

#include "force_feedback_scan.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>
#include <vector>

namespace koocadcam::skill::force_feedback_scan {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.max_force_n <= 0.0) {
        r.add("DFM-INPUT", "error",
              "force_feedback_scan max_force_n must be > 0 (got " +
              std::to_string(in.max_force_n) + ")");
    } else if (in.max_force_n > 10.0) {
        r.add("DFM-FFS-FORCE", "warning",
              "force_feedback_scan max_force_n " +
              std::to_string(in.max_force_n) +
              " > 10 N — high force, surface damage risk");
    } else if (in.max_force_n < 0.01) {
        r.add("DFM-FFS-LOWFORCE", "info",
              "force_feedback_scan max_force_n " +
              std::to_string(in.max_force_n) +
              " < 0.01 N — very low, limited to optical-grade compliance");
    }

    if (in.scan_resolution_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              "force_feedback_scan scan_resolution_um must be > 0 (got " +
              std::to_string(in.scan_resolution_um) + ")");
    } else if (in.scan_resolution_um > 250.0) {
        r.add("DFM-FFS-RES", "warning",
              "force_feedback_scan scan_resolution_um " +
              std::to_string(in.scan_resolution_um) +
              " > 250 µm — coarse, close to industrial limit");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "force_feedback_scan DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "max_force_n",        in.max_force_n },
        { "scan_resolution_um", in.scan_resolution_um },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_inspection_only",  true },
        { "geometric_change",    false },
        { "max_force_n",         in.max_force_n },
        { "scan_resolution_um",  in.scan_resolution_um },
        { "low_force_mode",      in.max_force_n < 0.1 },
        { "inspection_method",   "force_feedback_haptic_trace" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "haptic_stylus";
    tooling.tool_dia_mm       = 1.0;     // typical sapphire / ruby tip
    tooling.tool_length_mm    = 50.0;
    tooling.tool_material     = "sapphire";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Continuous sweep — runtime ~ inversely proportional to resolution.
    // Reference: 10 µm step ≈ 60 s for a small feature; coarser → faster.
    const double refTime = 60.0;
    const double scale   = (in.scan_resolution_um > 0.0)
                         ? (10.0 / in.scan_resolution_um) : 1.0;
    tooling.est_cycle_time_s  = std::max(15.0, refTime * scale);
    tooling.extra["dfm_findings"]         = findings;
    tooling.extra["machining_constraint"] =
        "inspection — closed-loop force-feedback haptic scan, "
        "no material removal";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::force_feedback_scan applied: F_max={} N res={} µm",
                  in.max_force_n, in.scan_resolution_um);

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

}  // namespace koocadcam::skill::force_feedback_scan
