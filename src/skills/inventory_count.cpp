// @lat: [[engine/skills#inventory_count]]

#include "inventory_count.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::inventory_count {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.lot_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": lot_id must not be empty (traceability requirement)");
    }
    if (in.expected_qty <= 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": expected_qty must be > 0 (got " +
              std::to_string(in.expected_qty) + ")");
    }
    if (in.actual_qty < 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": actual_qty must be >= 0 (got " +
              std::to_string(in.actual_qty) + ")");
    }

    if (in.expected_qty > 0) {
        const double variance_pct =
            100.0 *
            (static_cast<double>(in.actual_qty - in.expected_qty) /
             static_cast<double>(in.expected_qty));
        const double abs_pct = std::fabs(variance_pct);

        if (variance_pct < -2.0) {
            r.add("DFM-IC-SHORTAGE", "info",
                  std::string(kSkillId) + ": variance_pct " +
                  std::to_string(variance_pct) +
                  "% — significant shortage");
        }
        if (variance_pct > 2.0) {
            r.add("DFM-IC-OVERAGE", "info",
                  std::string(kSkillId) + ": variance_pct " +
                  std::to_string(variance_pct) +
                  "% — significant overage");
        }
        if (abs_pct > 10.0) {
            r.add("DFM-IC-CRITICAL", "info",
                  std::string(kSkillId) + ": |variance_pct| " +
                  std::to_string(abs_pct) +
                  "% > 10 — escalate to physical audit");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "inventory_count DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double variance_pct =
        100.0 *
        (static_cast<double>(in.actual_qty - in.expected_qty) /
         static_cast<double>(in.expected_qty));

    const std::string disposition =
        (variance_pct < -10.0) ? "critical_shortage" :
        (variance_pct > 10.0)  ? "critical_overage"  :
        (variance_pct < -2.0)  ? "shortage"          :
        (variance_pct > 2.0)   ? "overage"           : "ok";

    json params = {
        { "lot_id",       in.lot_id },
        { "expected_qty", in.expected_qty },
        { "actual_qty",   in.actual_qty },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_inspection_only",   true },
        { "geometric_change",     false },
        { "lot_id",               in.lot_id },
        { "expected_qty",         in.expected_qty },
        { "actual_qty",           in.actual_qty },
        { "variance_qty",         in.actual_qty - in.expected_qty },
        { "variance_pct",         variance_pct },
        { "disposition",          disposition },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "count_record";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Cycle-count operator ~ 1 second per unit, min 60 s.
    tooling.est_cycle_time_s  =
        std::max(60.0, static_cast<double>(in.actual_qty));
    tooling.extra["inspection_type"] = "inventory_count";
    tooling.extra["disposition"]     = disposition;
    tooling.extra["variance_pct"]    = variance_pct;
    tooling.extra["dfm_findings"]    = findings;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::inventory_count applied: lot={} expected={} actual={} var={}%",
        in.lot_id, in.expected_qty, in.actual_qty, variance_pct);

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

}  // namespace koocadcam::skill::inventory_count
