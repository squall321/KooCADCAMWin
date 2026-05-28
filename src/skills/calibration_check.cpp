// @lat: [[engine/skills#calibration_check]]

#include "calibration_check.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::calibration_check {

using nlohmann::json;

namespace {

// ISO 8601 "YYYY-MM-DD" comparison via lexicographic order.  This is exact
// because the format is fixed-width and big-endian.  Returns:
//   < 0 if a strictly before b,
//   = 0 if a == b,
//   > 0 if a strictly after b.
// Strings that are not 10 chars or are empty produce 0 (caller must guard).
int compareIsoDate(const std::string& a, const std::string& b)
{
    if (a.size() != 10 || b.size() != 10) return 0;
    return a.compare(b);
}

bool isKnownTraceability(const std::string& s)
{
    return s == "NIST" || s == "KRISS" || s == "PTB" ||
           s == "DKD"  || s == "JCSS";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.gauge_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": gauge_id must not be empty (traceability requirement)");
    }
    if (in.last_cal_date.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": last_cal_date must not be empty (ISO YYYY-MM-DD)");
    }
    if (in.due_date.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": due_date must not be empty (ISO YYYY-MM-DD)");
    }

    if (!in.last_cal_date.empty() && !in.due_date.empty()) {
        if (compareIsoDate(in.due_date, in.last_cal_date) < 0) {
            r.add("DFM-CAL-ORDER", "error",
                  std::string(kSkillId) + ": due_date '" + in.due_date +
                  "' is strictly before last_cal_date '" +
                  in.last_cal_date + "'");
        }
    }

    // Info-level: overdue check.
    if (!in.current_date.empty() && !in.due_date.empty()) {
        if (compareIsoDate(in.current_date, in.due_date) >= 0) {
            r.add("DFM-CAL-OVERDUE", "info",
                  std::string(kSkillId) + ": current_date '" +
                  in.current_date + "' >= due_date '" + in.due_date +
                  "' — gauge '" + in.gauge_id +
                  "' calibration is overdue, do not use for acceptance");
        }
    }

    // Info-level: unknown traceability chain.
    if (!isKnownTraceability(in.traceability)) {
        r.add("DFM-CAL-TRACE", "info",
              std::string(kSkillId) + ": traceability '" + in.traceability +
              "' not in {NIST, KRISS, PTB, DKD, JCSS} — verify the "
              "national-lab chain documentation");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "calibration_check DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const bool overdue =
        (!in.current_date.empty() && !in.due_date.empty() &&
         compareIsoDate(in.current_date, in.due_date) >= 0);

    json params = {
        { "gauge_id",      in.gauge_id },
        { "last_cal_date", in.last_cal_date },
        { "due_date",      in.due_date },
        { "traceability",  in.traceability },
        { "current_date",  in.current_date },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_inspection_only",   true },
        { "geometric_change",     false },
        { "gauge_id",             in.gauge_id },
        { "last_cal_date",        in.last_cal_date },
        { "due_date",             in.due_date },
        { "traceability",         in.traceability },
        { "current_date",         in.current_date },
        { "overdue",              overdue },
        { "inspection_method",    "calibration_verification" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "calibration_record";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Calibration verification — ~10 minutes typical.
    tooling.est_cycle_time_s  = 600.0;
    tooling.extra["inspection_type"] = "calibration_check";
    tooling.extra["overdue"]         = overdue;
    tooling.extra["traceability"]    = in.traceability;
    tooling.extra["dfm_findings"]    = findings;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::calibration_check applied: gauge={} last={} due={} trace={}",
        in.gauge_id, in.last_cal_date, in.due_date, in.traceability);

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

}  // namespace koocadcam::skill::calibration_check
