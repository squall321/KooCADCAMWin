// @lat: [[engine/skills#contact_probe_array_scan]]

#include "contact_probe_array_scan.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>
#include <vector>

namespace koocadcam::skill::contact_probe_array_scan {

using nlohmann::json;

namespace {

bool isKnownSweep(const std::string& s)
{
    return s == "raster" || s == "spiral" || s == "serpentine";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.probe_count < 1) {
        r.add("DFM-INPUT", "error",
              "contact_probe_array_scan probe_count must be >= 1 (got " +
              std::to_string(in.probe_count) + ")");
    } else if (in.probe_count > 256) {
        r.add("DFM-CPA-COUNT", "warning",
              "contact_probe_array_scan probe_count " +
              std::to_string(in.probe_count) +
              " > 256 — impractical fixture mass / calibration");
    }

    if (in.probe_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "contact_probe_array_scan probe_dia_mm must be > 0 (got " +
              std::to_string(in.probe_dia_mm) + ")");
    } else if (in.probe_dia_mm < 0.5) {
        r.add("DFM-CPA-DIA", "warning",
              "contact_probe_array_scan probe_dia_mm " +
              std::to_string(in.probe_dia_mm) +
              " < 0.5 mm — fragile ruby tip, high breakage risk");
    }

    if (!isKnownSweep(in.sweep_pattern)) {
        r.add("DFM-CPA-SWEEP", "info",
              "contact_probe_array_scan sweep_pattern '" + in.sweep_pattern +
              "' not in known set (raster|spiral|serpentine)");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "contact_probe_array_scan DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "probe_count",   in.probe_count },
        { "probe_dia_mm",  in.probe_dia_mm },
        { "sweep_pattern", in.sweep_pattern },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_inspection_only",  true },
        { "geometric_change",    false },
        { "probe_count",         in.probe_count },
        { "probe_dia_mm",        in.probe_dia_mm },
        { "sweep_pattern",       in.sweep_pattern },
        { "parallel_acquisition", in.probe_count > 1 },
        { "inspection_method",   "contact_probe_array_sweep" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "probe_array";
    tooling.tool_dia_mm       = in.probe_dia_mm;
    tooling.tool_length_mm    = 50.0;
    tooling.tool_material     = "ruby";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Cycle: parallel probes drop per cycle (~ 2 s).  Sweep ~ 100 cycles / part.
    // More probes per array → more parallel coverage per drop.
    const double cycles = 100.0;
    const double secPerCycle = 2.0;
    tooling.est_cycle_time_s  = std::max(20.0, cycles * secPerCycle);
    tooling.extra["dfm_findings"]         = findings;
    tooling.extra["machining_constraint"] =
        "inspection — multi-probe contact scan, no material removal";
    tooling.extra["probe_count"]          = in.probe_count;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::contact_probe_array_scan applied: N={} dia={} sweep={}",
                  in.probe_count, in.probe_dia_mm, in.sweep_pattern);

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

}  // namespace koocadcam::skill::contact_probe_array_scan
