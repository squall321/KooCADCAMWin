// @lat: [[engine/skills#qc_auto]]

#include "qc_auto.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::qc_auto {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.vin.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": vin must be non-empty");
    } else if (in.vin.size() != 17) {
        r.add("DFM-QC-VIN", "info",
              std::string(kSkillId) + ": vin '" + in.vin +
              "' length " + std::to_string(in.vin.size()) +
              " != 17 — non-standard VIN");
    }

    if (in.inspection_count <= 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": inspection_count must be > 0 (got " +
              std::to_string(in.inspection_count) + ")");
    }
    if (in.pass_count < 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": pass_count must be >= 0 (got " +
              std::to_string(in.pass_count) + ")");
    }
    if (in.inspection_count > 0 && in.pass_count > in.inspection_count) {
        r.add("DFM-QC-COUNT", "error",
              std::string(kSkillId) + ": pass_count " +
              std::to_string(in.pass_count) +
              " exceeds inspection_count " +
              std::to_string(in.inspection_count));
    }
    if (in.inspection_count > 0
        && in.pass_count >= 0
        && in.pass_count < in.inspection_count) {
        r.add("DFM-QC-FAILS", "info",
              std::string(kSkillId) + ": " +
              std::to_string(in.inspection_count - in.pass_count) +
              " inspection items failed out of " +
              std::to_string(in.inspection_count));
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "qc_auto DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double passRate = (in.inspection_count > 0)
        ? static_cast<double>(in.pass_count) /
          static_cast<double>(in.inspection_count)
        : 0.0;

    json params = {
        { "vin",              in.vin },
        { "inspection_count", in.inspection_count },
        { "pass_count",       in.pass_count },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "vin",              in.vin },
        { "inspection_count", in.inspection_count },
        { "pass_count",       in.pass_count },
        { "pass_rate",        passRate },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "qc_auto_station";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // 4 s / inspection item + 30 s station handshake.
    tooling.est_cycle_time_s  = 30.0
                              + 4.0 * static_cast<double>(in.inspection_count);
    tooling.extra = {
        { "process",          "qc_auto" },
        { "vin",              in.vin },
        { "inspection_count", in.inspection_count },
        { "pass_count",       in.pass_count },
        { "pass_rate",        passRate },
        { "process_category", "automotive_assembly" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::qc_auto applied: vin={} inspections={} pass={}",
        in.vin, in.inspection_count, in.pass_count);

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

}  // namespace koocadcam::skill::qc_auto
