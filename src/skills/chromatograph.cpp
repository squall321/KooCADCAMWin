// @lat: [[engine/skills#chromatograph]]

#include "chromatograph.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::chromatograph {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownMethods()
{
    static const std::unordered_set<std::string> s = {
        "HPLC", "GC", "LC-MS",
    };
    return s;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.method.empty() || knownMethods().count(in.method) == 0) {
        r.add("DFM-INPUT", "error",
              "chromatograph unknown method '" + in.method +
              "' (expected HPLC | GC | LC-MS)");
    }

    if (in.column_id.empty()) {
        r.add("DFM-INPUT", "error",
              "chromatograph column_id must not be empty");
    }

    if (in.runtime_min <= 0.0) {
        r.add("DFM-CHRO-TIME", "error",
              "chromatograph runtime_min must be > 0 (got " +
              std::to_string(in.runtime_min) + ")");
    } else if (in.runtime_min > 240.0) {
        r.add("DFM-CHRO-TIME", "info",
              "chromatograph runtime_min " +
              std::to_string(in.runtime_min) +
              " exceeds typical analytical envelope (>240 min)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "chromatograph DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "method",      in.method },
        { "column_id",   in.column_id },
        { "runtime_min", in.runtime_min },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "method",           in.method },
        { "column_id",        in.column_id },
        { "runtime_min",      in.runtime_min },
        { "process_family",   "analytical_chromatography" },
        { "geometry_changed", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    if      (in.method == "HPLC")  tooling.tool_type = "hplc_system";
    else if (in.method == "GC")    tooling.tool_type = "gc_system";
    else if (in.method == "LC-MS") tooling.tool_type = "lcms_system";
    else                            tooling.tool_type = "chromatograph";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_steel_316";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Run + equilibration overhead (~3 min).
    tooling.est_cycle_time_s  = (in.runtime_min + 3.0) * 60.0;
    tooling.extra = {
        { "process",          "analytical_chromatography" },
        { "method",           in.method },
        { "column_id",        in.column_id },
        { "runtime_min",      in.runtime_min },
        { "process_category", "laboratory" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::chromatograph applied: method={} col={} t={}min",
                  in.method, in.column_id, in.runtime_min);

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
        rf.matched_geometry = { { "source", "feature_history" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::chromatograph
