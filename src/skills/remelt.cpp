// @lat: [[engine/skills#remelt]]

#include "remelt.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::remelt {

using nlohmann::json;

namespace {

bool isKnownRefining(const std::string& m)
{
    return m == "flux" || m == "vacuum" || m == "argon_purge" || m == "none";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.melt_temp_c <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": melt_temp_c must be > 0 (got " +
              std::to_string(in.melt_temp_c) + ")");
    }
    if (in.yield_pct <= 0.0 || in.yield_pct > 100.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": yield_pct must be in (0, 100] (got " +
              std::to_string(in.yield_pct) + ")");
    }

    if (!isKnownRefining(in.refining_method)) {
        r.add("DFM-REMELT-REFINE", "info",
              std::string(kSkillId) + ": refining_method '" + in.refining_method +
              "' unrecognised — expected flux | vacuum | argon_purge | none");
    }

    if (in.melt_temp_c > 0.0 && in.melt_temp_c < 200.0) {
        r.add("DFM-REMELT-TEMP", "info",
              std::string(kSkillId) + ": melt_temp_c " +
              std::to_string(in.melt_temp_c) +
              " < 200 °C — below liquidus of any common industrial metal");
    }
    if (in.melt_temp_c > 1800.0) {
        r.add("DFM-REMELT-TEMP", "info",
              std::string(kSkillId) + ": melt_temp_c " +
              std::to_string(in.melt_temp_c) +
              " > 1800 °C — exceeds typical induction-furnace ceiling, "
              "consider EAF or plasma furnace");
    }
    if (in.yield_pct > 0.0 && in.yield_pct < 70.0) {
        r.add("DFM-REMELT-YIELD", "warning",
              std::string(kSkillId) + ": yield_pct " +
              std::to_string(in.yield_pct) +
              " < 70% — low recovery, check dross / oxidation / loss-to-slag");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "remelt DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "melt_temp_c",     in.melt_temp_c },
        { "refining_method", in.refining_method },
        { "yield_pct",       in.yield_pct },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "melt_temp_c",      in.melt_temp_c },
        { "refining_method",  in.refining_method },
        { "yield_pct",        in.yield_pct },
        { "process_category", "eol_remelt" },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "furnace";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "refractory_lining";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Charge + ramp + soak + tap-out, ~ 90 min nominal.
    tooling.est_cycle_time_s  = 90.0 * 60.0;
    tooling.extra = {
        { "process",          "remelt_to_ingot" },
        { "melt_temp_c",      in.melt_temp_c },
        { "refining_method",  in.refining_method },
        { "yield_pct",        in.yield_pct },
        { "process_category", "eol_remelt" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::remelt applied: T={}°C refining={} yield={}%",
                  in.melt_temp_c, in.refining_method, in.yield_pct);

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

}  // namespace koocadcam::skill::remelt
