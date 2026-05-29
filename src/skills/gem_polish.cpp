// @lat: [[engine/skills#gem_polish]]

#include "gem_polish.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::gem_polish {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownClasses()
{
    static const std::unordered_set<std::string> s = {
        "excellent", "very_good", "good", "fair", "poor",
    };
    return s;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.gem_species.empty()) {
        r.add("DFM-INPUT", "error",
              "gem_polish gem_species must be non-empty");
    }

    if (in.target_polish_class.empty()) {
        r.add("DFM-INPUT", "error",
              "gem_polish target_polish_class must be non-empty");
    } else if (knownClasses().count(in.target_polish_class) == 0) {
        r.add("DFM-POLISH-CLASS", "error",
              "gem_polish target_polish_class '" + in.target_polish_class +
              "' not in {excellent, very_good, good, fair, poor}");
    }

    if (in.lap_grit <= 0) {
        r.add("DFM-LAP-GRIT", "error",
              "gem_polish lap_grit must be > 0 (got " +
              std::to_string(in.lap_grit) + ")");
    } else {
        if (in.lap_grit < 1200) {
            r.add("DFM-LAP-GRIT-LO", "info",
                  "gem_polish lap_grit " + std::to_string(in.lap_grit) +
                  " < 1200 — too coarse for finish polish, will not reach"
                  " excellent class");
        }
        if (in.lap_grit > 100000) {
            r.add("DFM-LAP-GRIT-HI", "info",
                  "gem_polish lap_grit " + std::to_string(in.lap_grit) +
                  " > 100000 — diminishing returns above 100k mesh");
        }
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "gem_polish DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Surface roughness Ra (nm) inverse-scales with grit.  Empirical:
    // Ra_nm ≈ 50000 / lap_grit (capped at 200).
    double ra_nm = (in.lap_grit > 0) ? (50000.0 / in.lap_grit) : 200.0;
    if (ra_nm > 200.0) ra_nm = 200.0;

    json params = {
        { "gem_species",         in.gem_species },
        { "target_polish_class", in.target_polish_class },
        { "lap_grit",            in.lap_grit },
    };

    json pattern = {
        { "kind",                 kSkillId },
        { "gem_species",          in.gem_species },
        { "target_polish_class",  in.target_polish_class },
        { "lap_grit",             in.lap_grit },
        { "expected_surface_ra_nm", ra_nm },
        { "process_family",       "gem_finishing" },
        { "geometry_changed",     false },
        { "is_process_only",      true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "polish_lap";
    tooling.tool_dia_mm       = 200.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "diamond_powder_on_tin";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Finer grit → longer dwell; rough estimate 30 s per facet × ~57 facets.
    tooling.est_cycle_time_s  = 1700.0 + (in.lap_grit / 1000.0) * 10.0;
    tooling.extra = {
        { "process",             "gem_polishing" },
        { "gem_species",         in.gem_species },
        { "target_polish_class", in.target_polish_class },
        { "lap_grit",            in.lap_grit },
        { "expected_surface_ra_nm", ra_nm },
        { "process_category",    "jewelry" },
        { "dfm_findings",        findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::gem_polish applied: species={} class={} grit={}",
                  in.gem_species, in.target_polish_class, in.lap_grit);

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

}  // namespace koocadcam::skill::gem_polish
