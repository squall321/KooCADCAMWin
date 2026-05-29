// @lat: [[engine/skills#fabric_cutting]]

#include "fabric_cutting.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::fabric_cutting {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.pattern_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": pattern_id must be non-empty");
    }

    if (in.cut_length_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": cut_length_m must be > 0 (got " +
              std::to_string(in.cut_length_m) + ")");
    }

    if (in.waste_pct < 0.0 || in.waste_pct > 100.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": waste_pct must be in [0, 100] (got " +
              std::to_string(in.waste_pct) + ")");
    }

    if (in.waste_pct > 20.0 && in.waste_pct <= 100.0) {
        r.add("DFM-TEX-WASTE", "info",
              std::string(kSkillId) + ": waste_pct " +
              std::to_string(in.waste_pct) +
              " > 20 — consider re-nesting marker to improve yield");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "fabric_cutting DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double usable_length_m = in.cut_length_m * (1.0 - in.waste_pct / 100.0);

    json params = {
        { "pattern_id",   in.pattern_id },
        { "cut_length_m", in.cut_length_m },
        { "waste_pct",    in.waste_pct },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "pattern_id",        in.pattern_id },
        { "cut_length_m",      in.cut_length_m },
        { "waste_pct",         in.waste_pct },
        { "usable_length_m",   usable_length_m },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "fabric_cutter";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "hardened_steel_blade";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Automatic cutter ~ 0.5 m/s along the marker.
    tooling.est_cycle_time_s  = in.cut_length_m > 0.0 ? in.cut_length_m * 2.0 : 0.0;
    tooling.extra = {
        { "process",          "fabric_cutting" },
        { "pattern_id",       in.pattern_id },
        { "cut_length_m",     in.cut_length_m },
        { "waste_pct",        in.waste_pct },
        { "usable_length_m",  usable_length_m },
        { "process_category", "textile" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::fabric_cutting applied: pattern={} length={}m waste={}%",
        in.pattern_id, in.cut_length_m, in.waste_pct);

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

}  // namespace koocadcam::skill::fabric_cutting
