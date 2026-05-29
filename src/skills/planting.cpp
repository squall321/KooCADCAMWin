// @lat: [[engine/skills#planting]]

#include "planting.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::planting {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.seed_type.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": seed_type must be non-empty");
    }

    if (in.seed_density_per_m2 <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": seed_density_per_m2 must be > 0 (got " +
              std::to_string(in.seed_density_per_m2) + ")");
    } else if (in.seed_density_per_m2 > 2000.0) {
        r.add("DFM-PLANT-DENSITY", "info",
              std::string(kSkillId) + ": seed_density_per_m2 " +
              std::to_string(in.seed_density_per_m2) +
              " > 2000 — overcrowding risk, verify");
    }

    if (in.planting_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": planting_depth_mm must be > 0 (got " +
              std::to_string(in.planting_depth_mm) + ")");
    } else if (in.planting_depth_mm > 200.0) {
        r.add("DFM-PLANT-DEPTH", "error",
              std::string(kSkillId) + ": planting_depth_mm " +
              std::to_string(in.planting_depth_mm) +
              " > 200 mm — seed too deep to germinate");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "planting DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "seed_type",           in.seed_type },
        { "seed_density_per_m2", in.seed_density_per_m2 },
        { "planting_depth_mm",   in.planting_depth_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "seed_type",           in.seed_type },
        { "seed_density_per_m2", in.seed_density_per_m2 },
        { "planting_depth_mm",   in.planting_depth_mm },
        { "geometry_changed",    false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "seed_drill";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = in.planting_depth_mm;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Mechanized drill ~ 1 h/ha — but we have no area here; use a constant.
    tooling.est_cycle_time_s  = 3600.0;
    tooling.extra = {
        { "process",          "crop_planting" },
        { "seed_type",        in.seed_type },
        { "seed_density_per_m2", in.seed_density_per_m2 },
        { "planting_depth_mm", in.planting_depth_mm },
        { "process_category", "agriculture" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::planting applied: seed={} density={} per m2 depth={}mm",
        in.seed_type, in.seed_density_per_m2, in.planting_depth_mm);

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

}  // namespace koocadcam::skill::planting
