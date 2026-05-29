// @lat: [[engine/skills#marble_polish]]

#include "marble_polish.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::marble_polish {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.surface_area_m2 <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": surface_area_m2 must be > 0 (got " +
              std::to_string(in.surface_area_m2) + ")");
    }
    if (in.polish_grit_progression.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": polish_grit_progression must not be empty");
    }

    for (std::size_t i = 1; i < in.polish_grit_progression.size(); ++i) {
        if (in.polish_grit_progression[i] <= in.polish_grit_progression[i - 1]) {
            r.add("DFM-POLISH-GRIT-ORDER", "warning",
                  std::string(kSkillId) +
                  ": polish_grit_progression must be strictly increasing");
            break;
        }
    }

    if (!in.polish_grit_progression.empty()) {
        const int finalGrit = in.polish_grit_progression.back();
        if (finalGrit < 1500 && in.final_gloss_units > 80.0) {
            r.add("DFM-POLISH-FINAL-GRIT", "warning",
                  std::string(kSkillId) + ": final grit " +
                  std::to_string(finalGrit) +
                  " < 1500 cannot reach " +
                  std::to_string(in.final_gloss_units) +
                  " GU on marble");
        }
    }

    if (in.final_gloss_units < 0.0 || in.final_gloss_units > 100.0) {
        r.add("DFM-POLISH-GLOSS-RANGE", "info",
              std::string(kSkillId) + ": final_gloss_units " +
              std::to_string(in.final_gloss_units) +
              " outside [0, 100] standard GU range");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "marble_polish DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json gritsJson = json::array();
    for (int g : in.polish_grit_progression) gritsJson.push_back(g);

    json params = {
        { "surface_area_m2",         in.surface_area_m2 },
        { "polish_grit_progression", gritsJson },
        { "final_gloss_units",       in.final_gloss_units },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "surface_area_m2",         in.surface_area_m2 },
        { "polish_grit_progression", gritsJson },
        { "final_gloss_units",       in.final_gloss_units },
        { "pass_count",              in.polish_grit_progression.size() },
        { "geometry_changed",        false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "rotary_polisher";
    tooling.tool_dia_mm       = 250.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "diamond_resin_pad";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Rule of thumb: ~3 min per m^2 per pass for marble polishing.
    tooling.est_cycle_time_s  =
        in.surface_area_m2 *
        static_cast<double>(in.polish_grit_progression.size()) * 180.0;
    tooling.extra = {
        { "process",                 "stone_surface_polish" },
        { "surface_area_m2",         in.surface_area_m2 },
        { "polish_grit_progression", gritsJson },
        { "final_gloss_units",       in.final_gloss_units },
        { "pass_count",              in.polish_grit_progression.size() },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::marble_polish applied: area={}m^2 passes={} gloss={}GU",
                  in.surface_area_m2, in.polish_grit_progression.size(),
                  in.final_gloss_units);

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

}  // namespace koocadcam::skill::marble_polish
