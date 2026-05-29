// @lat: [[engine/skills#bullet_form]]

#include "bullet_form.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::bullet_form {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.caliber_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "bullet_form caliber_mm must be > 0");
    } else {
        if (in.caliber_mm < 3.0) {
            r.add("DFM-AMMO-CAL", "error",
                  "bullet_form caliber_mm " + std::to_string(in.caliber_mm) +
                  " < 3.0 mm — below smallest small-arms cartridge");
        }
        if (in.caliber_mm > 20.0) {
            r.add("DFM-AMMO-CAL", "error",
                  "bullet_form caliber_mm " + std::to_string(in.caliber_mm) +
                  " > 20.0 mm — exceeds small-arms classification");
        }
    }

    if (in.bullet_weight_grain <= 0.0) {
        r.add("DFM-INPUT", "error",
              "bullet_form bullet_weight_grain must be > 0");
    } else {
        if (in.bullet_weight_grain < 15.0) {
            r.add("DFM-AMMO-WEIGHT", "error",
                  "bullet_form bullet_weight_grain " +
                  std::to_string(in.bullet_weight_grain) +
                  " < 15 gr — below practical small-arms projectile mass");
        }
        if (in.bullet_weight_grain > 750.0) {
            r.add("DFM-AMMO-WEIGHT", "error",
                  "bullet_form bullet_weight_grain " +
                  std::to_string(in.bullet_weight_grain) +
                  " > 750 gr — exceeds small-arms classification");
        }
    }

    if (in.jacket_material.empty()) {
        r.add("DFM-INPUT", "warning",
              "bullet_form jacket_material empty — defaulting to copper");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "bullet_form DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string jacket =
        in.jacket_material.empty() ? "copper" : in.jacket_material;

    json params = {
        { "caliber_mm",          in.caliber_mm },
        { "bullet_weight_grain", in.bullet_weight_grain },
        { "jacket_material",     jacket },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "caliber_mm",          in.caliber_mm },
        { "bullet_weight_grain", in.bullet_weight_grain },
        { "jacket_material",     jacket },
        { "geometry_changed",    false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "bullet_seating_die";
    tooling.tool_dia_mm       = in.caliber_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 1.5;
    tooling.extra = {
        { "process",             "bullet_seating" },
        { "caliber_mm",          in.caliber_mm },
        { "bullet_weight_grain", in.bullet_weight_grain },
        { "jacket_material",     jacket },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::bullet_form applied: cal={} wt={} jacket={}",
                  in.caliber_mm, in.bullet_weight_grain, jacket);

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

}  // namespace koocadcam::skill::bullet_form
