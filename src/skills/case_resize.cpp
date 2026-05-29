// @lat: [[engine/skills#case_resize]]

#include "case_resize.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::case_resize {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.case_dia_neck_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "case_resize case_dia_neck_mm must be > 0");
    } else {
        if (in.case_dia_neck_mm < 3.0) {
            r.add("DFM-AMMO-NECK", "error",
                  "case_resize case_dia_neck_mm " +
                  std::to_string(in.case_dia_neck_mm) +
                  " < 3.0 mm — below smallest small-arms neck");
        }
        if (in.case_dia_neck_mm > 25.0) {
            r.add("DFM-AMMO-NECK", "error",
                  "case_resize case_dia_neck_mm " +
                  std::to_string(in.case_dia_neck_mm) +
                  " > 25.0 mm — exceeds small-arms classification");
        }
    }

    if (in.case_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "case_resize case_length_mm must be > 0");
    } else {
        if (in.case_length_mm < 5.0) {
            r.add("DFM-AMMO-CASELEN", "error",
                  "case_resize case_length_mm " +
                  std::to_string(in.case_length_mm) +
                  " < 5.0 mm — below smallest small-arms case");
        }
        if (in.case_length_mm > 100.0) {
            r.add("DFM-AMMO-CASELEN", "error",
                  "case_resize case_length_mm " +
                  std::to_string(in.case_length_mm) +
                  " > 100 mm — exceeds small-arms classification");
        }
    }

    if (in.anneal_temp_c < 350.0 || in.anneal_temp_c > 550.0) {
        r.add("DFM-AMMO-ANNEAL", "info",
              "case_resize anneal_temp_c " + std::to_string(in.anneal_temp_c) +
              " outside neck-anneal sweet spot 350–550 °C");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "case_resize DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "case_dia_neck_mm", in.case_dia_neck_mm },
        { "case_length_mm",   in.case_length_mm },
        { "anneal_temp_c",    in.anneal_temp_c },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "case_dia_neck_mm", in.case_dia_neck_mm },
        { "case_length_mm",   in.case_length_mm },
        { "anneal_temp_c",    in.anneal_temp_c },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "full_length_resize_die";
    tooling.tool_dia_mm       = in.case_dia_neck_mm;
    tooling.tool_length_mm    = in.case_length_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 3.5;
    tooling.extra = {
        { "process",          "case_resize_anneal" },
        { "case_dia_neck_mm", in.case_dia_neck_mm },
        { "case_length_mm",   in.case_length_mm },
        { "anneal_temp_c",    in.anneal_temp_c },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::case_resize applied: neck={} len={} anneal={}",
                  in.case_dia_neck_mm, in.case_length_mm, in.anneal_temp_c);

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

}  // namespace koocadcam::skill::case_resize
