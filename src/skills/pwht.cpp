// @lat: [[engine/skills#pwht]]

#include "pwht.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::pwht {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.weld_id.empty()) {
        r.add("DFM-INPUT", "warning",
              "pwht: weld_id empty — defaulting to 'WLD-UNK'");
    }

    if (in.hold_temp_c <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string("pwht: hold_temp_c must be > 0 (got ") +
              std::to_string(in.hold_temp_c) + ")");
    } else if (in.hold_temp_c < 550.0 || in.hold_temp_c > 730.0) {
        r.add("DFM-PWHT-TEMP", "info",
              std::string("pwht: hold_temp_c ") +
              std::to_string(in.hold_temp_c) +
              " outside typical C-Mn steel band [550, 730] °C");
    }

    if (in.hold_time_h <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string("pwht: hold_time_h must be > 0 (got ") +
              std::to_string(in.hold_time_h) + ")");
    } else if (in.hold_time_h < 1.0) {
        r.add("DFM-PWHT-TIME", "info",
              std::string("pwht: hold_time_h ") +
              std::to_string(in.hold_time_h) +
              " < 1 h — may not soak through 25 mm section");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pwht DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string weld_id = in.weld_id.empty() ? "WLD-UNK" : in.weld_id;

    json params = {
        { "weld_id",      weld_id },
        { "hold_temp_c",  in.hold_temp_c },
        { "hold_time_h",  in.hold_time_h },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "weld_id",          weld_id },
        { "hold_temp_c",      in.hold_temp_c },
        { "hold_time_h",      in.hold_time_h },
        { "residual_stress",  "low" },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "pwht_furnace";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "refractory_lining";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Ramp ≤ 220 °C/h up + hold + ramp down ≤ 275 °C/h ≈ 2 × ramp + hold.
    tooling.est_cycle_time_s  = (in.hold_time_h * 3600.0) +
                                (2.0 * (in.hold_temp_c / 220.0) * 3600.0);
    tooling.extra = {
        { "process",     "post_weld_heat_treatment" },
        { "weld_id",     weld_id },
        { "hold_temp_c", in.hold_temp_c },
        { "hold_time_h", in.hold_time_h },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::pwht applied: weld={} T={}°C t={}h",
                  weld_id, in.hold_temp_c, in.hold_time_h);

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

}  // namespace koocadcam::skill::pwht
