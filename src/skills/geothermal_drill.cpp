// @lat: [[engine/skills#geothermal_drill]]

#include "geothermal_drill.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::geothermal_drill {

using nlohmann::json;

namespace {

bool isKnownCompletion(const std::string& c)
{
    return c == "open_hole" || c == "slotted_liner" || c == "perforated_casing";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.well_depth_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": well_depth_m must be > 0 (got " +
              std::to_string(in.well_depth_m) + ")");
    } else if (in.well_depth_m < 500.0 || in.well_depth_m > 6000.0) {
        r.add("DFM-GEO-DEPTH", "info",
              std::string(kSkillId) + ": well_depth_m " +
              std::to_string(in.well_depth_m) +
              " outside typical geothermal envelope [500, 6000] m");
    }

    if (in.bottom_hole_temp_c <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              ": bottom_hole_temp_c must be > 0 (got " +
              std::to_string(in.bottom_hole_temp_c) + ")");
    } else if (in.bottom_hole_temp_c < 120.0) {
        r.add("DFM-GEO-TEMP", "info",
              std::string(kSkillId) + ": bottom_hole_temp_c " +
              std::to_string(in.bottom_hole_temp_c) +
              " < 120 °C — below threshold for economic flash-power generation");
    }

    if (!isKnownCompletion(in.completion_method)) {
        r.add("DFM-GEO-COMPL", "info",
              std::string(kSkillId) + ": completion_method '" +
              in.completion_method +
              "' not in {open_hole, slotted_liner, perforated_casing}");
    }

    if (in.well_depth_m > 0.0 && in.bottom_hole_temp_c > 0.0) {
        const double depth_km = in.well_depth_m / 1000.0;
        const double gradient = (in.bottom_hole_temp_c - 20.0) / depth_km;
        if (gradient < 15.0 || gradient > 90.0) {
            r.add("DFM-GEO-GRADIENT", "info",
                  std::string(kSkillId) +
                  ": implied geothermal gradient " +
                  std::to_string(gradient) +
                  " °C/km outside common range [15, 90] — verify BHT / depth");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "geothermal_drill DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double depth_km = in.well_depth_m / 1000.0;
    const double gradient =
        depth_km > 0.0 ? (in.bottom_hole_temp_c - 20.0) / depth_km : 0.0;

    json params = {
        { "well_depth_m",       in.well_depth_m },
        { "bottom_hole_temp_c", in.bottom_hole_temp_c },
        { "completion_method",  in.completion_method },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "well_depth_m",       in.well_depth_m },
        { "bottom_hole_temp_c", in.bottom_hole_temp_c },
        { "completion_method",  in.completion_method },
        { "implied_gradient_c_per_km", gradient },
        { "geometry_changed",   false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "rotary_drilling_rig";
    tooling.tool_dia_mm       = 311.0;   // ~12.25" production-section bit
    tooling.tool_length_mm    = in.well_depth_m * 1000.0;
    tooling.tool_material     = "PDC_tungsten_carbide";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;     // metadata-only, no Workpiece cutout
    // ~ 20 m / day spud-to-TD plus 30-day completion budget.
    tooling.est_cycle_time_s  =
        (in.well_depth_m / 20.0 + 30.0) * 86400.0;
    tooling.extra = {
        { "process",                "geothermal_drill" },
        { "completion_method",      in.completion_method },
        { "well_depth_m",           in.well_depth_m },
        { "bottom_hole_temp_c",     in.bottom_hole_temp_c },
        { "implied_gradient_c_per_km", gradient },
        { "process_category",       "geothermal_well_construction" },
        { "geometric_no_op",        true },
        { "dfm_findings",           findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::geothermal_drill applied: depth={}m BHT={}°C completion={}",
        in.well_depth_m, in.bottom_hole_temp_c, in.completion_method);

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

}  // namespace koocadcam::skill::geothermal_drill
