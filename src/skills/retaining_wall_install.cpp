// @lat: [[engine/skills#retaining_wall_install]]

#include "retaining_wall_install.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::retaining_wall_install {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.wall_type != "gravity" &&
        in.wall_type != "reinforced" &&
        in.wall_type != "sheet") {
        r.add("DFM-WALL-TYPE", "error",
              std::string(kSkillId) + ": wall_type '" + in.wall_type +
              "' invalid — must be 'gravity', 'reinforced', or 'sheet'");
    }

    if (in.height_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": height_m must be > 0 (got " +
              std::to_string(in.height_m) + ")");
    }

    if (in.length_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": length_m must be > 0 (got " +
              std::to_string(in.length_m) + ")");
    }

    if (in.wall_type == "gravity" && in.height_m > 6.0) {
        r.add("DFM-WALL-HEIGHT", "info",
              std::string(kSkillId) + ": gravity wall height " +
              std::to_string(in.height_m) +
              " m > 6 — switch to reinforced soil (MSE) for stability");
    }

    if (in.wall_type == "sheet" && in.height_m > 8.0) {
        r.add("DFM-WALL-HEIGHT", "info",
              std::string(kSkillId) + ": sheet pile height " +
              std::to_string(in.height_m) +
              " m > 8 — add tie-back anchors or wales");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "retaining_wall_install DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string equip = (in.wall_type == "sheet")
                                  ? "vibratory_pile_driver"
                                  : "crawler_crane";

    json params = {
        { "wall_type", in.wall_type },
        { "height_m",  in.height_m },
        { "length_m",  in.length_m },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "wall_type",         in.wall_type },
        { "height_m",          in.height_m },
        { "length_m",          in.length_m },
        { "wall_face_area_m2", in.height_m * in.length_m },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = equip;
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Wall installation ~ 8 m of run per hour for sheet pile, ~ 4 m/h for
    // reinforced/gravity (slower lift + backfill).
    {
        const double rate_m_per_h = (in.wall_type == "sheet") ? 8.0 : 4.0;
        tooling.est_cycle_time_s  = (in.length_m / rate_m_per_h) * 3600.0;
    }
    tooling.extra = {
        { "process",          "retaining_wall_install" },
        { "wall_type",        in.wall_type },
        { "height_m",         in.height_m },
        { "length_m",         in.length_m },
        { "code_reference",   "FHWA_NHI_10_024" },
        { "process_category", "civil_earth_retention" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::retaining_wall_install applied: type={} H={}m L={}m",
        in.wall_type, in.height_m, in.length_m);

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

}  // namespace koocadcam::skill::retaining_wall_install
