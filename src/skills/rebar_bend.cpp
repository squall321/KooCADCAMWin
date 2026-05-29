// @lat: [[engine/skills#rebar_bend]]

#include "rebar_bend.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::rebar_bend {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": dia_mm must be > 0 (got " +
              std::to_string(in.dia_mm) + ")");
    } else {
        if (in.dia_mm < 6.0) {
            r.add("DFM-REBAR-DIA", "error",
                  std::string(kSkillId) + ": dia_mm " +
                  std::to_string(in.dia_mm) +
                  " < 6 mm — below smallest structural rebar (#2 = 6.4 mm)");
        }
        if (in.dia_mm > 57.0) {
            r.add("DFM-REBAR-DIA", "error",
                  std::string(kSkillId) + ": dia_mm " +
                  std::to_string(in.dia_mm) +
                  " > 57 mm — above largest standard rebar (#18 = 57.3 mm)");
        }
    }

    if (in.bend_angle_deg <= 0.0 || in.bend_angle_deg > 180.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": bend_angle_deg must be in (0, 180] (got " +
              std::to_string(in.bend_angle_deg) + ")");
    }

    if (in.bend_radius_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": bend_radius_mm must be > 0");
    } else if (in.dia_mm > 0.0) {
        if (in.bend_radius_mm < 3.0 * in.dia_mm) {
            r.add("DFM-REBAR-RADIUS", "error",
                  std::string(kSkillId) + ": bend_radius_mm " +
                  std::to_string(in.bend_radius_mm) +
                  " < 3 × dia_mm (" + std::to_string(3.0 * in.dia_mm) +
                  ") — violates ACI 318 §25.3 minimum bend diameter");
        }
        if (in.bend_radius_mm > 12.0 * in.dia_mm) {
            r.add("DFM-REBAR-RADIUS", "info",
                  std::string(kSkillId) + ": bend_radius_mm " +
                  std::to_string(in.bend_radius_mm) +
                  " > 12 × dia_mm — gentle bend, capacity underused");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "rebar_bend DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double bend_ratio =
        in.dia_mm > 0.0 ? (in.bend_radius_mm / in.dia_mm) : 0.0;

    json params = {
        { "dia_mm",         in.dia_mm },
        { "bend_angle_deg", in.bend_angle_deg },
        { "bend_radius_mm", in.bend_radius_mm },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "dia_mm",            in.dia_mm },
        { "bend_angle_deg",    in.bend_angle_deg },
        { "bend_radius_mm",    in.bend_radius_mm },
        { "bend_ratio",        bend_ratio },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "rebar_bender";
    tooling.tool_dia_mm       = in.dia_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "hardened_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Hydraulic site bender: ~ 10 s per 90 deg bend, scaled by angle.
    tooling.est_cycle_time_s  = 10.0 * (in.bend_angle_deg / 90.0);
    tooling.extra = {
        { "process",          "cold_bend_rebar" },
        { "dia_mm",           in.dia_mm },
        { "bend_angle_deg",   in.bend_angle_deg },
        { "bend_radius_mm",   in.bend_radius_mm },
        { "bend_ratio",       bend_ratio },
        { "process_category", "construction_rebar" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::rebar_bend applied: dia={} angle={} radius={} ratio={:.2f}",
        in.dia_mm, in.bend_angle_deg, in.bend_radius_mm, bend_ratio);

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

}  // namespace koocadcam::skill::rebar_bend
