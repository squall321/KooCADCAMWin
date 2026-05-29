// @lat: [[engine/skills#screen_print]]

#include "screen_print.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::screen_print {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.mesh_count <= 0) {
        r.add("DFM-INPUT", "error",
              "screen_print mesh_count must be > 0 (got " +
              std::to_string(in.mesh_count) + ")");
    }
    if (in.impressions <= 0) {
        r.add("DFM-INPUT", "error",
              "screen_print impressions must be > 0 (got " +
              std::to_string(in.impressions) + ")");
    }
    if (in.ink_color.empty()) {
        r.add("DFM-INPUT", "error",
              "screen_print ink_color must be non-empty");
    }

    if (in.mesh_count > 0 && in.mesh_count < 60) {
        r.add("DFM-MESH-COARSE", "info",
              "screen_print mesh_count " + std::to_string(in.mesh_count) +
              " < 60 — coarse mesh; heavy ink lay-down");
    }
    if (in.mesh_count > 420) {
        r.add("DFM-MESH-FINE", "info",
              "screen_print mesh_count " + std::to_string(in.mesh_count) +
              " > 420 — specialty fine mesh; verify ink particle size");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "screen_print DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "mesh_count",  in.mesh_count },
        { "ink_color",   in.ink_color },
        { "impressions", in.impressions },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "mesh_count",       in.mesh_count },
        { "ink_color",        in.ink_color },
        { "impressions",      in.impressions },
        { "process_family",   "surface_decoration" },
        { "geometry_changed", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "screen_press";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "polyester_mesh";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 4.0 * static_cast<double>(in.impressions);
    tooling.extra = {
        { "process",          "screen_printing" },
        { "mesh_count",       in.mesh_count },
        { "ink_color",        in.ink_color },
        { "impressions",      in.impressions },
        { "process_category", "printing" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::screen_print applied: mesh={} color={} imp={}",
                  in.mesh_count, in.ink_color, in.impressions);

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

}  // namespace koocadcam::skill::screen_print
