// @lat: [[engine/skills#gravure_print]]

#include "gravure_print.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::gravure_print {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.cylinder_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "gravure_print cylinder_dia_mm must be > 0 (got " +
              std::to_string(in.cylinder_dia_mm) + ")");
    }
    if (in.line_speed_m_min <= 0.0) {
        r.add("DFM-INPUT", "error",
              "gravure_print line_speed_m_min must be > 0 (got " +
              std::to_string(in.line_speed_m_min) + ")");
    }
    if (in.depth_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              "gravure_print depth_um must be > 0 (got " +
              std::to_string(in.depth_um) + ")");
    }

    if (in.cylinder_dia_mm > 0.0 && in.cylinder_dia_mm < 100.0) {
        r.add("DFM-CYL-SMALL", "info",
              "gravure_print cylinder_dia_mm " +
              std::to_string(in.cylinder_dia_mm) +
              " < 100 — short repeat; verify press compatibility");
    }
    if (in.cylinder_dia_mm > 800.0) {
        r.add("DFM-CYL-LARGE", "info",
              "gravure_print cylinder_dia_mm " +
              std::to_string(in.cylinder_dia_mm) +
              " > 800 — heavy cylinder; check handling rating");
    }
    if (in.depth_um > 0.0 && in.depth_um < 5.0) {
        r.add("DFM-DEPTH-SHALLOW", "info",
              "gravure_print depth_um " + std::to_string(in.depth_um) +
              " < 5 — very light ink lay-down (highlight tones)");
    }
    if (in.depth_um > 60.0) {
        r.add("DFM-DEPTH-DEEP", "info",
              "gravure_print depth_um " + std::to_string(in.depth_um) +
              " > 60 — heavy ink lay-down; verify dryer capacity");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "gravure_print DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "cylinder_dia_mm",  in.cylinder_dia_mm },
        { "line_speed_m_min", in.line_speed_m_min },
        { "depth_um",         in.depth_um },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "cylinder_dia_mm",  in.cylinder_dia_mm },
        { "line_speed_m_min", in.line_speed_m_min },
        { "depth_um",         in.depth_um },
        { "process_family",   "rotogravure" },
        { "geometry_changed", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "gravure_cylinder";
    tooling.tool_dia_mm       = in.cylinder_dia_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "chrome_plated_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Reference: 60 s of run as nominal cycle estimate.
    tooling.est_cycle_time_s  = (in.line_speed_m_min > 0.0)
                                ? 60.0 : 0.0;
    tooling.extra = {
        { "process",          "rotogravure_printing" },
        { "cylinder_dia_mm",  in.cylinder_dia_mm },
        { "line_speed_m_min", in.line_speed_m_min },
        { "depth_um",         in.depth_um },
        { "process_category", "printing" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::gravure_print applied: cyl={}mm speed={}m/min depth={}um",
                  in.cylinder_dia_mm, in.line_speed_m_min, in.depth_um);

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

}  // namespace koocadcam::skill::gravure_print
