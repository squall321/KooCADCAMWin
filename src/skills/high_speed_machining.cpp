// @lat: [[engine/skills#high_speed_machining]]

#include "high_speed_machining.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::high_speed_machining {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.sub_skill_id.empty()) {
        r.add("DFM-INPUT", "error",
              "high_speed_machining: sub_skill_id must be non-empty");
    }

    // Spindle RPM envelope: HSM regime is 10 000 – 60 000 rpm.
    if (in.spindle_rpm < 10000.0) {
        r.add("DFM-HSM-RPM", "error",
              "high_speed_machining: spindle_rpm " +
              std::to_string(in.spindle_rpm) +
              " < 10000 rpm — below HSM regime; use conventional milling");
    }
    if (in.spindle_rpm > 60000.0) {
        r.add("DFM-HSM-RPM", "error",
              "high_speed_machining: spindle_rpm " +
              std::to_string(in.spindle_rpm) +
              " > 60000 rpm — above typical HSM spindle limit");
    }

    // Radial engagement ratio (ae/D).  Must be in (0, 0.10] to retain
    // chip-thinning advantage.
    if (in.radial_engagement_ratio <= 0.0) {
        r.add("DFM-HSM-RAE", "error",
              "high_speed_machining: radial_engagement_ratio must be > 0");
    }
    if (in.radial_engagement_ratio > 0.10) {
        r.add("DFM-HSM-RAE", "error",
              "high_speed_machining: radial_engagement_ratio " +
              std::to_string(in.radial_engagement_ratio) +
              " > 0.10 — chip thinning lost; conventional regime");
    }

    // Feed per tooth sanity.
    if (in.feed_per_tooth_mm < 0.01 || in.feed_per_tooth_mm > 0.20) {
        r.add("DFM-HSM-FEED", "warning",
              "high_speed_machining: feed_per_tooth_mm " +
              std::to_string(in.feed_per_tooth_mm) +
              " outside typical HSM [0.01, 0.20] mm/tooth");
    }

    if (in.tool_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "high_speed_machining: tool_dia_mm must be > 0");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Metadata-only wrapper.  Clone the workpiece geometry unchanged and stamp
// the HSM strategy signature; expansion to the real sub-skill is done by
// the process planner at codegen time.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "high_speed_machining DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Surface speed: v = π · D · rpm / 1000 (m/min); convert to sfm × 3.28.
    const double v_m_min = M_PI * in.tool_dia_mm * in.spindle_rpm / 1000.0;
    const double v_sfm   = v_m_min * 3.28084;

    json params = {
        { "sub_skill_id",            in.sub_skill_id },
        { "spindle_rpm",             in.spindle_rpm },
        { "radial_engagement_ratio", in.radial_engagement_ratio },
        { "feed_per_tooth_mm",       in.feed_per_tooth_mm },
        { "axial_doc_mm",            in.axial_doc_mm },
        { "tool_dia_mm",             in.tool_dia_mm },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "sub_skill_id",            in.sub_skill_id },
        { "spindle_rpm",             in.spindle_rpm },
        { "radial_engagement_ratio", in.radial_engagement_ratio },
        { "feed_per_tooth_mm",       in.feed_per_tooth_mm },
        { "axial_doc_mm",            in.axial_doc_mm },
        { "tool_dia_mm",             in.tool_dia_mm },
        { "strategy",                "hsm" },
        { "geometry_changed",        false },
        { "surface_speed_m_min",     v_m_min },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill_hsm";
    tooling.tool_dia_mm       = in.tool_dia_mm;
    tooling.tool_length_mm    = std::max(8.0, in.tool_dia_mm * 3.0);
    tooling.tool_material     = "carbide";  // typically AlTiN-coated carbide
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = v_sfm;
    tooling.feed_per_tooth_mm = in.feed_per_tooth_mm;
    tooling.stock_removed_mm3 = 0.0;        // wrapper itself doesn't cut
    tooling.est_cycle_time_s  = 0.0;        // sub-skill carries the time
    tooling.extra["strategy"]              = "hsm";
    tooling.extra["spindle_rpm"]           = in.spindle_rpm;
    tooling.extra["radial_engagement_ratio"]= in.radial_engagement_ratio;
    tooling.extra["axial_doc_mm"]          = in.axial_doc_mm;
    tooling.extra["sub_skill_id"]          = in.sub_skill_id;
    tooling.extra["dfm_findings"]          = findings;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::high_speed_machining applied: rpm={} ae/D={} fz={}",
                  in.spindle_rpm, in.radial_engagement_ratio, in.feed_per_tooth_mm);

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

}  // namespace koocadcam::skill::high_speed_machining
