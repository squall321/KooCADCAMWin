// @lat: [[engine/skills#fiber_draw]]

#include "fiber_draw.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace koocadcam::skill::fiber_draw {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.final_dia_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": final_dia_um must be > 0 (got " +
              std::to_string(in.final_dia_um) + ")");
    } else if (in.final_dia_um < 50.0 || in.final_dia_um > 1000.0) {
        r.add("DFM-DRAW-DIA", "error",
              std::string(kSkillId) + ": final_dia_um " +
              std::to_string(in.final_dia_um) +
              " not in [50, 1000] µm (standard telecom + fiber-laser range)");
    }

    if (in.draw_speed_m_min <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": draw_speed_m_min must be > 0 (got " +
              std::to_string(in.draw_speed_m_min) + ")");
    } else if (in.draw_speed_m_min < 10.0 || in.draw_speed_m_min > 3000.0) {
        r.add("DFM-DRAW-SPEED", "info",
              std::string(kSkillId) + ": draw_speed_m_min " +
              std::to_string(in.draw_speed_m_min) +
              " outside typical [10, 3000] window — verify cooling-tower height");
    }

    if (in.draw_temp_c < 1900.0) {
        r.add("DFM-DRAW-TEMP-LOW", "error",
              std::string(kSkillId) + ": draw_temp_c " +
              std::to_string(in.draw_temp_c) +
              " < 1900 — silica viscosity too high to neck");
    } else if (in.draw_temp_c > 2200.0) {
        r.add("DFM-DRAW-TEMP-HIGH", "error",
              std::string(kSkillId) + ": draw_temp_c " +
              std::to_string(in.draw_temp_c) +
              " > 2200 — graphite element life + dopant volatilisation risk");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "fiber_draw DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Coating is realistic at intermediate speeds; ultra-fast draws often
    // bypass inline coating and post-coat the spool later.
    const bool coating_applied =
        (in.draw_speed_m_min >= 50.0 && in.draw_speed_m_min <= 2500.0);

    json params = {
        { "final_dia_um",     in.final_dia_um },
        { "draw_temp_c",      in.draw_temp_c },
        { "draw_speed_m_min", in.draw_speed_m_min },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "final_dia_um",     in.final_dia_um },
        { "draw_temp_c",      in.draw_temp_c },
        { "draw_speed_m_min", in.draw_speed_m_min },
        { "coating_applied",  coating_applied },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "draw_tower";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 20000.0;   // 20 m drop typical
    tooling.tool_material     = "graphite_susceptor";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;       // no removal — flow process
    // Rough estimate: 10 km of fiber at draw_speed (m/min).
    const double prod_km = 10.0;
    tooling.est_cycle_time_s  = (prod_km * 1000.0 / in.draw_speed_m_min) * 60.0;
    tooling.extra = {
        { "process",           "fiber_draw" },
        { "final_dia_um",      in.final_dia_um },
        { "draw_temp_c",       in.draw_temp_c },
        { "draw_speed_m_min",  in.draw_speed_m_min },
        { "coating_applied",   coating_applied },
        { "dfm_findings",      findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::fiber_draw applied: dia={}µm T={}°C speed={}m/min",
                  in.final_dia_um, in.draw_temp_c, in.draw_speed_m_min);

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

}  // namespace koocadcam::skill::fiber_draw
