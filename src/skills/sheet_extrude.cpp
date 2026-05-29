// @lat: [[engine/skills#sheet_extrude]]

#include "sheet_extrude.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::sheet_extrude {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.sheet_thickness_mm < 0.25) {
        r.add("DFM-SHEET-THICKNESS", "error",
              std::string(kSkillId) + ": sheet_thickness_mm " +
              std::to_string(in.sheet_thickness_mm) +
              " < 0.25 mm — use film_extrude for thinner gauges");
    } else if (in.sheet_thickness_mm > 30.0) {
        r.add("DFM-SHEET-THICKNESS", "error",
              std::string(kSkillId) + ": sheet_thickness_mm " +
              std::to_string(in.sheet_thickness_mm) +
              " > 30 mm — beyond 3-roll stack gauging envelope");
    }

    if (in.width_mm <= 0.0) {
        r.add("DFM-SHEET-WIDTH", "error",
              std::string(kSkillId) + ": width_mm must be > 0 (got " +
              std::to_string(in.width_mm) + ")");
    } else if (in.width_mm > 3000.0) {
        r.add("DFM-SHEET-WIDTH", "error",
              std::string(kSkillId) + ": width_mm " +
              std::to_string(in.width_mm) +
              " > 3000 — beyond commercial sheet-line slot-die width");
    }

    if (in.line_speed_m_min <= 0.0) {
        r.add("DFM-SHEET-SPEED", "error",
              std::string(kSkillId) + ": line_speed_m_min must be > 0 (got " +
              std::to_string(in.line_speed_m_min) + ")");
    } else if (in.line_speed_m_min > 100.0) {
        r.add("DFM-SHEET-SPEED", "error",
              std::string(kSkillId) + ": line_speed_m_min " +
              std::to_string(in.line_speed_m_min) +
              " > 100 — 3-roll stack cooling capacity exceeded");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Mass output = ρ × t × w × v   ρ ≈ 1.05e-6 kg/mm³ (ABS/HIPS)
    const double v_mm_h = in.line_speed_m_min * 60000.0;
    const double mass_output_kg_h =
        1.05e-6 * in.sheet_thickness_mm * in.width_mm * v_mm_h;

    json params = {
        { "sheet_thickness_mm",  in.sheet_thickness_mm },
        { "width_mm",            in.width_mm },
        { "line_speed_m_min",    in.line_speed_m_min },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "sheet_thickness_mm",   in.sheet_thickness_mm },
        { "width_mm",             in.width_mm },
        { "line_speed_m_min",     in.line_speed_m_min },
        { "mass_output_kg_h",     mass_output_kg_h },
        { "geometry_changed",     false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "sheet_slot_die_3roll";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = in.width_mm;
    tooling.tool_material     = "chrome_plated_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 3600.0;
    tooling.extra = {
        { "process",              "polymer_sheet_extrusion" },
        { "sheet_thickness_mm",   in.sheet_thickness_mm },
        { "width_mm",             in.width_mm },
        { "line_speed_m_min",     in.line_speed_m_min },
        { "mass_output_kg_h",     mass_output_kg_h },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::sheet_extrude applied: t={}mm w={}mm v={}m/min",
                  in.sheet_thickness_mm, in.width_mm, in.line_speed_m_min);

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

}  // namespace koocadcam::skill::sheet_extrude
