// @lat: [[engine/skills#film_extrude]]

#include "film_extrude.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::film_extrude {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.film_thickness_um < 5.0) {
        r.add("DFM-FILM-THICKNESS", "error",
              std::string(kSkillId) + ": film_thickness_um " +
              std::to_string(in.film_thickness_um) +
              " < 5 µm — below stable cast-film draw window");
    } else if (in.film_thickness_um > 500.0) {
        r.add("DFM-FILM-THICKNESS", "error",
              std::string(kSkillId) + ": film_thickness_um " +
              std::to_string(in.film_thickness_um) +
              " > 500 µm — too thick for film (use sheet_extrude)");
    }

    if (in.width_mm <= 0.0) {
        r.add("DFM-FILM-WIDTH", "error",
              std::string(kSkillId) + ": width_mm must be > 0 (got " +
              std::to_string(in.width_mm) + ")");
    } else if (in.width_mm > 4000.0) {
        r.add("DFM-FILM-WIDTH", "error",
              std::string(kSkillId) + ": width_mm " +
              std::to_string(in.width_mm) +
              " > 4000 — beyond commercial cast-film slot-die width");
    }

    if (in.line_speed_m_min <= 0.0) {
        r.add("DFM-FILM-SPEED", "error",
              std::string(kSkillId) + ": line_speed_m_min must be > 0 (got " +
              std::to_string(in.line_speed_m_min) + ")");
    } else if (in.line_speed_m_min > 800.0) {
        r.add("DFM-FILM-SPEED", "error",
              std::string(kSkillId) + ": line_speed_m_min " +
              std::to_string(in.line_speed_m_min) +
              " > 800 — beyond chill-roll heat-transfer capability");
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

    // Polymer mean density ~ 0.95 g/cc → 0.95e-6 kg/mm³.
    // throughput (kg/h) = ρ × t_mm × w_mm × v_mm/h
    //   v_mm/h = line_speed_m_min × 1000 × 60
    const double t_mm   = in.film_thickness_um / 1000.0;
    const double v_mm_h = in.line_speed_m_min * 60000.0;
    const double mass_output_kg_h = 0.95e-6 * t_mm * in.width_mm * v_mm_h;

    json params = {
        { "film_thickness_um",  in.film_thickness_um },
        { "width_mm",           in.width_mm },
        { "line_speed_m_min",   in.line_speed_m_min },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "film_thickness_um",   in.film_thickness_um },
        { "width_mm",            in.width_mm },
        { "line_speed_m_min",    in.line_speed_m_min },
        { "melt_output_kg_h",    mass_output_kg_h },
        { "geometry_changed",    false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "cast_film_slot_die";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = in.width_mm;        // die lip width
    tooling.tool_material     = "chrome_plated_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 3600.0;
    tooling.extra = {
        { "process",            "cast_film_extrusion" },
        { "film_thickness_um",  in.film_thickness_um },
        { "width_mm",           in.width_mm },
        { "line_speed_m_min",   in.line_speed_m_min },
        { "melt_output_kg_h",   mass_output_kg_h },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::film_extrude applied: t={}µm w={}mm v={}m/min",
                  in.film_thickness_um, in.width_mm, in.line_speed_m_min);

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

}  // namespace koocadcam::skill::film_extrude
